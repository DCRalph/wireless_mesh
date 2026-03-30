# Mesh Sync API Specification (`src/Mesh.h`)

This document describes the current public API exposed by the mesh layer in `src/Mesh.h`. For mesh wire format, relay rules, constants, and runtime behavior, see [mesh.md](mesh.md). For transport primitives and the ESP-NOW `Wireless` implementation, see [wireless.md](wireless.md).

## Core types

- `SyncMode`
  - `SOLO`, `JOIN`, `HOST`, `AUTO`
- `DiscoveredDevice`
  - `deviceId`, `address`, `lastSeen`, `syncMode`
- `GroupAdvert`
  - `groupId`, `masterDeviceId`, `masterAddress`, `lastSeen`
- `GroupMember`
  - `deviceId`, `address`
- `GroupInfo`
  - `groupId`, `masterDeviceId`, `isMaster`, `members`, `timeSynced`, `timeOffset`
- `PropertySnapshot`
  - `key`, `ownerDeviceId`, `revision`, `payload`

## `SyncManager`

### Singleton and lifecycle

#### `static SyncManager *getInstance();`

- Returns the singleton mesh manager instance.

#### `void begin();`

- Marks the manager as started.
- Requires a transport to already be provided with `setTransport()`.
- If the transport exists but is not ready, calls `transport->begin()`.
- Registers the receive callback, derives the local device ID, seeds the frame sequence counter, and loads persisted sync mode preferences.

#### `void loop();`

- Calls `transport->loop()` when a transport is configured.
- Runs heartbeat transmission, discovery cleanup, group cleanup, member timeout checks, relay-cache cleanup, property cleanup, and periodic group/time/property maintenance.
- When not currently in a group, also drives automatic JOIN and AUTO behavior.

#### `void setTransport(ITransport *transport);`

- Installs the transport implementation used for mesh traffic.
- The repository includes `Wireless`, an ESP-NOW transport intended for ESP32-class targets.
- Transport details for `ITransport`, `TransportPacket`, transport-defined addressing, and `Wireless` are documented in [wireless.md](wireless.md).

#### `ITransport *getTransport() const;`

- Returns the current transport pointer.
- May return `nullptr` if no transport has been configured.

#### `void setModePersistence(std::function<uint8_t()> loadCb, std::function<void(uint8_t)> saveCb);`

- Registers callbacks used to load and save the current `SyncMode` as a raw byte.

#### `void setDeviceIdProvider(std::function<uint32_t()> provider);`

- Registers a callback that can provide a stable non-zero device ID.
- If the callback is absent or returns `0`, `begin()` falls back to a random ID.

```cpp
SyncManager *sync = SyncManager::getInstance();
sync->setTransport(&Wireless::instance());
sync->setModePersistence(loadMode, saveMode);
sync->setDeviceIdProvider(getStableDeviceId);
sync->begin();
```

### Discovery and group state

#### `const std::map<std::string, DiscoveredDevice> &getDiscoveredDevices() const;`

- Returns discovered devices keyed by the transport-address string used internally by `SyncManager`.

#### `std::vector<GroupAdvert> getDiscoveredGroups() const;`

- Returns the currently discovered group advertisements.

#### `const GroupInfo &getGroupInfo() const;`

- Returns the local view of the current group state.

#### `bool isInGroup() const;`

- Returns `true` when `currentGroup.groupId != 0`.

#### `bool isGroupMaster() const;`

- Returns whether the local node is the current group master.

#### `uint32_t getGroupId() const;`

- Returns the current group ID, or `0` when not in a group.

#### `uint32_t getDeviceId() const;`

- Returns the local device ID selected during `begin()`.

### Group actions

#### `void createGroup(uint32_t groupId = 0);`

- Creates a group and makes the local node the master.
- If `groupId == 0`, generates a random group ID.
- Inserts the local device into `currentGroup.members`.
- Marks the local clock as the master reference.
- Immediately sends both group announce and group info frames.

#### `void joinGroup(uint32_t groupId);`

- Leaves any current group first.
- Joins only if `groupId` exists in the discovered-group table.
- Sets the group master from the discovered advertisement.
- Inserts the local device into member state.
- Broadcasts a Join frame, requests time sync immediately, and force-reannounces locally owned properties.

#### `void leaveGroup();`

- If not currently in a group, does nothing.
- Broadcasts a Leave frame up to three times.
- Resets time-sync state.
- Clears current group state.
- Removes remote property snapshots while preserving locally owned property values.

```cpp
sync->createGroup();
sync->joinGroup(0x12345678);
sync->leaveGroup();
```

### Sync mode

#### `void setSyncMode(SyncMode mode);`

- Updates the operating mode and applies immediate transition logic:
  - `SOLO` leaves the current group if necessary
  - `HOST` creates a group immediately if needed
  - `JOIN` leaves only when the local node is currently the master
  - `AUTO` waits for nearby AUTO peers when not already in a group
- Saves the new mode via the persistence callback if one was registered.

#### `SyncMode getSyncMode() const;`

- Returns the current mode.

#### `String getSyncModeString();`

- Returns a printable label for the current mode.

#### `String getSyncModeString(SyncMode mode);`

- Returns a printable label for the supplied mode.

### Time API

#### `void requestTimeSync();`

- Broadcasts a time-sync request only when the local node is a non-master member of a group.

#### `bool isTimeSynced() const;`

- Returns `true` for a current group master.
- Otherwise returns whether the offset clock is locked.

#### `uint32_t getSyncedTime() const;`

- Returns the current mesh-synchronized time derived from the offset clock.

#### `int32_t getTimeOffset() const;`

- Returns the current signed offset in milliseconds.

#### `static uint32_t syncMillis();`

- Convenience helper that returns synchronized time when available, otherwise falls back to `millis()`.

### Relay API

#### `void setRelayEnabled(bool enabled);`

- Enables or disables relay forwarding.
- Disabling relay also clears the relay deduplication cache.
- New locally generated frames are built with `kFlagNoRelay` and `hopLimit = 1` while relay is disabled.

#### `bool isRelayEnabled() const;`

- Returns whether relay forwarding is currently enabled.

### Callback registration

#### `void setDeviceDiscoveredCallback(std::function<void(const DiscoveredDevice &)> cb);`

- Called when a previously unseen device heartbeat is first discovered.

#### `void setGroupFoundCallback(std::function<void(const GroupAdvert &)> cb);`

- Called when a previously unseen group advertisement is first discovered.

#### `void setGroupCreatedCallback(std::function<void(const GroupInfo &)> cb);`

- Called after the local node creates a group.

#### `void setGroupJoinedCallback(std::function<void(const GroupInfo &)> cb);`

- Called after the local node joins a discovered group.

#### `void setGroupLeftCallback(std::function<void()> cb);`

- Called after the local node leaves or after the current master disbands the group.

#### `void setTimeSyncCallback(std::function<void(uint32_t syncedTime)> cb);`

- Called when a time-sync response is accepted and the offset clock updates.

```cpp
sync->setGroupJoinedCallback([](const GroupInfo &g) {
  Serial.printf("Joined group 0x%08lx\n", static_cast<unsigned long>(g.groupId));
});
```

### Property and event registration

#### `template <typename T> PropertyHandle<T> property(uint16_t key);`

- Registers a typed property and returns its handle.
- `T` must be trivially copyable.
- Registration fails if `sizeof(T)` exceeds the mesh property limit or conflicts with an existing registration for the same key.
- Can be called before or after `begin()`.

#### `RawPropertyHandle propertyRaw(uint16_t key, uint8_t maxLen);`

- Registers a raw property with an explicit maximum payload length.
- Fails if `maxLen` exceeds `MeshConstants::kMaxPropertyValueSize` or conflicts with an existing registration.

#### `bool unregisterProperty(uint16_t key);`

- Removes the property registration for that key.
- Also removes all stored property snapshots for that key.
- Returns `true` when the key existed.

#### `template <typename T> EventHandle<T> event(uint16_t channelId);`

- Registers a typed event channel and returns its handle.
- `T` must be trivially copyable.
- Registration fails if `sizeof(T)` exceeds the event payload limit or conflicts with an existing channel registration.

#### `EventHandle<void> event(uint16_t channelId);`

- Registers a zero-payload event channel and returns its handle.

#### `bool unregisterEvent(uint16_t channelId);`

- Removes the event-channel registration and its callbacks.
- Returns `true` when the channel existed.

#### `std::vector<PropertySnapshot> getDiscoveredProperties() const;`

- Returns all currently stored property snapshots, including the local owner and remote owners.

```cpp
auto tempProp = sync->property<float>(0x0101);
auto pingEvent = sync->event<void>(0x0201);
sync->unregisterProperty(0x0101);
sync->unregisterEvent(0x0201);
```

### Debug and manual helpers

#### `void printDeviceInfo();`

- Prints discovered-device information to serial output.

#### `void printGroupInfo();`

- Prints current group details to serial output.

#### `void printSyncModeInfo();`

- Prints sync-mode diagnostics to serial output.

#### `void sendHeartbeat();`

- Immediately broadcasts a discovery heartbeat.

#### `void sendGroupAnnounce();`

- Immediately broadcasts a group announce frame when the local node is the master.

#### `void sendGroupInfo();`

- Immediately broadcasts a full group member snapshot when the local node is the master.

## `PropertyHandle<T>`

`PropertyHandle<T>` is a typed wrapper for fixed-size property payloads.

#### `void set(const T &value) const;`

- Publishes a local property value.
- Internally no-ops if the handle is invalid.
- The underlying send succeeds only when the node is in a group and the property registration is valid.

#### `T get() const;`

- Returns the property value owned by the local device.
- Returns `T{}` if unavailable.

#### `T get(uint32_t deviceId) const;`

- Returns the property value for the requested owner.
- Returns `T{}` if unavailable or if the stored byte length does not match `sizeof(T)`.

#### `bool has(uint32_t deviceId) const;`

- Returns whether a value is currently stored for that owner.

#### `void onChange(std::function<void(uint32_t deviceId, T value)> cb) const;`

- Registers a typed callback for accepted local or remote updates on this key.
- If the handle is invalid or the callback is empty, it does nothing.

#### `uint16_t key() const;`

- Returns the property key bound to the handle.

#### `uint16_t revision() const;`

- Returns the local owner revision for this key, or `0` if unavailable.

#### `uint16_t revision(uint32_t deviceId) const;`

- Returns the revision for a specific owner, or `0` if unavailable.

```cpp
struct Reading {
  float tempC;
  float ph;
};

auto reading = sync->property<Reading>(0x1001);
reading.onChange([](uint32_t from, Reading r) {
  Serial.printf("from=%lu temp=%.2f ph=%.2f\n",
                static_cast<unsigned long>(from), r.tempC, r.ph);
});
reading.set({25.1f, 6.8f});
```

## `RawPropertyHandle`

`RawPropertyHandle` is the variable-length property wrapper.

#### `void set(const uint8_t *data, uint8_t len) const;`

- Publishes raw bytes for the local owner.
- Internally no-ops if the handle is invalid.

#### `uint8_t get(uint8_t *outBuf, uint8_t bufSize) const;`

- Reads the local owner value into `outBuf`.
- Returns bytes copied, or `0` if unavailable.

#### `uint8_t get(uint32_t deviceId, uint8_t *outBuf, uint8_t bufSize) const;`

- Reads the requested owner's value into `outBuf`.
- Returns the number of bytes copied, truncated to `bufSize`.

#### `bool has(uint32_t deviceId) const;`

- Returns whether the owner currently has a stored value.

#### `void onChange(std::function<void(uint32_t deviceId, const uint8_t *data, uint8_t len)> cb) const;`

- Registers a callback for accepted updates to this property key.

#### `uint16_t key() const;`

- Returns the property key.

#### `uint8_t maxLen() const;`

- Returns the configured maximum payload length for this registration.

#### `uint16_t revision() const;`

- Returns the local owner revision, or `0` if unavailable.

#### `uint16_t revision(uint32_t deviceId) const;`

- Returns the revision for the requested owner, or `0` if unavailable.

```cpp
uint8_t buf[32];
auto raw = sync->propertyRaw(0x1100, sizeof(buf));
const uint8_t written[] = {1, 2, 3, 4};
raw.set(written, sizeof(written));
uint8_t n = raw.get(buf, sizeof(buf));
```

## `EventHandle<T>`

`EventHandle<T>` is the typed wrapper for fixed-size event payloads.

#### `void emit(const T &payload) const;`

- Broadcasts a typed event payload to the current group.
- Internally no-ops if the handle is invalid.

#### `void onEvent(std::function<void(uint32_t fromDevice, T payload)> cb) const;`

- Registers a typed callback for accepted events on this channel.

#### `uint16_t channelId() const;`

- Returns the event channel ID.

```cpp
auto trigger = sync->event<uint8_t>(0x2201);
trigger.onEvent([](uint32_t from, uint8_t v) {
  Serial.printf("trigger from %lu value=%u\n",
                static_cast<unsigned long>(from), static_cast<unsigned>(v));
});
trigger.emit(1);
```

## `EventHandle<void>`

`EventHandle<void>` is the zero-payload event wrapper.

#### `void emit() const;`

- Emits the channel with an empty payload.

#### `void onEvent(std::function<void(uint32_t fromDevice)> cb) const;`

- Registers a callback for accepted no-payload events on this channel.

#### `uint16_t channelId() const;`

- Returns the event channel ID.

```cpp
auto ping = sync->event(0x2202);
ping.onEvent([](uint32_t from) {
  Serial.printf("ping from %lu\n", static_cast<unsigned long>(from));
});
ping.emit();
```

## Ingress and security notes

- Discovery and group-management traffic is not cryptographically authenticated.
- Property updates, property batches, events, time requests, and time responses are filtered against current group state before being accepted.
- For direct frames, the source transport address must match the current member record for the originating device.
- For relayed frames, source-address matching is intentionally skipped because the immediate sender is the relay node.
- Time-sync responses are accepted only from the current group master and only for the requesting device.
- Encryption and cryptographic authentication are not implemented by this layer.

---

For protocol framing, relay behavior, sync modes, and constants, see [mesh.md](mesh.md). For transport-layer setup and packet delivery behavior, see [wireless.md](wireless.md).

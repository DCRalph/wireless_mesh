# Mesh Sync System (`src/Mesh.*`) — Protocol v2

This document describes the mesh-layer protocol and runtime behavior implemented by `SyncManager`, `MeshProtocol`, `OffsetClock`, and `PropertyStore` in `src/Mesh.h` and `src/Mesh.cpp`.

For transport primitives and the ESP-NOW `Wireless` implementation, see [wireless.md](wireless.md).

## 1) Architecture

- Mesh layer:
  - `SyncManager` owns discovery, groups, time sync, relay, property replication, and event delivery.
  - `MeshProtocol` encodes and decodes the on-wire frame format.
  - `PropertyStore` stores owner-partitioned property snapshots.
  - `OffsetClock` tracks master-relative time offset for non-master members.
- Transport layer:
  - Mesh runs on top of the `ITransport` abstraction.
  - Mesh frames are carried inside `TransportPacket` payloads delivered by the selected transport.
  - The included ESP-NOW transport is documented separately in [wireless.md](wireless.md).

All mesh traffic is sent through `ITransport::sendPacket(const TransportPacket &, const TransportAddress &)`.

## 2) Transport framing

- Mesh traffic uses `TransportPacket.type == MeshConstants::kMeshPacketType`.
- `TransportPacket.len` is the number of valid bytes in the transport payload buffer.
- `SyncManager::handleTransportFrame()` rejects packets whose `len` exceeds the transport payload buffer or is shorter than `MeshProtocol::FrameHeader`.
- `MeshProtocol::decode()` validates:
  - packet type matches `kMeshPacketType`
  - packet length is large enough for the frame header
  - `version == MeshConstants::kProtocolVersion`
  - `sizeof(FrameHeader) + payloadLen <= len`

The transport envelope itself, including `TransportPacket`, `TransportAddress`, callback flow, and the included ESP-NOW transport behavior, is covered in [wireless.md](wireless.md).

## 3) Protocol v2 frame header

`MeshProtocol::FrameHeader` is a packed 15-byte header:

| Field | Type | Description |
|---|---|---|
| `version` | `uint8_t` | Always `MeshConstants::kProtocolVersion` |
| `kind` | `uint8_t` | Discovery=1, Group=2, Time=3, Property=5, Event=6 |
| `command` | `uint8_t` | Command within the selected message kind |
| `flags` | `uint8_t` | Includes `MeshConstants::kFlagNoRelay` |
| `originDeviceId` | `uint32_t` | Device that originally created the frame |
| `sequence` | `uint32_t` | Sequence number for deduplication |
| `hopLimit` | `uint8_t` | Remaining relay hops |
| `payloadLen` | `uint16_t` | Payload size in bytes |

## 4) Message payloads

### Discovery (`kind=1`)

- Heartbeat (`command=1`):
  - `HeartbeatHeader { uint32_t deviceId; uint8_t syncMode; }`
  - followed by one length-prefixed `TransportAddress`
  - `syncMode` is one of `SOLO`, `JOIN`, `HOST`, `AUTO`
- payload width now depends on the transport address length

### Group (`kind=2`)

- Announce (`command=1`):
  - `GroupAnnounceHeader { groupId, masterDeviceId }`
  - followed by one length-prefixed master `TransportAddress`
- Join (`command=2`):
  - `GroupJoinHeader { groupId, deviceId }`
  - followed by one length-prefixed member `TransportAddress`
- Info (`command=3`):
  - `GroupInfoHeader { groupId, masterDeviceId, memberCount }`
  - followed by `memberCount` entries of `GroupInfoMemberHeader { deviceId }` plus one length-prefixed member `TransportAddress`
- Leave (`command=4`):
  - `GroupLeaveCmd { groupId, deviceId }`

### Time (`kind=3`)

- Request (`command=1`):
  - `TimeRequestCmd { requestTimestamp, requesterDeviceId, groupId }`
- Response (`command=2`):
  - `TimeResponseCmd { requestTimestamp, masterTimestamp, requesterDeviceId, groupId }`

### Property (`kind=5`)

- Update (`command=1`):
  - exactly one `PropertyEntryHeader { key, revision, valueLen }`
  - followed by `valueLen` bytes of property data
  - property owner is `frame.originDeviceId`
- Batch (`command=2`):
  - first byte is `count`
  - followed by `count` entries using the same entry layout as Update
  - used by periodic and forced reannounce

### Event (`kind=6`)

- Emit (`command=1`):
  - `EventHeader { channelId, payloadLen }`
  - followed by `payloadLen` bytes

## 5) Sync modes

- `SOLO`
  - no group interaction
  - property and event traffic is effectively disabled because the node is not in a group
- `HOST`
  - creates a group immediately if not already hosting one
  - if currently a non-master member, leaves the current group and creates a new one
- `JOIN`
  - when not in a group, joins the first discovered group
  - if already a slave in a group, stays in that group
  - if currently the master of a group, leaves it
- `AUTO`
  - when not in a group, waits until at least two nearby AUTO devices exist within discovery timeout
  - the smallest discovered `deviceId` becomes the elected creator
  - the elected creator generates `kAutoGroupId | (random16)` and hosts that group
  - other AUTO nodes look for discovered group IDs whose high 16 bits match `MeshConstants::kAutoGroupIdMask` and join the first matching live group

## 6) Size limits

The key payload limits from `MeshConstants` are:

| Constant | Value | Meaning |
|---|---:|---|
| `kMaxMeshPayload` | 235 | Maximum mesh payload bytes after the 15-byte mesh header |
| `kMaxPropertyValueSize` | 230 | Maximum bytes in one property value |
| `kMaxEventPayloadSize` | 232 | Maximum bytes in one event payload |
| `kMaxPropertyEntries` | 200 | Maximum entries stored by `PropertyStore` |

## 7) Property model

Properties are partitioned by `(key, ownerDeviceId)`.

- Registration can happen before or after `SyncManager::begin()`.
- `property()` and `propertyRaw()` only succeed if the requested max length is compatible with the existing registration for that key.
- Local `set()` only succeeds when the node is currently in a group.
- Local updates increment a `uint16_t` revision, with revision `0` treated as unset.
- Incoming updates are applied only when:
  - the node is in a group
  - `frame.originDeviceId` is a current group member
  - the source transport address is accepted for that device, unless the frame is being relayed
  - the property key is registered locally
  - the incoming payload length fits the registered max length
  - the revision is newer according to `MeshProtocol::isNewerRevision()`
- Accepted updates trigger registered property callbacks.

### Property reannounce and cleanup

- `reannounceProperties(now, false)` runs periodically using `kPropertyReannounceIntervalMs`.
- `reannounceProperties(now, true)` bypasses the timer and is used after membership changes.
- Reannounce sends only properties owned by the local device.
- Reannounce batches entries into one or more Property Batch frames, respecting `kMaxMeshPayload`.
- Remote property entries are removed when they exceed `kPropertyStaleMs`.
- Cleanup runs every `kPropertyCleanupIntervalMs`.
- Leaving or disbanding a group clears remote entries but keeps locally owned properties.

## 8) Event model

- Events are transient broadcasts and are not stored.
- Event registration can happen before or after `begin()`.
- Local `emit()` only succeeds while in a group and only if the channel has been registered with a compatible payload size.
- Incoming events are delivered only when:
  - the node is in a group
  - `frame.originDeviceId` is a current group member
  - source validation passes for direct frames, or the frame is accepted as relayed
  - the channel is registered locally
  - the payload length matches the registered channel constraints
- Event callbacks fire only for registered listeners on that channel.

## 9) Group and time behavior

- Group masters immediately mark themselves time-synced and act as the time reference.
- A joining node:
  - leaves any existing group first
  - inserts itself into local member state
  - broadcasts a Join frame
  - requests time sync immediately
  - force-reannounces its local properties
- Group masters periodically broadcast:
  - group announcements every `kGroupAnnounceInterval`
  - group info snapshots every `kGroupInfoInterval`
- Non-master members periodically request time sync every `kTimeSyncInterval`.
- Time sync responses are accepted only when:
  - the node is currently a non-master group member
  - the response targets this device and current group
  - `originDeviceId` matches the current master
  - the origin device is still a known group member
  - the source transport address is valid for direct frames, or the frame is relayed
  - RTT is within `kMaxAcceptedRttMs`

## 10) Relay behavior

For every received frame, `SyncManager`:

1. Decodes and validates the mesh header.
2. Deduplicates on `(originDeviceId, sequence)` using the relay cache.
3. Forwards the frame if relay is enabled, `kFlagNoRelay` is not set, and `hopLimit > 1`.
4. Delivers the frame to local protocol handlers.

Important relay details:

- Locally created frames use `kDefaultRelayHopLimit` unless relay is disabled.
- `setRelayEnabled(false)` changes locally built packets by forcing:
  - `flags |= kFlagNoRelay`
  - `hopLimit = 1`
- Forwarded frames preserve `originDeviceId` and `sequence` but decrement `hopLimit`.
- For relayed frames, `sourceMatchesDevice()` intentionally skips strict source-address matching because the immediate sender is the relay node, not the original creator.

## 11) Constants reference

All current mesh tunables are defined in `MeshConstants` in `src/Mesh.h`.

| Constant | Value | Description |
|---|---:|---|
| `kMeshPacketType` | `0xBEEF` | Transport packet type used for mesh frames |
| `kProtocolVersion` | `2` | Mesh frame version |
| `kFlagNoRelay` | `0x01` | Frame flag that disables relay forwarding |
| `kMaxMeshPayload` | `235` | Maximum payload bytes per mesh frame |
| `kMaxEventPayloadSize` | `232` | Maximum event payload bytes |
| `kMaxPropertyValueSize` | `230` | Maximum property value bytes |
| `kMaxPropertyEntries` | `200` | Maximum property-store entries |
| `kPropertyReannounceIntervalMs` | `5000` | Periodic property republish interval |
| `kPropertyCleanupIntervalMs` | `10000` | Property cleanup cadence |
| `kPropertyStaleMs` | `30000` | Age threshold for remote property eviction |
| `kHeartbeatInterval` | `1000` | Heartbeat transmit interval |
| `kDiscoveryTimeout` | `6000` | Discovery entry timeout |
| `kGroupAnnounceInterval` | `2000` | Group announce interval |
| `kGroupDiscoveryTimeout` | `6000` | Group advertisement timeout |
| `kGroupInfoInterval` | `2000` | Group info interval |
| `kGroupMemberTimeout` | `8000` | Group member timeout |
| `kAutoGroupId` | `0xA1100000` | AUTO group ID base |
| `kAutoGroupIdMask` | `0xFFFF0000` | Mask used to recognize AUTO groups |
| `kTimeSyncInterval` | `10000` | Time sync request interval |
| `kDefaultRelayHopLimit` | `4` | Default hop count for new local frames |
| `kRelaySeenTtlMs` | `15000` | Relay dedup cache entry TTL |
| `kRelaySeenMax` | `512` | Maximum relay dedup cache size |
| `kMaxAcceptedRttMs` | `2000` | Maximum RTT accepted for time sync samples |

## 12) Debug and manual helpers

This repository does not contain the previously documented `SerialMenus/syncMenu` command set.

The currently exposed debug and manual mesh helpers on `SyncManager` are:

- `printDeviceInfo()`
- `printGroupInfo()`
- `printSyncModeInfo()`
- `sendHeartbeat()`
- `sendGroupAnnounce()`
- `sendGroupInfo()`

For the public API surface, see [api-spec.md](api-spec.md). For transport behavior and setup, see [wireless.md](wireless.md).

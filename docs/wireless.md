# Wireless Transport (`src/ITransport.h`, `src/Wireless.*`)

This document describes the transport layer used by the mesh stack: the generic `ITransport` interface plus the included ESP-NOW-based `Wireless` implementation.

For mesh protocol framing, relay, groups, properties, and events, see [mesh.md](mesh.md). For the mesh public API, see [api-spec.md](api-spec.md).

## 1) Transport overview

The transport layer is responsible for moving `TransportPacket` payloads between devices and reporting the sender address back to higher layers.

- `ITransport` defines the interface the mesh layer depends on.
- `Wireless` is the concrete ESP-NOW transport included in this repository.
- `SyncManager` uses `ITransport::sendPacket()` and `setReceiveCallback()` to send and receive mesh frames.

## 2) Shared transport primitives

`src/ITransport.h` defines the common types used by both `Wireless` and the mesh layer.

### `TransportAddress`

- Wraps a transport-defined address with an explicit `length` plus a byte buffer.
- `TransportAddress::fromBytes()` builds an address from any transport-specific byte sequence up to `TransportAddress::kMaxLength`.
- `TransportAddress::fromMac()` remains as a convenience for ESP-NOW-style 6-byte addresses.
- `isValid()`, `empty()`, equality operators, and `isFrom()` let higher layers compare addresses without assuming MAC layout.

### `TransportPacket`

Packed transport payload container:

| Field | Type | Description |
|---|---|---|
| `type` | `uint16_t` | Application- or protocol-defined packet type |
| `len` | `uint16_t` | Number of valid bytes in `data` |
| `data` | `uint8_t[250]` | Transport payload buffer |

Important limits:

- The transport payload buffer is 250 bytes.
- Mesh uses only part of that budget because it adds its own frame header and internal limits.
- `Wireless::recvCallback()` rejects incoming packets whose declared `len` exceeds `data[250]` or whose on-air length is shorter than `sizeof(type) + sizeof(len) + len`.

### `ITransport`

`ITransport` exposes the minimal contract needed by the mesh layer:

- `begin()`
  - bring up the transport
- `end()`
  - tear down the transport
- `loop()`
  - pump queued receive work
- `isReady()`
  - report readiness
- `localAddress()`
  - return the transport-defined address for this node
- `broadcastAddress()`
  - return the transport-defined broadcast destination
- `sendPacket(const TransportPacket &, const TransportAddress &)`
  - send one transport packet
- `setReceiveCallback(ReceiveCallback cb)`
  - install the callback that receives `(source, packet)`

The mesh layer does not depend on ESP-NOW-specific APIs directly. Any alternative transport must provide its own local address, broadcast address, and packet delivery behavior through this interface.

## 3) `Wireless` implementation

`Wireless` in `src/Wireless.h` and `src/Wireless.cpp` is the included ESP-NOW transport.

### Lifecycle

- `Wireless::instance()` returns the single supported ESP-NOW transport instance.
- `setup()` is a backward-compatible alias for `begin()`.
- `unSetup()` is a backward-compatible alias for `end()`.
- `isSetupDone()` and `isReady()` both expose whether the transport has been initialized.
- Independent `Wireless` construction is intentionally blocked because ESP-NOW callback trampolines dispatch through the singleton instance.

### `begin()`

`Wireless::begin()` performs the ESP-NOW setup sequence:

1. Returns immediately if already set up.
2. Unless `ESPNOW_NO_DISABLE_WIFI` is defined:
   - disconnects Wi-Fi
   - switches to `WIFI_STA`
   - temporarily enables promiscuous mode
   - sets the radio channel to `ESP_NOW_CHANNEL`
   - disables promiscuous mode again
3. Calls `esp_now_init()`.
4. Creates the RX queue if needed.
   - queue depth is `Wireless::kRxQueueDepth`, currently `16`
5. Resets the dropped-frame counter.
6. Registers ESP-NOW send and receive callback trampolines that dispatch through `Wireless::instance()`.
7. Adds the transport broadcast peer returned by `broadcastAddress()` on `WIFI_IF_STA`, with `encrypt = false`.

If queue creation fails, `begin()` deinitializes ESP-NOW and returns `false`.

### `end()`

`Wireless::end()`:

- removes the transport broadcast peer if it was configured
- unregisters ESP-NOW callbacks
- deinitializes ESP-NOW
- deletes the RX queue
- clears the setup flag

## 4) Receive path

ESP-NOW receive callbacks run through `Wireless::recvCallback()`:

- ignores frames if the queue is missing
- requires at least enough bytes for `type` and `len`
- validates declared payload length
- copies the packet and source MAC into a `WirelessFrame`
- pushes that frame into the FreeRTOS queue
- increments `droppedRxFrames_` if the queue is full

`Wireless::loop()` drains that queue and dispatches received frames in this order:

1. If `receiveCb` is set, converts the source ESP-NOW MAC into `TransportAddress` and invokes the generic transport callback.
2. If a type-specific callback exists in `onReceiveForCallbacks`, invokes it.
3. Otherwise, if `onReceiveOtherCb` is set, invokes the generic fallback callback.

This matters because `SyncManager` uses the generic `ReceiveCallback`, while applications that use `Wireless` directly can also register packet-type handlers.

## 5) Send path

`Wireless` exposes several send helpers:

- `send(const TransportPacket *p, const TransportAddress &peer)`
- `send(const uint8_t *data, uint16_t len, const TransportAddress &peer)`
- `send(const TransportPacket *p, const uint8_t *peer_addr)`
- `send(const uint8_t *data, uint16_t len, const uint8_t *peer_addr)`
- `send(WirelessFrame *frame)`
- `sendPacket(const TransportPacket &, const TransportAddress &)`

Current send behavior:

- Returns `-1` if the transport is not initialized.
- Address-based sends require `peer.length == ESP_NOW_ETH_ALEN`; otherwise the send is rejected.
- Non-broadcast sends use an ephemeral peer pattern:
  - if the peer is not already registered, `Wireless` adds it with `esp_now_add_peer()`
  - sends the packet with `esp_now_send()`
  - removes that peer after send completes or after a send failure
- Broadcast sends compare against `broadcastAddress()` and use the preconfigured broadcast peer without removing it afterward.
- `send(WirelessFrame *frame)` only sends frames whose direction is `PacketDirection::SEND`.

`sendPacket()` is the `ITransport` entry point and simply forwards to the transport’s packet send helper.

## 6) Status and callbacks

### Send status

- `sendCallback()` stores the last `esp_now_send_status_t` in `lastStatus_`.
- `getLastStatus()` returns that stored value.
- A successful `send()` call returning `0` means the packet was submitted to ESP-NOW, not necessarily that the remote device processed it.

### Direct receive registration

`Wireless` provides extra helpers for applications that want to work at the transport layer rather than through `SyncManager`:

- `setOnReceiveOther()`
  - fallback callback for packet types without a registered specific handler
- `addOnReceiveFor(type, cb)`
  - register a callback for a specific `TransportPacket.type`
- `removeOnReceiveFor(type)`
  - remove a type-specific callback

These are transport-level conveniences and are separate from mesh-level property/event registration.

## 7) Typed convenience helpers

`Wireless` also exposes transport-level typed helpers:

- `sendProperty()`
- `sendEvent()`
- `decodeProperty()`
- `decodeEvent()`
- `addOnReceiveProperty()`
- `addOnReceiveEvent()`

These helpers:

- require trivially copyable payload types
- are limited to the 250-byte `TransportPacket::data` capacity
- operate on raw transport packet types and channels
- can be useful for direct `Wireless` applications outside the mesh protocol

They are not the same as mesh properties and mesh events implemented by `SyncManager`.

## 8) Build-time knobs

The transport implementation currently uses these compile-time knobs in `src/Wireless.cpp`:

| Macro | Default | Meaning |
|---|---:|---|
| `DEBUG_ESP_NOW` | `0` | Enables debug logging when set to `1` |
| `ESP_NOW_CHANNEL` | `1` | Wi-Fi channel used for ESP-NOW |
| `ESPNOW_NO_DISABLE_WIFI` | not defined | When undefined, `begin()` reconfigures Wi-Fi into STA mode and sets the channel |

## 9) Platform expectations

The included `Wireless` transport is ESP32-oriented:

- includes `esp_now.h`, `esp_wifi.h`, `WiFi.h`, and FreeRTOS queue headers
- uses `WIFI_IF_STA`
- depends on ESP-NOW being available in the target environment

That makes `Wireless` a good default transport for ESP32 Arduino targets. The mesh layer itself is written against `ITransport`, so a different transport could be added later without changing the mesh protocol design.

## 10) Typical usage with mesh

Typical integration with the mesh layer looks like:

```cpp
SyncManager *sync = SyncManager::getInstance();
sync->setTransport(&Wireless::instance());
sync->begin();

void loop() {
  sync->loop();
}
```

In that setup:

- `Wireless` handles radio setup, packet send/receive, ESP-NOW address translation, and singleton callback dispatch
- `SyncManager` handles mesh protocol decoding, discovery, groups, properties, events, time sync, and relay

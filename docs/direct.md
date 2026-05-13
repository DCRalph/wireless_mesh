# Direct ESP-to-ESP Comms (`src/Direct.h`, `src/Direct.cpp`)

`DirectComms` is a typed point-to-point messaging layer built on top of `ITransport`. It depends only on the abstract transport interface — no `Wireless`-specific includes, no `WirelessFrame`, no MAC-only assumptions in its public API. Any transport that implements `ITransport` will work.

For the mesh protocol see [mesh.md](mesh.md). For the transport see [wireless.md](wireless.md). For the public API reference see [api-spec.md](api-spec.md).

## 1) When to use it

Use `DirectComms` when you have two (or more) ESPs that just need to send each other typed payloads:

- you know how to address the peer (e.g. its 6-byte MAC for the included `Wireless` transport, or whatever address form the transport defines)
- you do not want the overhead of mesh groups, masters, discovery, or time sync just to send a struct
- you want a request/response RPC pattern with timeout, in addition to fire-and-forget sends

Use `SyncManager` when you want replicated properties, broadcast events, group state, time sync, or multi-hop relay.

The two layers can coexist on the same physical transport — `DirectComms` registers a separate transport packet type (`0xD17C`) that doesn't collide with the mesh's `0xBEEF`. See §6 below for the wiring.

## 2) Layout and limits

Direct frames are carried inside a regular `TransportPacket` with `type = DirectFraming::kDirectPacketType` and the following framing inside `data`:

| Offset | Field | Size | Notes |
|---:|---|---:|---|
| 0 | `kind` | 1 | `1=OneWay`, `2=Request`, `3=Response` |
| 1 | `channelId` | 2 | user-chosen channel |
| 3 | `requestId` | 2 | `0` for `OneWay`; nonzero, monotonically allocated for `Request`/`Response` |
| 5 | `payloadLen` | 1 | bytes of user payload that follow |
| 6 | `payload` | up to 244 | trivially-copyable struct bytes |

`DirectFraming::kMaxPayloadSize` (244) is enforced at compile time on the typed handles via `static_assert`.

## 3) Lifecycle

```cpp
#include "Direct.h"
#include "Wireless.h"   // any ITransport implementation works; this is the included one

void setup() {
  ITransport *transport = Wireless::getInstance();
  transport->begin();

  DirectComms::getInstance()->begin(transport);   // standalone wiring
}

void loop() {
  Wireless::getInstance()->loop();         // drains the RX queue
  DirectComms::getInstance()->loop();      // expires pending RPC timeouts
}
```

- `begin(transport)` is the convenience setup: it calls `setTransport(transport)` and `transport->setReceiveCallback(receiveCallback())`.
- Use `setTransport(transport)` alone when something else (e.g. `SyncManager`) already owns the transport's single `setReceiveCallback`. See §6.
- `end()` clears any pending RPC entries. The transport's receive callback is owned by the caller and is not touched.
- `DirectComms::loop()` is the only periodic work needed. It walks the pending-request table once per call and fires `onResult(false, ...)` for any expired entry.

## 4) `DirectChannel<T>` — fire-and-forget

```cpp
struct Reading { uint32_t seq; float tempC; };
auto reading = direct->channel<Reading>(0x4001);

reading.onReceive([](const TransportAddress &peer, const Reading &r) {
  Serial.printf("seq=%lu temp=%.2f\n",
                static_cast<unsigned long>(r.seq), r.tempC);
});

uint8_t peerMac[6] = { 0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC };
reading.sendTo(peerMac, Reading{1, 22.5f});                  // raw-MAC convenience
reading.sendTo(TransportAddress::fromMac(peerMac), Reading{}); // explicit form
```

- `T` must be trivially copyable and at most 244 bytes; both checks are compile-time `static_assert`s.
- `sendTo()` returns `false` only if the transport refused the submit (returns nonzero from `sendPacket`). For ESP-NOW that means "submitted to the radio", not "the peer received it" — use `DirectRpc` for round-trip confirmation.
- `onReceive()` runs on whatever thread your transport drains receive frames on (for the included `Wireless` transport, that's the Arduino loop thread inside `Wireless::loop()`).
- Re-registering on the same channel replaces the existing handler. Passing an empty `std::function` clears it.

## 5) `DirectRpc<Req, Resp>` — request / response with timeout

```cpp
struct PingReq  { uint32_t seq; };
struct PingResp { uint32_t seq; uint32_t respMs; };

auto ping = direct->rpc<PingReq, PingResp>(0x5001);

// Responder side
ping.onRequest([](const TransportAddress &peer, const PingReq &req, PingResp &resp) {
  resp.seq = req.seq;
  resp.respMs = millis();
  return true;        // false to drop the request silently
});

// Caller side
ping.request(peerMac, PingReq{42}, /*timeoutMs=*/200,
             [](bool ok, const PingResp &resp) {
               if (ok) {
                 Serial.printf("seq=%lu ok respMs=%lu\n",
                               static_cast<unsigned long>(resp.seq),
                               static_cast<unsigned long>(resp.respMs));
               } else {
                 Serial.println("timeout");
               }
             });
```

Behavior:

- `request()` allocates a 16-bit `requestId` (skips zero), sends a `Request` frame, and stores the callback with a deadline of `millis() + timeoutMs`.
- An incoming `Response` frame matching the `requestId` and `channelId` fires the callback with `(true, decodedResp)` and removes the entry.
- `DirectComms::loop()` removes any expired entry and fires `(false, Resp{})`.
- If the radio refuses the initial submit, `request()` fires `(false, Resp{})` immediately and returns `false`. No pending entry is kept.
- Late responses (arriving after the timeout fired) are silently dropped because the entry has already been removed.
- The callback fires exactly once per `request()` call.

Responder semantics:

- `onRequest()` decodes `Req`, allocates `Resp{}`, calls the user handler. If the handler returns `true`, `Resp` is serialized and sent back to the source address with the same `requestId`.
- A handler that returns `false` produces no wire response — the caller will see a timeout.

## 6) Coexistence with `SyncManager`

`ITransport` exposes a single `setReceiveCallback` slot. `SyncManager::begin()` claims it for the mesh's `0xBEEF` frames. To run `DirectComms` on the same transport without disturbing the mesh callback, you wire the demuxing yourself.

The included `Wireless` transport offers a per-packet-type dispatch hook (`addOnReceiveFor`) for exactly this purpose:

```cpp
SyncManager *mesh = SyncManager::getInstance();
mesh->setTransport(Wireless::getInstance());
mesh->begin();   // owns transport->setReceiveCallback() for 0xBEEF frames

DirectComms *direct = DirectComms::getInstance();
direct->setTransport(Wireless::getInstance());   // for sending

auto directRx = direct->receiveCallback();
Wireless::getInstance()->addOnReceiveFor(
    DirectFraming::kDirectPacketType,
    [directRx](WirelessFrame *frame) {
      directRx(TransportAddress::fromMac(frame->mac), frame->packet);
    });
```

The four lines of wiring stay in user code; `DirectComms` itself never includes `Wireless.h`. If you write a different `ITransport` implementation, expose a similar packet-type demux hook on it and the same pattern applies.

For a non-Wireless transport that has no packet-type demux, you can chain inline:

```cpp
// Pseudocode for a transport with no addOnReceiveFor equivalent
auto otherCb = mesh->getReceiveCallback();   // requires SyncManager to expose this
auto directCb = direct->receiveCallback();
transport->setReceiveCallback(
    [otherCb, directCb](const TransportAddress &src, const TransportPacket &pkt) {
      if (pkt.type == DirectFraming::kDirectPacketType) directCb(src, pkt);
      else                                              otherCb(src, pkt);
    });
```

(`SyncManager` does not currently expose `getReceiveCallback()`; the `Wireless`-specific demux pattern above is the supported coexistence path today.)

## 7) Build requirements

The library requires C++17 (mesh and direct headers use `std::enable_if_t`, `std::is_void_v`, and lambda init-captures). On PlatformIO ESP32 add to `platformio.ini`:

```ini
build_unflags = -std=gnu++11
build_flags   = -std=gnu++17
```

Recent Arduino-ESP32 cores (3.x) already default to `gnu++17`.

## 8) Limits and caveats

- **Encryption / authentication**: none. Same trust model as the rest of the library — see the security note in `api-spec.md`.
- **No retransmits**: ESP-NOW reports send status but does not auto-retry the application payload. If you need delivery guarantees, use `DirectRpc` (the timeout becomes your retry trigger) or build retries on top of `DirectChannel`.
- **Channel namespace**: 16-bit `channelId` is local to your application. Pick distinct values for `DirectChannel<T>` and `DirectRpc<Req,Resp>` so handlers don't shadow each other.
- **Pending-request table**: lives in a `std::vector<PendingRequest>`. Keep your `loop()` interval below `timeoutMs` to keep the table from growing unboundedly under heavy concurrent traffic.
- **Wi-Fi channel** (Wireless transport): ESP-NOW peers must be on the same channel. `Wireless::begin()` sets the channel via `ESP_NOW_CHANNEL` (default `1`). Both boards must agree.

# Wireless Mesh

`Wireless Mesh` is an Arduino-oriented mesh synchronization library built around two layers:

- a transport layer, with an included ESP-NOW `Wireless` implementation
- a mesh layer, centered on `SyncManager`, that handles discovery, groups, time sync, replicated properties, events, and optional relay

## What It Provides

At the mesh layer, the library provides:

- device discovery
- group creation and joining
- sync modes: `SOLO`, `JOIN`, `HOST`, `AUTO`
- master-based time synchronization
- replicated owner-partitioned properties
- transient mesh events
- optional multi-hop relay with deduplication

The mesh protocol itself is transport-agnostic through `ITransport`. The transport interface now owns both the local-address and broadcast-address concepts, so mesh code no longer assumes MAC-style broadcast semantics. This repository currently includes one concrete transport: `Wireless`, which uses ESP-NOW.

## Platform Expectations

The included transport implementation is ESP32-oriented:

- `library.properties` declares `architectures=esp32`
- `Wireless` depends on ESP-NOW, `WiFi.h`, `esp_wifi.h`, and FreeRTOS queue support
- the mesh layer is written against `ITransport`, so other transports could be added later, but they are not included here

`library.json` currently lists broader platforms than the code realistically supports. For this repository as it exists today, the safe expectation is ESP32 with Arduino.

## Architecture

- `src/ITransport.h`
  - shared transport primitives and the `ITransport` interface
- `src/Wireless.h` / `src/Wireless.cpp`
  - ESP-NOW transport implementation
- `src/Mesh.h` / `src/Mesh.cpp`
  - mesh protocol, `SyncManager`, property/event handling, group state, relay, and time sync

Typical usage is:

```cpp
SyncManager *sync = SyncManager::getInstance();
sync->setTransport(Wireless::getInstance());
sync->begin();

void loop() {
  sync->loop();
}
```

`Wireless::getInstance()` returns a non-owning singleton transport pointer. Tear it down with `end()` or `unSetup()`, not `delete`.

## Documentation

- [`docs/mesh.md`](docs/mesh.md)
  - mesh protocol, runtime behavior, sync modes, relay, constants, properties, and events
- [`docs/wireless.md`](docs/wireless.md)
  - transport primitives, transport-defined addressing, ESP-NOW setup, callback flow, peer handling, and singleton usage
- [`docs/api-spec.md`](docs/api-spec.md)
  - public mesh API reference for `SyncManager`, `PropertyHandle`, `RawPropertyHandle`, and `EventHandle`

## Installation Notes

- Arduino library metadata is included via `library.properties`.
- PlatformIO metadata is included via `library.json`.
- `library.json` references `examples/*/*.ino`, but this repository currently does not include an `examples/` directory.

## License

This project is licensed under the GNU GPL v3. See [`LICENSE`](LICENSE).

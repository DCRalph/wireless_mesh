// MeshSlave - explicit slave / joiner node.
//
// Pairs with the MeshMaster sketch. Flash this onto one or more boards.
// SyncMode::JOIN tells SyncManager to discover and join the first group
// it sees - so when a MeshMaster is in range it will be picked up
// automatically.
//
// What it demonstrates:
//   - JOIN mode (slave waits for a group and joins it)
//   - Reading a master-owned property (kCommandSeqKey)
//   - Owning and publishing a property (kSlaveReadingKey) for the master
//   - Listening for a void event (kPingChannel) broadcast by the master
//   - Time sync via SyncManager::syncMillis()

#include <Arduino.h>
#include "Mesh.h"
#include "Wireless.h"

constexpr uint16_t kCommandSeqKey   = 0x7001; // master -> us
constexpr uint16_t kSlaveReadingKey = 0x7002; // us -> master
constexpr uint16_t kPingChannel     = 0x7101;

struct SlaveReading
{
  uint32_t seq;
  float    tempC;
};

SyncManager *mesh = nullptr;
PropertyHandle<uint32_t>     commandSeq;
PropertyHandle<SlaveReading> myReading;
EventHandle<void>            pingEvent;

uint32_t txSeq      = 0;
uint32_t lastTxMs   = 0;
uint32_t lastDumpMs = 0;

void setup()
{
  Serial.begin(115200);
  delay(200);
  Serial.println("== MeshSlave ==");

  mesh = SyncManager::getInstance();
  mesh->setTransport(Wireless::getInstance());
  mesh->setSyncMode(SyncMode::JOIN); // wait for a group, then join it
  mesh->begin();

  Serial.printf("Local device ID: 0x%08lx\n",
                static_cast<unsigned long>(mesh->getDeviceId()));

  commandSeq = mesh->property<uint32_t>(kCommandSeqKey);
  myReading  = mesh->property<SlaveReading>(kSlaveReadingKey);
  pingEvent  = mesh->event<void>(kPingChannel);

  // The master owns kCommandSeqKey - onChange fires when the master updates it.
  commandSeq.onChange([](uint32_t fromDevice, uint32_t value) {
    Serial.printf("[cmd] from master 0x%08lx seq=%lu\n",
                  static_cast<unsigned long>(fromDevice),
                  static_cast<unsigned long>(value));
  });

  pingEvent.onEvent([](uint32_t fromDevice) {
    Serial.printf("[ping] from 0x%08lx at sync=%lums\n",
                  static_cast<unsigned long>(fromDevice),
                  static_cast<unsigned long>(SyncManager::syncMillis()));
  });

  mesh->setGroupJoinedCallback([](const GroupInfo &g) {
    Serial.printf("[group] joined 0x%08lx (master=0x%08lx)\n",
                  static_cast<unsigned long>(g.groupId),
                  static_cast<unsigned long>(g.masterDeviceId));
  });

  mesh->setGroupLeftCallback([]() {
    Serial.println("[group] left");
  });

  mesh->setTimeSyncCallback([](uint32_t syncedTime) {
    Serial.printf("[time] synced -> %lums\n",
                  static_cast<unsigned long>(syncedTime));
  });
}

void loop()
{
  mesh->loop();

  const uint32_t now = millis();

  if (mesh->isInGroup() && now - lastTxMs >= 1500)
  {
    lastTxMs = now;
    SlaveReading r{++txSeq, 20.0f + (txSeq % 10) * 0.5f};
    myReading.set(r); // master will see this via onChange
  }

  if (now - lastDumpMs >= 5000)
  {
    lastDumpMs = now;
    Serial.printf("[status] inGroup=%d timeSynced=%d offset=%ldms\n",
                  static_cast<int>(mesh->isInGroup()),
                  static_cast<int>(mesh->isTimeSynced()),
                  static_cast<long>(mesh->getTimeOffset()));
  }
}

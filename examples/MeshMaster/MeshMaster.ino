// MeshMaster - explicit master / host node.
//
// Flash this onto ONE board, and flash MeshSlave onto one or more others.
// This board uses SyncMode::HOST: SyncManager creates a group during
// begin() and becomes its master. Slaves running MeshSlave (SyncMode::JOIN)
// will discover and join this group automatically.
//
// What it demonstrates:
//   - HOST mode (master creates the group)
//   - Owning and publishing a property (kCommandSeqKey) the slaves read
//   - Receiving slave-owned properties (kSlaveReadingKey) per device
//   - Emitting a void event (kPingChannel) to all members at once
//   - Group lifecycle callbacks

#include <Arduino.h>
#include "Mesh.h"
#include "Wireless.h"

constexpr uint16_t kCommandSeqKey   = 0x7001; // master -> slaves
constexpr uint16_t kSlaveReadingKey = 0x7002; // slaves -> master (owner per slave)
constexpr uint16_t kPingChannel     = 0x7101;

struct SlaveReading
{
  uint32_t seq;
  float    tempC;
};

SyncManager *mesh = nullptr;
PropertyHandle<uint32_t>     commandSeq;
PropertyHandle<SlaveReading> slaveReading;
EventHandle<void>            pingEvent;

uint32_t cmdCounter = 0;
uint32_t lastCmdMs  = 0;
uint32_t lastPingMs = 0;
uint32_t lastDumpMs = 0;

void setup()
{
  Serial.begin(115200);
  delay(200);
  Serial.println("== MeshMaster ==");

  mesh = SyncManager::getInstance();
  mesh->setTransport(Wireless::getInstance());
  mesh->setSyncMode(SyncMode::HOST); // creates the group on begin()
  mesh->begin();

  Serial.printf("Local device ID: 0x%08lx\n",
                static_cast<unsigned long>(mesh->getDeviceId()));

  // --- Properties ---
  commandSeq   = mesh->property<uint32_t>(kCommandSeqKey);
  slaveReading = mesh->property<SlaveReading>(kSlaveReadingKey);

  // Slaves own kSlaveReadingKey, so onChange fires once per slave that updates.
  slaveReading.onChange([](uint32_t fromDevice, SlaveReading r) {
    Serial.printf("slave 0x%08lx -> seq=%lu temp=%.2fC\n",
                  static_cast<unsigned long>(fromDevice),
                  static_cast<unsigned long>(r.seq), r.tempC);
  });

  // --- Events ---
  pingEvent = mesh->event<void>(kPingChannel);

  // --- Group lifecycle ---
  mesh->setGroupCreatedCallback([](const GroupInfo &g) {
    Serial.printf("[group] created 0x%08lx (I am master)\n",
                  static_cast<unsigned long>(g.groupId));
  });
}

void loop()
{
  mesh->loop();

  const uint32_t now = millis();

  if (mesh->isInGroup() && now - lastCmdMs >= 2000)
  {
    lastCmdMs = now;
    commandSeq.set(++cmdCounter);
  }

  if (mesh->isInGroup() && now - lastPingMs >= 5000)
  {
    lastPingMs = now;
    pingEvent.emit(); // broadcast to all members
  }

  if (now - lastDumpMs >= 5000)
  {
    lastDumpMs = now;
    const GroupInfo &g = mesh->getGroupInfo();
    Serial.printf("[status] members=%u cmd=%lu\n",
                  static_cast<unsigned>(g.members.size()),
                  static_cast<unsigned long>(cmdCounter));
  }
}

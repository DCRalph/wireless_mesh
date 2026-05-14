// MeshAuto - self-organising mesh group (no fixed master).
//
// Flash this onto two or more boards. SyncMode::AUTO makes every node:
//   - wait until it sees at least one other AUTO peer
//   - elect a creator (smallest deviceId) which hosts a group
//   - everyone else joins that group automatically
//
// Each node publishes its own NodeStatus property; every node sees every
// other node's status via onChange. Use mesh->isGroupMaster() at runtime
// if a single node needs to do extra work for the group.
//
// No MAC pinning, no #defines per board - identical firmware everywhere.

#include <Arduino.h>
#include "Mesh.h"
#include "Wireless.h"

constexpr uint16_t kStatusKey = 0x7201;

struct NodeStatus
{
  uint32_t uptimeSeconds;
  uint16_t freeMemKb;
  uint8_t  battery;
};

SyncManager *mesh = nullptr;
PropertyHandle<NodeStatus> status;

uint32_t lastTickMs = 0;
uint32_t lastDumpMs = 0;

void setup()
{
  Serial.begin(115200);
  delay(200);
  Serial.println("== MeshAuto ==");

  mesh = SyncManager::getInstance();
  mesh->setTransport(Wireless::getInstance());
  mesh->setSyncMode(SyncMode::AUTO);
  mesh->begin();

  Serial.printf("Local device ID: 0x%08lx\n",
                static_cast<unsigned long>(mesh->getDeviceId()));

  status = mesh->property<NodeStatus>(kStatusKey);

  status.onChange([](uint32_t fromDevice, NodeStatus s) {
    Serial.printf("status from 0x%08lx: up=%lus mem=%uKb batt=%u%%\n",
                  static_cast<unsigned long>(fromDevice),
                  static_cast<unsigned long>(s.uptimeSeconds),
                  static_cast<unsigned>(s.freeMemKb),
                  static_cast<unsigned>(s.battery));
  });

  mesh->setGroupCreatedCallback([](const GroupInfo &g) {
    Serial.printf("[auto] created group 0x%08lx - I am master\n",
                  static_cast<unsigned long>(g.groupId));
  });
  mesh->setGroupJoinedCallback([](const GroupInfo &g) {
    Serial.printf("[auto] joined group 0x%08lx - master is 0x%08lx\n",
                  static_cast<unsigned long>(g.groupId),
                  static_cast<unsigned long>(g.masterDeviceId));
  });
}

void loop()
{
  mesh->loop();

  const uint32_t now = millis();

  if (mesh->isInGroup() && now - lastTickMs >= 2000)
  {
    lastTickMs = now;
    NodeStatus s{
        now / 1000,
        static_cast<uint16_t>(ESP.getFreeHeap() / 1024),
        static_cast<uint8_t>(80 + (now / 1000) % 20)};
    status.set(s);
  }

  if (now - lastDumpMs >= 6000)
  {
    lastDumpMs = now;
    const GroupInfo &g = mesh->getGroupInfo();
    Serial.printf("[summary] inGroup=%d isMaster=%d members=%u\n",
                  static_cast<int>(mesh->isInGroup()),
                  static_cast<int>(mesh->isGroupMaster()),
                  static_cast<unsigned>(g.members.size()));
  }
}

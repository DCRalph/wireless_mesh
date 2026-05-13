// DirectAndMesh — proves DirectComms and SyncManager coexist on one
// device on the same Wireless transport.
//
// SyncManager owns the transport's single setReceiveCallback (for the
// mesh's 0xBEEF frames). DirectComms is wired up for SENDING via
// setTransport(), and its receive callback is plumbed through the
// Wireless-specific addOnReceiveFor() so direct (0xD17C) frames reach
// it without disturbing the mesh's callback.
//
// DirectComms itself never touches Wireless — the user does the wiring
// here, in 4 lines, so DirectComms stays transport-agnostic.

#include <Arduino.h>
#include <WiFi.h>

#include "Direct.h"
#include "Mesh.h"
#include "Wireless.h"

static const uint8_t kPeerMac[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

constexpr uint16_t kCounterPropertyKey = 0x6001;
constexpr uint16_t kDirectMsgChannel = 0x6002;

struct DirectMsg
{
  uint32_t seq;
  uint8_t bytes[8];
};

SyncManager *mesh = nullptr;
DirectComms *direct = nullptr;
PropertyHandle<uint32_t> counterProp;
DirectChannel<DirectMsg> directMsg;

uint32_t counter = 0;
uint32_t lastTickMs = 0;

void setup()
{
  Serial.begin(115200);
  delay(200);

  // ---- Mesh side: owns the transport's setReceiveCallback ----
  mesh = SyncManager::getInstance();
  mesh->setTransport(Wireless::getInstance());
  mesh->setSyncMode(SyncMode::AUTO);
  mesh->begin();

  counterProp = mesh->property<uint32_t>(kCounterPropertyKey);
  counterProp.onChange([](uint32_t fromDevice, uint32_t value)
                       {
    Serial.printf("[mesh] counter from %lu = %lu\n",
                  static_cast<unsigned long>(fromDevice),
                  static_cast<unsigned long>(value)); });

  // ---- Direct side: setTransport for sending; route receives via
  //      Wireless::addOnReceiveFor so we don't clobber mesh's callback.
  direct = DirectComms::getInstance();
  direct->setTransport(Wireless::getInstance());

  auto directRx = direct->receiveCallback();
  Wireless::getInstance()->addOnReceiveFor(
      DirectFraming::kDirectPacketType,
      [directRx](WirelessFrame *frame)
      {
        directRx(TransportAddress::fromMac(frame->mac), frame->packet);
      });

  directMsg = direct->channel<DirectMsg>(kDirectMsgChannel);
  directMsg.onReceive([](const TransportAddress &peer, const DirectMsg &m)
                      {
    const uint8_t *p = peer.data();
    Serial.printf("[direct] msg seq=%lu from %02X:%02X:%02X:%02X:%02X:%02X\n",
                  static_cast<unsigned long>(m.seq),
                  p[0], p[1], p[2], p[3], p[4], p[5]); });
}

void loop()
{
  mesh->loop();      // pumps Wireless::loop() internally
  direct->loop();    // pumps RPC timeouts (none here, but safe to call)

  const uint32_t now = millis();
  if (now - lastTickMs >= 1000)
  {
    lastTickMs = now;
    ++counter;

    if (mesh->isInGroup())
    {
      counterProp.set(counter);
    }

    DirectMsg m{counter, {1, 2, 3, 4, 5, 6, 7, 8}};
    directMsg.sendTo(kPeerMac, m);
  }
}

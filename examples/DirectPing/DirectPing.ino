// DirectPing — minimal two-board demo of DirectComms one-way struct send.
//
// Flash both boards. Each prints its own MAC at boot; copy each board's
// MAC into kPeerMac on the other board, then re-flash. Both boards send
// a Reading struct to the peer once a second and print whatever they
// receive.
//
// DirectComms is wired to the transport via the standalone begin() form,
// which uses ITransport::setReceiveCallback. No mesh group is needed.

#include <Arduino.h>
#include <WiFi.h>

#include "Direct.h"
#include "Wireless.h"

// >>> Set this to the OTHER board's MAC address. <<<
static const uint8_t kPeerMac[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

constexpr uint16_t kReadingChannel = 0x4001;

struct Reading
{
  uint32_t seq;
  float tempC;
  float humidity;
};

DirectComms *direct = nullptr;
DirectChannel<Reading> reading;
uint32_t lastSendMs = 0;
uint32_t txSeq = 0;

static void printMac(const uint8_t *mac)
{
  for (int i = 0; i < 6; ++i)
  {
    if (i)
    {
      Serial.print(':');
    }
    if (mac[i] < 0x10)
    {
      Serial.print('0');
    }
    Serial.print(mac[i], HEX);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(200);

  ITransport *transport = Wireless::getInstance();
  transport->begin();

  uint8_t myMac[6] = {0};
  WiFi.macAddress(myMac);
  Serial.print("My MAC: ");
  printMac(myMac);
  Serial.println();

  direct = DirectComms::getInstance();
  direct->begin(transport);

  reading = direct->channel<Reading>(kReadingChannel);
  reading.onReceive([](const TransportAddress &peer, const Reading &r)
                    {
    Serial.print("RX from ");
    printMac(peer.data());
    Serial.printf("  seq=%lu temp=%.2f hum=%.2f\n",
                  static_cast<unsigned long>(r.seq), r.tempC, r.humidity); });
}

void loop()
{
  Wireless::getInstance()->loop();
  direct->loop();

  const uint32_t now = millis();
  if (now - lastSendMs >= 1000)
  {
    lastSendMs = now;
    Reading r{++txSeq, 22.5f + (txSeq % 5) * 0.1f, 48.0f};
    if (!reading.sendTo(kPeerMac, r))
    {
      Serial.println("send failed");
    }
  }
}

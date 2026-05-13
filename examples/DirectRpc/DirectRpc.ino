// DirectRpc — request/response with timeout via DirectComms.
//
// Both boards both call/respond. Each one sends a PingReq once a second
// to the other and logs either round-trip time or "TIMEOUT". Each one
// also serves PingReq messages by filling out a PingResp.
//
// Set kPeerMac on each board to the OTHER board's MAC address.

#include <Arduino.h>
#include <WiFi.h>

#include "Direct.h"
#include "Wireless.h"

static const uint8_t kPeerMac[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

constexpr uint16_t kPingChannel = 0x5001;
constexpr uint32_t kRequestTimeoutMs = 200;

struct PingReq
{
  uint32_t seq;
  uint32_t senderMillis;
};

struct PingResp
{
  uint32_t seq;
  uint32_t senderMillis;
  uint32_t responderMillis;
};

DirectComms *direct = nullptr;
DirectRpc<PingReq, PingResp> ping;
uint32_t lastSendMs = 0;
uint32_t txSeq = 0;

void setup()
{
  Serial.begin(115200);
  delay(200);

  ITransport *transport = Wireless::getInstance();
  transport->begin();

  uint8_t myMac[6] = {0};
  WiFi.macAddress(myMac);
  Serial.printf("My MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);

  direct = DirectComms::getInstance();
  direct->begin(transport);

  ping = direct->rpc<PingReq, PingResp>(kPingChannel);

  ping.onRequest([](const TransportAddress &peer, const PingReq &req, PingResp &resp)
                 {
    resp.seq = req.seq;
    resp.senderMillis = req.senderMillis;
    resp.responderMillis = millis();
    return true; });
}

void loop()
{
  Wireless::getInstance()->loop();
  direct->loop();

  const uint32_t now = millis();
  if (now - lastSendMs >= 1000)
  {
    lastSendMs = now;
    PingReq req{++txSeq, now};
    const uint32_t startMs = now;
    const uint32_t seq = txSeq;
    ping.request(kPeerMac, req, kRequestTimeoutMs,
                 [seq, startMs](bool ok, const PingResp &resp)
                 {
                   if (ok)
                   {
                     const uint32_t rtt = millis() - startMs;
                     Serial.printf("seq=%lu OK rtt=%lu ms responderMs=%lu\n",
                                   static_cast<unsigned long>(seq),
                                   static_cast<unsigned long>(rtt),
                                   static_cast<unsigned long>(resp.responderMillis));
                   }
                   else
                   {
                     Serial.printf("seq=%lu TIMEOUT\n",
                                   static_cast<unsigned long>(seq));
                   }
                 });
  }
}

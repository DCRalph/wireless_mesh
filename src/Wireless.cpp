#include "Wireless.h"
#include <cstring>
#include <utility>

#ifndef DEBUG_ESP_NOW
#define DEBUG_ESP_NOW 0
#endif

#ifndef ESP_NOW_CHANNEL
#define ESP_NOW_CHANNEL 1
#endif

namespace
{
  constexpr uint8_t kBroadcastMac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
} // namespace

Wireless *Wireless::getInstance()
{
  static Wireless inst;
  return &inst;
}

Wireless::Wireless()
{
}

void Wireless::setup()
{
  begin();
}

void Wireless::unSetup()
{
  end();
}

void Wireless::loop()
{
  if (!setupDone.load(std::memory_order_acquire) || incomingQueue_ == nullptr)
  {
    return;
  }

  WirelessFrame frame;
  while (xQueueReceive(incomingQueue_, &frame, 0) == pdTRUE)
  {
    if (receiveCb)
    {
      receiveCb(TransportAddress::fromMac(frame.mac), frame.packet);
    }

    auto it = onReceiveForCallbacks.find(frame.packet.type);
    if (it != onReceiveForCallbacks.end())
    {
      // Copy out for re-entrancy: the handler may add/remove map entries.
      auto cb = it->second;
      cb(&frame);
    }
    else if (onReceiveOtherCb)
    {
      onReceiveOtherCb(&frame);
    }
  }
}

bool Wireless::isSetupDone() const
{
  return setupDone.load(std::memory_order_acquire);
}

void Wireless::sendCallback(const uint8_t *mac_addr,
                            esp_now_send_status_t status)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr),
           "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2],
           mac_addr[3], mac_addr[4], mac_addr[5]);

#if DEBUG_ESP_NOW == 1
  Serial.println("########### Sent Packet ###########");
  Serial.printf("Sent to: %s\n", macStr);
  Serial.printf("Send Status: %s\n",
                status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  Serial.println("###################################");
#endif

  lastStatus_.store(status, std::memory_order_relaxed);
}

void Wireless::recvCallback(const uint8_t *mac_addr, const uint8_t *data,
                            uint16_t len)
{
  constexpr uint16_t kHeaderLen = sizeof(TransportPacket::type) + sizeof(TransportPacket::len);
  if (incomingQueue_ == nullptr || data == nullptr || len < kHeaderLen)
  {
    return;
  }

  WirelessFrame frame;
  // TransportPacket is packed: type + len occupy the first kHeaderLen bytes.
  memcpy(&frame.packet, data, kHeaderLen);

  if (frame.packet.len > sizeof(frame.packet.data) ||
      static_cast<uint16_t>(kHeaderLen + frame.packet.len) > len)
  {
    return;
  }

  memcpy(frame.mac, mac_addr, ESP_NOW_ETH_ALEN);
  frame.direction = PacketDirection::RECV;
  if (frame.packet.len > 0)
  {
    memcpy(frame.packet.data, data + kHeaderLen, frame.packet.len);
  }

  if (xQueueSendToBack(incomingQueue_, &frame, 0) != pdTRUE)
  {
    droppedRxFrames_.fetch_add(1, std::memory_order_relaxed);
  }
}

void Wireless::setOnReceiveOther(std::function<void(WirelessFrame *frame)> cb)
{
  onReceiveOtherCb = cb;
}

void Wireless::addOnReceiveFor(uint16_t type,
                               std::function<void(WirelessFrame *frame)> cb)
{
  onReceiveForCallbacks[type] = cb;
}

void Wireless::removeOnReceiveFor(uint16_t type)
{
  onReceiveForCallbacks.erase(type);
}

int Wireless::send(const TransportPacket *p, const uint8_t *peer_addr)
{
  const uint16_t payloadLen = p->len > sizeof(p->data) ? sizeof(p->data) : p->len;
  const uint16_t frameLen = static_cast<uint16_t>(sizeof(p->type) + sizeof(p->len) + payloadLen);
  return send(reinterpret_cast<const uint8_t *>(p), frameLen, peer_addr);
}

int Wireless::send(const TransportPacket *p, const TransportAddress &peer)
{
  return send(reinterpret_cast<const uint8_t *>(p),
              static_cast<uint16_t>(sizeof(p->type) + sizeof(p->len) +
                                    (p->len > sizeof(p->data) ? sizeof(p->data) : p->len)),
              peer);
}

int Wireless::send(const uint8_t *data, uint16_t len, const uint8_t *peer_addr)
{
  if (!setupDone.load(std::memory_order_acquire))
  {
    Serial.println("ESP-NOW not initialized");
    return -1;
  }

  if (!ensurePeerRegistered(peer_addr))
  {
    return -1;
  }

  esp_err_t err = esp_now_send(peer_addr, data, len);
  if (err != ESP_OK)
  {
    Serial.printf("esp_now_send failed: %d\n", err);
    return -1;
  }
  return 0;
}

bool Wireless::ensurePeerRegistered(const uint8_t *peer_addr)
{
  if (peer_addr == nullptr)
  {
    return false;
  }
  if (esp_now_is_peer_exist(peer_addr))
  {
    return true;
  }

  esp_now_peer_info_t peerInfo{};
  memcpy(peerInfo.peer_addr, peer_addr, ESP_NOW_ETH_ALEN);
  peerInfo.channel = ESP_NOW_CHANNEL;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  esp_err_t err = esp_now_add_peer(&peerInfo);
  if (err == ESP_OK || err == ESP_ERR_ESPNOW_EXIST)
  {
    return true;
  }
  Serial.printf("esp_now_add_peer failed: %d\n", err);
  return false;
}

int Wireless::send(const uint8_t *data, uint16_t len, const TransportAddress &peer)
{
  if (peer.length != kAddressLength)
  {
    Serial.println("ESP-NOW peer address length mismatch");
    return -1;
  }
  return send(data, len, peer.data());
}

int Wireless::send(WirelessFrame *frame)
{
  if (frame->direction == PacketDirection::SEND)
  {
    return send(&frame->packet, frame->mac);
  }
  else
  {
    Serial.println("Cannot send a receive packet");
    return -1;
  }
}

bool Wireless::begin()
{
  if (setupDone.load(std::memory_order_acquire))
  {
    return true;
  }

#ifndef ESPNOW_NO_DISABLE_WIFI
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESP_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
#endif

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return false;
  }

  if (incomingQueue_ == nullptr)
  {
    incomingQueue_ = xQueueCreate(kRxQueueDepth, sizeof(WirelessFrame));
    if (incomingQueue_ == nullptr)
    {
      Serial.println("Failed to create ESP-NOW RX queue");
      esp_now_deinit();
      return false;
    }
  }
  droppedRxFrames_.store(0, std::memory_order_relaxed);

  // Note: keep global callback trampoline for ESPNOW C callbacks.
  esp_now_register_send_cb([](const uint8_t *mac_addr,
                              esp_now_send_status_t status)
                           { Wireless::getInstance()->sendCallback(mac_addr, status); });

  esp_now_register_recv_cb([](const uint8_t *mac_addr,
                              const uint8_t *data,
                              int len)
                           { Wireless::getInstance()->recvCallback(mac_addr, data, static_cast<uint16_t>(len)); });

  broadcastPeerConfigured_ = ensurePeerRegistered(broadcastAddress().data());

  setupDone.store(true, std::memory_order_release);
  return true;
}

void Wireless::end()
{
  if (setupDone.load(std::memory_order_acquire))
  {
    if (broadcastPeerConfigured_)
    {
      const TransportAddress broadcast = broadcastAddress();
      esp_now_del_peer(broadcast.data());
      broadcastPeerConfigured_ = false;
    }
    esp_now_unregister_recv_cb();
    esp_now_unregister_send_cb();
    esp_now_deinit();
    if (incomingQueue_ != nullptr)
    {
      vQueueDelete(incomingQueue_);
      incomingQueue_ = nullptr;
    }
    setupDone.store(false, std::memory_order_release);
  }
  receiveCb = ReceiveCallback{};
  onReceiveOtherCb = {};
  onReceiveForCallbacks.clear();
}

bool Wireless::isReady() const
{
  return setupDone.load(std::memory_order_acquire);
}

TransportAddress Wireless::localAddress() const
{
  uint8_t mac[kAddressLength]{};
  WiFi.macAddress(mac);
  return TransportAddress::fromBytes(mac, kAddressLength);
}

TransportAddress Wireless::broadcastAddress() const
{
  return TransportAddress::fromBytes(kBroadcastMac, kAddressLength);
}

esp_now_send_status_t Wireless::getLastStatus() const
{
  return lastStatus_.load(std::memory_order_relaxed);
}

int Wireless::sendPacket(const TransportPacket &packet, const TransportAddress &peer)
{
  return send(&packet, peer);
}

void Wireless::setReceiveCallback(ReceiveCallback cb)
{
  receiveCb = std::move(cb);
}
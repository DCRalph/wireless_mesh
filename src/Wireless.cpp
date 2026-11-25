#include "Wireless.h"
#include <cstring>
#include <esp_idf_version.h>
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

  static_assert(TransportPacket::kMaxFrame <= ESP_NOW_MAX_DATA_LEN,
                "TransportPacket must fit one ESP-NOW v1 frame");

  // ESP-IDF 5.0 passes receive metadata instead of the source MAC, and 5.5 passes
  // transmit metadata instead of the destination MAC.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
  {
    if (info != nullptr && info->src_addr != nullptr)
    {
      Wireless::getInstance()->recvCallback(info->src_addr, data, len);
    }
  }
#else
  void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len)
  {
    Wireless::getInstance()->recvCallback(mac, data, len);
  }
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
  void onEspNowSend(const esp_now_send_info_t *info, esp_now_send_status_t status)
  {
    Wireless::getInstance()->sendCallback(info != nullptr ? info->des_addr : nullptr, status);
  }
#else
  void onEspNowSend(const uint8_t *mac, esp_now_send_status_t status)
  {
    Wireless::getInstance()->sendCallback(mac, status);
  }
#endif
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
  if (incomingQueue_ == nullptr)
  {
    return;
  }

  // At most one queue's worth per call, so a sender that keeps the queue full
  // cannot hold the caller's loop forever.
  WirelessFrame frame;
  for (uint8_t handled = 0; handled < kRxQueueDepth; ++handled)
  {
    // A handler may call end(); stop dispatching once the transport is down.
    if (!setupDone.load(std::memory_order_acquire) ||
        xQueueReceive(incomingQueue_, &frame, 0) != pdTRUE)
    {
      return;
    }

    if (const std::shared_ptr<const ReceiveCallback> cb = receiveCb)
    {
      (*cb)(TransportAddress::fromMac(frame.mac), frame.packet);
    }
    if (!setupDone.load(std::memory_order_acquire))
    {
      return;
    }

    const auto it = onReceiveForCallbacks.find(frame.packet.type);
    const std::shared_ptr<const FrameHandler> handler =
        it != onReceiveForCallbacks.end() ? it->second : onReceiveOtherCb;
    if (handler)
    {
      (*handler)(&frame);
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
  lastStatus_.store(status, std::memory_order_relaxed);

#if DEBUG_ESP_NOW == 1
  if (mac_addr != nullptr)
  {
    Serial.printf("[ESP-NOW] sent to %02x:%02x:%02x:%02x:%02x:%02x: %s\n",
                  mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
                  status == ESP_NOW_SEND_SUCCESS ? "ok" : "fail");
  }
#else
  (void)mac_addr;
#endif
}

void Wireless::recvCallback(const uint8_t *mac_addr, const uint8_t *data, int len)
{
  constexpr int kHeaderLen = TransportPacket::kHeaderSize;
  if (!setupDone.load(std::memory_order_acquire) || mac_addr == nullptr ||
      data == nullptr || len < kHeaderLen || len > TransportPacket::kMaxFrame)
  {
    return;
  }

  uint16_t type = 0;
  uint16_t payloadLen = 0;
  memcpy(&type, data, sizeof(type));
  memcpy(&payloadLen, data + sizeof(type), sizeof(payloadLen));
  if (kHeaderLen + payloadLen != len)
  {
    return; // truncated, padded, or not our framing
  }

  WirelessFrame frame;
  frame.packet.type = type;
  frame.packet.len = payloadLen;

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
  onReceiveOtherCb = cb ? std::make_shared<const FrameHandler>(std::move(cb)) : nullptr;
}

void Wireless::addOnReceiveFor(uint16_t type,
                               std::function<void(WirelessFrame *frame)> cb)
{
  if (!cb)
  {
    onReceiveForCallbacks.erase(type);
    return;
  }
  onReceiveForCallbacks[type] = std::make_shared<const FrameHandler>(std::move(cb));
}

void Wireless::removeOnReceiveFor(uint16_t type)
{
  onReceiveForCallbacks.erase(type);
}

int Wireless::send(const TransportPacket *p, const uint8_t *peer_addr)
{
  if (p == nullptr || p->len > TransportPacket::kMaxData)
  {
    return -1;
  }
  return send(reinterpret_cast<const uint8_t *>(p),
              static_cast<uint16_t>(TransportPacket::kHeaderSize + p->len), peer_addr);
}

int Wireless::send(const TransportPacket *p, const TransportAddress &peer)
{
  if (peer.length != kAddressLength)
  {
    return -1;
  }
  return send(p, peer.data());
}

int Wireless::send(const uint8_t *data, uint16_t len, const uint8_t *peer_addr)
{
  if (!setupDone.load(std::memory_order_acquire) || data == nullptr ||
      len == 0 || len > TransportPacket::kMaxFrame)
  {
    return -1;
  }

  if (!ensurePeerRegistered(peer_addr))
  {
    return -1;
  }

  const esp_err_t err = esp_now_send(peer_addr, data, len);
  if (err != ESP_OK)
  {
#if DEBUG_ESP_NOW == 1
    Serial.printf("[ESP-NOW] send failed: %d\n", err);
#endif
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

  // Broadcast peer is added in begin() and lives outside the LRU cache.
  if (memcmp(peer_addr, kBroadcastMac, ESP_NOW_ETH_ALEN) == 0)
  {
    return esp_now_is_peer_exist(peer_addr) || addPeerToEspNow(peer_addr) == ESP_OK;
  }

  for (uint8_t i = 0; i < peerCacheSize_; ++i)
  {
    if (memcmp(peerCache_[i].data(), peer_addr, ESP_NOW_ETH_ALEN) == 0)
    {
      promoteToMru(i);
      // Re-add if something outside this class deleted it.
      if (esp_now_is_peer_exist(peer_addr) || addWithRoom(peer_addr, 1))
      {
        return true;
      }
      // Still gone: forget it so the cache matches the driver.
      --peerCacheSize_;
      return false;
    }
  }

  // Registered by someone else: use it, but leave it out of the cache so it is never evicted.
  if (esp_now_is_peer_exist(peer_addr))
  {
    return true;
  }

  if (peerCacheSize_ == kMaxCachedPeers)
  {
    evictLru();
  }
  if (!addWithRoom(peer_addr, 0))
  {
    return false;
  }
  memcpy(peerCache_[peerCacheSize_].data(), peer_addr, ESP_NOW_ETH_ALEN);
  ++peerCacheSize_;
  return true;
}

// Adds a peer; if ESP-NOW's table is full (other users hold slots), evicts our
// least recently used peer and retries once. `keep` MRU entries are never evicted.
bool Wireless::addWithRoom(const uint8_t *peer_addr, uint8_t keep)
{
  esp_err_t err = addPeerToEspNow(peer_addr);
  if (err == ESP_ERR_ESPNOW_FULL && peerCacheSize_ > keep)
  {
    evictLru();
    err = addPeerToEspNow(peer_addr);
  }
#if DEBUG_ESP_NOW == 1
  if (err != ESP_OK)
  {
    Serial.printf("[ESP-NOW] add peer failed: %d\n", err);
  }
#endif
  return err == ESP_OK;
}

esp_err_t Wireless::addPeerToEspNow(const uint8_t *peer_addr)
{
  esp_now_peer_info_t peerInfo{};
  memcpy(peerInfo.peer_addr, peer_addr, ESP_NOW_ETH_ALEN);
  peerInfo.channel = ESP_NOW_CHANNEL;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  const esp_err_t err = esp_now_add_peer(&peerInfo);
  return err == ESP_ERR_ESPNOW_EXIST ? ESP_OK : err;
}

void Wireless::evictLru()
{
  if (peerCacheSize_ == 0)
  {
    return;
  }
  esp_now_del_peer(peerCache_[0].data());
  for (uint8_t i = 0; i + 1 < peerCacheSize_; ++i)
  {
    peerCache_[i] = peerCache_[i + 1];
  }
  --peerCacheSize_;
}

void Wireless::promoteToMru(uint8_t index)
{
  if (index + 1 >= peerCacheSize_)
  {
    return;
  }
  const std::array<uint8_t, ESP_NOW_ETH_ALEN> saved = peerCache_[index];
  for (uint8_t i = index; i + 1 < peerCacheSize_; ++i)
  {
    peerCache_[i] = peerCache_[i + 1];
  }
  peerCache_[peerCacheSize_ - 1] = saved;
}

int Wireless::send(const uint8_t *data, uint16_t len, const TransportAddress &peer)
{
  if (peer.length != kAddressLength)
  {
    return -1;
  }
  return send(data, len, peer.data());
}

int Wireless::send(WirelessFrame *frame)
{
  if (frame == nullptr || frame->direction != PacketDirection::SEND)
  {
    return -1;
  }
  return send(&frame->packet, frame->mac);
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

  if (incomingQueue_ == nullptr)
  {
    incomingQueue_ = xQueueCreate(kRxQueueDepth, sizeof(WirelessFrame));
    if (incomingQueue_ == nullptr)
    {
      Serial.println("[ESP-NOW] no memory for the RX queue");
      return false;
    }
  }
  xQueueReset(incomingQueue_);
  droppedRxFrames_.store(0, std::memory_order_relaxed);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("[ESP-NOW] init failed");
    return false;
  }

  peerCacheSize_ = 0;
  if (esp_now_register_send_cb(onEspNowSend) != ESP_OK ||
      esp_now_register_recv_cb(onEspNowRecv) != ESP_OK ||
      addPeerToEspNow(kBroadcastMac) != ESP_OK)
  {
    Serial.println("[ESP-NOW] callback or broadcast peer setup failed");
    esp_now_deinit();
    return false;
  }
  broadcastPeerConfigured_ = true;

  setupDone.store(true, std::memory_order_release);
  return true;
}

void Wireless::end()
{
  if (setupDone.load(std::memory_order_acquire))
  {
    // Clear first so in-flight Wi-Fi callbacks stop queueing and loop() stops dispatching.
    setupDone.store(false, std::memory_order_release);
    esp_now_unregister_recv_cb();
    esp_now_unregister_send_cb();
    esp_now_deinit(); // also drops the peer table
    broadcastPeerConfigured_ = false;
    peerCacheSize_ = 0;
    xQueueReset(incomingQueue_);
  }
  // Registrations may capture objects the app destroys after end(); drop them.
  // A handler running now stays alive through loop()'s own reference.
  receiveCb = nullptr;
  onReceiveOtherCb = nullptr;
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
  receiveCb = cb ? std::make_shared<const ReceiveCallback>(std::move(cb)) : nullptr;
}

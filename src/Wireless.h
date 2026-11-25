#pragma once

#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <WiFi.h>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <type_traits>

#include "ITransport.h"

// ============================================================
// Wireless-layer frame (MAC + direction + payload)
// ============================================================

enum class PacketDirection
{
  SEND,
  RECV
};

struct WirelessFrame
{
  uint8_t mac[ESP_NOW_ETH_ALEN];
  PacketDirection direction;
  TransportPacket packet;
};

// ============================================================
// ESP-NOW transport implementation
// ============================================================

// One ESP-NOW transport per device. Callbacks registered here survive end()/begin(),
// so a radio restart needs no rewiring. Not thread-safe: send and loop from one task.
class Wireless : public ITransport
{
private:
  using FrameHandler = std::function<void(WirelessFrame *frame)>;

  static constexpr uint8_t kRxQueueDepth = 16;
  static constexpr uint8_t kAddressLength = ESP_NOW_ETH_ALEN;
  // ESP-NOW's peer table holds ~20 entries. The broadcast peer takes one,
  // so cap our unicast LRU at 18 (one slot of headroom).
  static constexpr uint8_t kMaxCachedPeers = 18;

  std::atomic<bool> setupDone{false};
  bool broadcastPeerConfigured_ = false;
  // Created once and never deleted: a Wi-Fi task callback can still be holding it
  // while end() runs.
  QueueHandle_t incomingQueue_ = nullptr;
  std::atomic<uint32_t> droppedRxFrames_{0};

  // LRU of peers this class registered. Index 0 is least-recently-used. Peers that
  // someone else registered are used but never cached, so they are never deleted.
  std::array<std::array<uint8_t, ESP_NOW_ETH_ALEN>, kMaxCachedPeers> peerCache_{};
  uint8_t peerCacheSize_ = 0;

  // shared_ptr so dispatch can hold a handler while it replaces or removes itself.
  std::shared_ptr<const FrameHandler> onReceiveOtherCb;
  std::map<uint16_t, std::shared_ptr<const FrameHandler>> onReceiveForCallbacks;
  std::shared_ptr<const ReceiveCallback> receiveCb;

public:
  static Wireless *getInstance(); // Non-owning singleton; do not delete.

  Wireless(const Wireless &) = delete;
  Wireless &operator=(const Wireless &) = delete;
  Wireless(Wireless &&) = delete;
  Wireless &operator=(Wireless &&) = delete;

  void setup();   // Backward-compatible alias for begin()
  void unSetup(); // Backward-compatible alias for end()
  void loop() override;

  bool isSetupDone() const;
  uint32_t droppedRxFrames() const { return droppedRxFrames_.load(std::memory_order_relaxed); }

  // Called from the Wi-Fi task by the ESP-NOW callback trampolines.
  void sendCallback(const uint8_t *mac_addr,
                    esp_now_send_status_t status);
  void recvCallback(const uint8_t *mac_addr, const uint8_t *data, int len);

  // Set the generic "other" callback.
  void setOnReceiveOther(std::function<void(WirelessFrame *frame)> cb);
  // Register a type-specific callback.
  void addOnReceiveFor(uint16_t type,
                       std::function<void(WirelessFrame *frame)> cb);
  // Remove a type-specific callback.
  void removeOnReceiveFor(uint16_t type);

  int send(const TransportPacket *p, const TransportAddress &peer);
  int send(const uint8_t *data, uint16_t len, const TransportAddress &peer);
  int send(const TransportPacket *p, const uint8_t *peer_addr);
  int send(const uint8_t *data, uint16_t len, const uint8_t *peer_addr);

  int send(WirelessFrame *frame);

  // ITransport
  bool begin() override;
  void end() override;
  bool isReady() const override;
  TransportAddress localAddress() const override;
  TransportAddress broadcastAddress() const override;
  esp_now_send_status_t getLastStatus() const;
  int sendPacket(const TransportPacket &packet, const TransportAddress &peer) override;
  void setReceiveCallback(ReceiveCallback cb) override;

  // --- Typed payload helpers (trivially copyable structs, at most TransportPacket::kMaxData bytes) ---

  template <typename T>
  int sendTyped(uint16_t type, const T &value, const uint8_t *peer_addr)
  {
    static_assert(std::is_trivially_copyable<T>::value,
                  "Wireless payload type must be trivially copyable");
    static_assert(sizeof(T) <= sizeof(TransportPacket{}.data),
                  "Wireless payload exceeds TransportPacket::data");
    TransportPacket pkt{};
    pkt.type = type;
    pkt.len = static_cast<uint16_t>(sizeof(T));
    std::memcpy(pkt.data, &value, sizeof(T));
    return send(&pkt, peer_addr);
  }

  template <typename T>
  int sendTyped(uint16_t type, const T &value, const TransportAddress &peer)
  {
    return sendTyped(type, value, peer.data());
  }

  int sendTyped(uint16_t type, const uint8_t *peer_addr)
  {
    TransportPacket pkt{};
    pkt.type = type;
    pkt.len = 0;
    return send(&pkt, peer_addr);
  }

  int sendTyped(uint16_t type, const TransportAddress &peer)
  {
    return sendTyped(type, peer.data());
  }

  template <typename T>
  static bool decodeTyped(uint16_t expectedType, const TransportPacket &pkt, T &out)
  {
    static_assert(std::is_trivially_copyable<T>::value,
                  "Wireless payload type must be trivially copyable");
    if (pkt.type != expectedType || pkt.len != sizeof(T))
    {
      return false;
    }
    std::memcpy(&out, pkt.data, sizeof(T));
    return true;
  }

  template <typename T>
  static bool decodeTyped(uint16_t expectedType, const WirelessFrame &frame, T &out)
  {
    return decodeTyped(expectedType, frame.packet, out);
  }

  template <typename T>
  static bool decodeTyped(uint16_t expectedType, const WirelessFrame *frame, T &out)
  {
    return frame != nullptr && decodeTyped(expectedType, frame->packet, out);
  }

  static bool decodeTyped(uint16_t expectedType, const TransportPacket &pkt)
  {
    return pkt.type == expectedType && pkt.len == 0;
  }

  static bool decodeTyped(uint16_t expectedType, const WirelessFrame &frame)
  {
    return decodeTyped(expectedType, frame.packet);
  }

  static bool decodeTyped(uint16_t expectedType, const WirelessFrame *frame)
  {
    return frame != nullptr && decodeTyped(expectedType, frame->packet);
  }

  template <typename T>
  void addOnReceiveTyped(uint16_t type, std::function<void(const uint8_t *mac, const T &)> cb)
  {
    if (!cb)
    {
      return;
    }
    addOnReceiveFor(type, [cb = std::move(cb), type](WirelessFrame *frame)
                    {
                      T value{};
                      if (decodeTyped(type, frame->packet, value))
                      {
                        cb(frame->mac, value);
                      }
                    });
  }

  void addOnReceiveTyped(uint16_t type, std::function<void(const uint8_t *mac)> cb)
  {
    if (!cb)
    {
      return;
    }
    addOnReceiveFor(type, [cb = std::move(cb), type](WirelessFrame *frame)
                    {
                      if (decodeTyped(type, frame->packet))
                      {
                        cb(frame->mac);
                      }
                    });
  }

  // ---- Backward-compatible aliases (property/event vocabulary) ----

  template <typename T>
  int sendProperty(uint16_t key, const T &value, const uint8_t *peer_addr) { return sendTyped(key, value, peer_addr); }
  template <typename T>
  int sendProperty(uint16_t key, const T &value, const TransportAddress &peer) { return sendTyped(key, value, peer); }

  template <typename T>
  int sendEvent(uint16_t channel, const T &value, const uint8_t *peer_addr) { return sendTyped(channel, value, peer_addr); }
  template <typename T>
  int sendEvent(uint16_t channel, const T &value, const TransportAddress &peer) { return sendTyped(channel, value, peer); }
  int sendEvent(uint16_t channel, const uint8_t *peer_addr) { return sendTyped(channel, peer_addr); }
  int sendEvent(uint16_t channel, const TransportAddress &peer) { return sendTyped(channel, peer); }

  template <typename T>
  static bool decodeProperty(uint16_t expectedKey, const TransportPacket &pkt, T &out) { return decodeTyped(expectedKey, pkt, out); }
  template <typename T>
  static bool decodeProperty(uint16_t expectedKey, const WirelessFrame &frame, T &out) { return decodeTyped(expectedKey, frame, out); }
  template <typename T>
  static bool decodeProperty(uint16_t expectedKey, const WirelessFrame *frame, T &out) { return decodeTyped(expectedKey, frame, out); }

  template <typename T>
  static bool decodeEvent(uint16_t expectedChannel, const TransportPacket &pkt, T &out) { return decodeTyped(expectedChannel, pkt, out); }
  template <typename T>
  static bool decodeEvent(uint16_t expectedChannel, const WirelessFrame &frame, T &out) { return decodeTyped(expectedChannel, frame, out); }
  template <typename T>
  static bool decodeEvent(uint16_t expectedChannel, const WirelessFrame *frame, T &out) { return decodeTyped(expectedChannel, frame, out); }
  static bool decodeEvent(uint16_t expectedChannel, const TransportPacket &pkt) { return decodeTyped(expectedChannel, pkt); }
  static bool decodeEvent(uint16_t expectedChannel, const WirelessFrame &frame) { return decodeTyped(expectedChannel, frame); }
  static bool decodeEvent(uint16_t expectedChannel, const WirelessFrame *frame) { return decodeTyped(expectedChannel, frame); }

  template <typename T>
  void addOnReceiveProperty(uint16_t key, std::function<void(const uint8_t *mac, const T &)> cb) { addOnReceiveTyped<T>(key, std::move(cb)); }
  template <typename T>
  void addOnReceiveEvent(uint16_t channel, std::function<void(const uint8_t *mac, const T &)> cb) { addOnReceiveTyped<T>(channel, std::move(cb)); }
  void addOnReceiveEvent(uint16_t channel, std::function<void(const uint8_t *mac)> cb) { addOnReceiveTyped(channel, std::move(cb)); }

private:
  Wireless();
  bool ensurePeerRegistered(const uint8_t *peer_addr);
  esp_err_t addPeerToEspNow(const uint8_t *peer_addr);
  void evictLru();
  void promoteToMru(uint8_t index);
  std::atomic<esp_now_send_status_t> lastStatus_{ESP_NOW_SEND_FAIL};
};
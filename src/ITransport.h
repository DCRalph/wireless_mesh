#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <functional>

// ============================================================
// Transport primitives (shared by Wireless + Mesh)
// ============================================================

struct TransportAddress
{
  static constexpr uint8_t kMaxLength = 16;

  uint8_t length = 0;
  std::array<uint8_t, kMaxLength> bytes{};

  bool isValid() const
  {
    return length > 0 && length <= bytes.size();
  }

  bool empty() const
  {
    return length == 0;
  }

  bool isFrom(const uint8_t *address, uint8_t addressLength) const
  {
    return length == addressLength &&
           (length == 0 || memcmp(bytes.data(), address, length) == 0);
  }

  const uint8_t *data() const
  {
    return bytes.data();
  }

  uint8_t *data()
  {
    return bytes.data();
  }

  static TransportAddress fromBytes(const uint8_t *address, uint8_t addressLength)
  {
    TransportAddress addr{};
    if (address == nullptr || addressLength == 0 || addressLength > addr.bytes.size())
    {
      return addr;
    }
    addr.length = addressLength;
    memcpy(addr.bytes.data(), address, addressLength);
    return addr;
  }

  static TransportAddress fromMac(const uint8_t *mac)
  {
    return fromBytes(mac, 6);
  }

  bool operator==(const TransportAddress &other) const
  {
    return length == other.length &&
           (length == 0 || memcmp(bytes.data(), other.bytes.data(), length) == 0);
  }

  bool operator!=(const TransportAddress &other) const
  {
    return !(*this == other);
  }

  void clear()
  {
    length = 0;
    bytes.fill(0);
  }
};

// On air a packet is type + len + len bytes of data. ESP-NOW v1 caps a frame at
// 250 bytes (ESP_NOW_MAX_DATA_LEN), so the header leaves 246 for data. Every
// layer above derives its limits from kMaxData.
struct __attribute__((packed)) TransportPacket
{
  static constexpr uint16_t kMaxFrame = 250;
  static constexpr uint16_t kHeaderSize = 4; // type + len
  static constexpr uint16_t kMaxData = kMaxFrame - kHeaderSize;

  uint16_t type = 0;
  uint16_t len = 0;
  uint8_t data[kMaxData]{};
};

static_assert(sizeof(TransportPacket) == TransportPacket::kMaxFrame, "TransportPacket must match one ESP-NOW frame");

class ITransport
{
public:
  using ReceiveCallback = std::function<void(const TransportAddress &source, const TransportPacket &packet)>;

  virtual ~ITransport() = default;
  virtual bool begin() = 0;
  virtual void end() = 0;
  virtual void loop() = 0;
  virtual bool isReady() const = 0;
  virtual TransportAddress localAddress() const = 0;
  virtual TransportAddress broadcastAddress() const = 0;
  virtual int sendPacket(const TransportPacket &packet, const TransportAddress &peer) = 0;
  virtual void setReceiveCallback(ReceiveCallback cb) = 0;
};
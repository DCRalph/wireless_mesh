#pragma once

#include <Arduino.h>
#include <cstdint>
#include <cstring>
#include <functional>
#include <map>
#include <type_traits>
#include <utility>
#include <vector>

#include "ITransport.h"

// ============================================================
// Direct ESP-to-ESP comms — built on the ITransport abstraction
// ============================================================
//
// DirectComms gives two devices a simple way to exchange typed structs
// without joining a mesh group. It is transport-agnostic: it depends
// only on ITransport (sendPacket + ReceiveCallback) and TransportAddress.
// Any transport that implements ITransport will work.
//
// Two primitives are provided:
//
//   DirectChannel<T>  — fire-and-forget typed send, with onReceive() callback.
//   DirectRpc<Req,R>  — request/response with timeout, async callback model.
//
// DirectComms uses a private TransportPacket type (DirectFraming::kDirectPacketType)
// that does not collide with the mesh layer's MeshConstants::kMeshPacketType.

namespace DirectFraming
{
  // Transport-level packet type used for every direct frame.
  // Distinct from the mesh layer's packet type so the two protocols
  // can share one transport without collision.
  constexpr uint16_t kDirectPacketType = 0xD17C;

  enum class Kind : uint8_t
  {
    OneWay = 1,
    Request = 2,
    Response = 3,
  };

  struct __attribute__((packed)) Header
  {
    uint8_t kind;
    uint16_t channelId;
    uint16_t requestId; // 0 for OneWay
    uint8_t payloadLen;
  };

  static_assert(sizeof(Header) == 6, "DirectFraming::Header must be 6 bytes");

  // Maximum user payload after the 6-byte header.
  constexpr uint16_t kMaxPayloadSize =
      sizeof(TransportPacket{}.data) - sizeof(Header);
}

class DirectComms;

template <typename T>
class DirectChannel
{
  static_assert(std::is_trivially_copyable<T>::value,
                "DirectChannel<T> requires a trivially copyable type");
  static_assert(sizeof(T) <= DirectFraming::kMaxPayloadSize,
                "DirectChannel<T> payload exceeds direct frame capacity");

public:
  DirectChannel() = default;

  bool sendTo(const TransportAddress &peer, const T &value) const;
  bool sendTo(const uint8_t *peerMac, const T &value) const; // 6-byte MAC convenience

  void onReceive(std::function<void(const TransportAddress &peer, const T &value)> cb) const;
  uint16_t channelId() const { return channelId_; }
  bool valid() const { return comms_ != nullptr; }

private:
  friend class DirectComms;
  DirectChannel(DirectComms *comms, uint16_t channelId)
      : comms_(comms), channelId_(channelId) {}

  DirectComms *comms_ = nullptr;
  uint16_t channelId_ = 0;
};

template <typename Req, typename Resp>
class DirectRpc
{
  static_assert(std::is_trivially_copyable<Req>::value,
                "DirectRpc<Req,Resp> requires a trivially copyable Req");
  static_assert(std::is_trivially_copyable<Resp>::value,
                "DirectRpc<Req,Resp> requires a trivially copyable Resp");
  static_assert(sizeof(Req) <= DirectFraming::kMaxPayloadSize,
                "DirectRpc Req payload exceeds direct frame capacity");
  static_assert(sizeof(Resp) <= DirectFraming::kMaxPayloadSize,
                "DirectRpc Resp payload exceeds direct frame capacity");

public:
  DirectRpc() = default;

  // Caller side. Sends `req` to `peer` and arms a timeout.
  // onResult fires exactly once: (true, decodedResp) on reply,
  // or (false, Resp{}) on timeout.
  // Returns false only if the send could not be scheduled at all.
  bool request(const TransportAddress &peer,
               const Req &req,
               uint32_t timeoutMs,
               std::function<void(bool ok, const Resp &resp)> onResult) const;

  bool request(const uint8_t *peerMac,
               const Req &req,
               uint32_t timeoutMs,
               std::function<void(bool ok, const Resp &resp)> onResult) const;

  // Responder side. The handler fills outResp and returns true to send
  // a response, false to drop the request silently.
  void onRequest(std::function<bool(const TransportAddress &peer,
                                    const Req &req,
                                    Resp &outResp)>
                     handler) const;

  uint16_t channelId() const { return channelId_; }
  bool valid() const { return comms_ != nullptr; }

private:
  friend class DirectComms;
  DirectRpc(DirectComms *comms, uint16_t channelId)
      : comms_(comms), channelId_(channelId) {}

  DirectComms *comms_ = nullptr;
  uint16_t channelId_ = 0;
};

class DirectComms
{
public:
  static DirectComms *getInstance(); // Non-owning singleton; do not delete.

  DirectComms(const DirectComms &) = delete;
  DirectComms &operator=(const DirectComms &) = delete;
  DirectComms(DirectComms &&) = delete;
  DirectComms &operator=(DirectComms &&) = delete;

  // Set the transport used for sending direct frames. Required before
  // sendTo() / request(). Does NOT touch the transport's receive
  // callback — wire that up separately (see receiveCallback() below).
  void setTransport(ITransport *transport);
  ITransport *getTransport() const;

  // Convenience: setTransport(transport) AND
  // transport->setReceiveCallback(receiveCallback()).
  // Use this when no other layer is installed on this transport's
  // setReceiveCallback (e.g. when SyncManager is NOT also active).
  // Returns false only if `transport` is null.
  bool begin(ITransport *transport);

  // Tear down: clears any pending RPC entries. Does not touch the
  // transport's receive callback — the caller installed it, the caller
  // owns it.
  void end();

  // Pumps RPC timeouts. Call from your Arduino loop().
  void loop();

  // Returns a ReceiveCallback that decodes direct frames and dispatches
  // to registered handlers. Wire it however suits your setup:
  //
  //   // Pure ITransport (DirectComms owns the only callback slot):
  //   transport->setReceiveCallback(direct->receiveCallback());
  //
  //   // Chain it for coexistence with another handler:
  //   transport->setReceiveCallback(
  //       [other, direct = direct->receiveCallback()](
  //           const TransportAddress &src, const TransportPacket &pkt) {
  //         direct(src, pkt);
  //         other(src, pkt);  // your other layer
  //       });
  //
  //   // Or feed it through a packet-type demuxer like Wireless::addOnReceiveFor.
  //
  // The returned callable captures `this` by pointer; do not keep it
  // alive past DirectComms.
  ITransport::ReceiveCallback receiveCallback();

  template <typename T>
  DirectChannel<T> channel(uint16_t channelId)
  {
    return DirectChannel<T>(this, channelId);
  }

  template <typename Req, typename Resp>
  DirectRpc<Req, Resp> rpc(uint16_t channelId)
  {
    return DirectRpc<Req, Resp>(this, channelId);
  }

  // ---- Internal helpers, public so template methods can reach them ----

  using OneWayHandler = std::function<void(const TransportAddress &peer,
                                           const uint8_t *data,
                                           uint8_t len)>;

  using RawRequestHandler = std::function<bool(const TransportAddress &peer,
                                               const uint8_t *reqData,
                                               uint8_t reqLen,
                                               uint8_t *outResp,
                                               uint8_t outRespCap,
                                               uint8_t &outRespLen)>;

  using PendingResponseCb = std::function<void(bool ok,
                                               const uint8_t *data,
                                               uint8_t len)>;

  bool sendOneWayRaw(uint16_t channelId,
                     const TransportAddress &peer,
                     const uint8_t *payload,
                     uint8_t payloadLen);

  bool sendRequestRaw(uint16_t channelId,
                      const TransportAddress &peer,
                      const uint8_t *payload,
                      uint8_t payloadLen,
                      uint32_t timeoutMs,
                      PendingResponseCb onResult);

  void registerOneWayHandler(uint16_t channelId, OneWayHandler cb);
  void registerRequestHandler(uint16_t channelId, RawRequestHandler handler);

  // Direct entry point for the receive callback. Public so receiveCallback()
  // can wrap it cleanly; users should normally go through receiveCallback().
  void handleIncoming(const TransportAddress &source, const TransportPacket &packet);

private:
  DirectComms() = default;

  bool sendFrame(const TransportAddress &peer,
                 DirectFraming::Kind kind,
                 uint16_t channelId,
                 uint16_t requestId,
                 const uint8_t *payload,
                 uint8_t payloadLen);
  uint16_t allocateRequestId();

  ITransport *transport_ = nullptr;
  uint16_t nextRequestId_ = 1;

  std::map<uint16_t, OneWayHandler> oneWayHandlers_;
  std::map<uint16_t, RawRequestHandler> requestHandlers_;

  struct PendingRequest
  {
    uint16_t requestId;
    uint16_t channelId;
    uint32_t deadlineMs;
    PendingResponseCb onResult;
  };
  std::vector<PendingRequest> pendingRequests_;
};

// ============================================================
// Template method implementations
// ============================================================

template <typename T>
bool DirectChannel<T>::sendTo(const TransportAddress &peer, const T &value) const
{
  if (comms_ == nullptr)
  {
    return false;
  }
  return comms_->sendOneWayRaw(channelId_,
                               peer,
                               reinterpret_cast<const uint8_t *>(&value),
                               static_cast<uint8_t>(sizeof(T)));
}

template <typename T>
bool DirectChannel<T>::sendTo(const uint8_t *peerMac, const T &value) const
{
  if (peerMac == nullptr)
  {
    return false;
  }
  return sendTo(TransportAddress::fromMac(peerMac), value);
}

template <typename T>
void DirectChannel<T>::onReceive(
    std::function<void(const TransportAddress &peer, const T &value)> cb) const
{
  if (comms_ == nullptr || !cb)
  {
    return;
  }
  comms_->registerOneWayHandler(
      channelId_,
      [cb = std::move(cb)](const TransportAddress &peer,
                           const uint8_t *data,
                           uint8_t len)
      {
        if (data == nullptr || len != sizeof(T))
        {
          return;
        }
        T value{};
        std::memcpy(&value, data, sizeof(T));
        cb(peer, value);
      });
}

template <typename Req, typename Resp>
bool DirectRpc<Req, Resp>::request(
    const TransportAddress &peer,
    const Req &req,
    uint32_t timeoutMs,
    std::function<void(bool ok, const Resp &resp)> onResult) const
{
  if (comms_ == nullptr || !onResult)
  {
    return false;
  }
  return comms_->sendRequestRaw(
      channelId_,
      peer,
      reinterpret_cast<const uint8_t *>(&req),
      static_cast<uint8_t>(sizeof(Req)),
      timeoutMs,
      [onResult = std::move(onResult)](bool ok,
                                       const uint8_t *data,
                                       uint8_t len)
      {
        Resp value{};
        if (ok && data != nullptr && len == sizeof(Resp))
        {
          std::memcpy(&value, data, sizeof(Resp));
          onResult(true, value);
        }
        else
        {
          onResult(false, value);
        }
      });
}

template <typename Req, typename Resp>
bool DirectRpc<Req, Resp>::request(
    const uint8_t *peerMac,
    const Req &req,
    uint32_t timeoutMs,
    std::function<void(bool ok, const Resp &resp)> onResult) const
{
  if (peerMac == nullptr)
  {
    return false;
  }
  return request(TransportAddress::fromMac(peerMac), req, timeoutMs, std::move(onResult));
}

template <typename Req, typename Resp>
void DirectRpc<Req, Resp>::onRequest(
    std::function<bool(const TransportAddress &peer,
                       const Req &req,
                       Resp &outResp)>
        handler) const
{
  if (comms_ == nullptr || !handler)
  {
    return;
  }
  comms_->registerRequestHandler(
      channelId_,
      [handler = std::move(handler)](const TransportAddress &peer,
                                     const uint8_t *reqData,
                                     uint8_t reqLen,
                                     uint8_t *outResp,
                                     uint8_t outRespCap,
                                     uint8_t &outRespLen) -> bool
      {
        if (reqData == nullptr || reqLen != sizeof(Req))
        {
          outRespLen = 0;
          return false;
        }
        if (outRespCap < sizeof(Resp))
        {
          outRespLen = 0;
          return false;
        }
        Req req{};
        std::memcpy(&req, reqData, sizeof(Req));
        Resp resp{};
        if (!handler(peer, req, resp))
        {
          outRespLen = 0;
          return false;
        }
        std::memcpy(outResp, &resp, sizeof(Resp));
        outRespLen = static_cast<uint8_t>(sizeof(Resp));
        return true;
      });
}

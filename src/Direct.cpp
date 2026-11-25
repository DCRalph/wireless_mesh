#include "Direct.h"

#include <algorithm>

namespace
{
  bool deadlinePassed(uint32_t now, uint32_t deadlineMs)
  {
    return static_cast<int32_t>(now - deadlineMs) >= 0;
  }
} // namespace

DirectComms *DirectComms::getInstance()
{
  static DirectComms instance;
  return &instance;
}

void DirectComms::setTransport(ITransport *transport)
{
  transport_ = transport;
}

ITransport *DirectComms::getTransport() const
{
  return transport_;
}

bool DirectComms::begin(ITransport *transport)
{
  if (transport == nullptr)
  {
    return false;
  }
  transport_ = transport;
  transport_->setReceiveCallback(receiveCallback());
  return true;
}

void DirectComms::end()
{
  // Detach first: a callback may issue a new request while we complete these.
  std::vector<PendingRequest> cancelled;
  cancelled.swap(pendingRequests_);
  for (auto &pending : cancelled)
  {
    if (pending.onResult)
    {
      pending.onResult(false, nullptr, 0);
    }
  }
}

void DirectComms::loop()
{
  if (pendingRequests_.empty())
  {
    return;
  }
  const uint32_t now = millis();

  // Drain first so a callback can re-enter request() without invalidating iteration.
  std::vector<PendingResponseCb> expired;
  for (auto it = pendingRequests_.begin(); it != pendingRequests_.end();)
  {
    if (deadlinePassed(now, it->deadlineMs))
    {
      expired.push_back(std::move(it->onResult));
      it = pendingRequests_.erase(it);
    }
    else
    {
      ++it;
    }
  }
  for (auto &cb : expired)
  {
    if (cb)
    {
      cb(false, nullptr, 0);
    }
  }
}

ITransport::ReceiveCallback DirectComms::receiveCallback()
{
  return [this](const TransportAddress &source, const TransportPacket &packet)
  { handleIncoming(source, packet); };
}

bool DirectComms::isPending(uint16_t requestId) const
{
  for (const auto &pending : pendingRequests_)
  {
    if (pending.requestId == requestId)
    {
      return true;
    }
  }
  return false;
}

uint16_t DirectComms::allocateRequestId()
{
  // Skip 0 (OneWay frames) and any id still awaiting a reply, so a late reply to
  // an old request can never complete a new one.
  for (uint32_t tries = 0; tries < 0x10000u; ++tries)
  {
    const uint16_t id = nextRequestId_++;
    if (id != 0 && !isPending(id))
    {
      return id;
    }
  }
  return 0; // 65535 requests in flight
}

bool DirectComms::takePending(uint16_t requestId, PendingResponseCb &out)
{
  for (auto it = pendingRequests_.begin(); it != pendingRequests_.end(); ++it)
  {
    if (it->requestId == requestId)
    {
      out = std::move(it->onResult);
      pendingRequests_.erase(it);
      return true;
    }
  }
  return false;
}

bool DirectComms::sendFrame(const TransportAddress &peer,
                            DirectFraming::Kind kind,
                            uint16_t channelId,
                            uint16_t requestId,
                            const uint8_t *payload,
                            uint8_t payloadLen)
{
  if (transport_ == nullptr || !peer.isValid())
  {
    return false;
  }
  if (payloadLen > DirectFraming::kMaxPayloadSize)
  {
    return false;
  }

  TransportPacket pkt;
  pkt.type = DirectFraming::kDirectPacketType;
  pkt.len = static_cast<uint16_t>(sizeof(DirectFraming::Header) + payloadLen);

  DirectFraming::Header hdr{};
  hdr.kind = static_cast<uint8_t>(kind);
  hdr.channelId = channelId;
  hdr.requestId = requestId;
  hdr.payloadLen = payloadLen;
  std::memcpy(pkt.data, &hdr, sizeof(hdr));
  if (payloadLen > 0 && payload != nullptr)
  {
    std::memcpy(pkt.data + sizeof(hdr), payload, payloadLen);
  }

  return transport_->sendPacket(pkt, peer) == 0;
}

bool DirectComms::sendOneWayRaw(uint16_t channelId,
                                const TransportAddress &peer,
                                const uint8_t *payload,
                                uint8_t payloadLen)
{
  return sendFrame(peer,
                   DirectFraming::Kind::OneWay,
                   channelId,
                   0,
                   payload,
                   payloadLen);
}

bool DirectComms::sendRequestRaw(uint16_t channelId,
                                 const TransportAddress &peer,
                                 const uint8_t *payload,
                                 uint8_t payloadLen,
                                 uint32_t timeoutMs,
                                 PendingResponseCb onResult)
{
  if (!onResult)
  {
    return false;
  }
  const uint16_t requestId = allocateRequestId();
  if (requestId == 0)
  {
    onResult(false, nullptr, 0);
    return false;
  }

  // Register before sending: a synchronous transport may deliver the reply from
  // inside sendPacket().
  PendingRequest entry;
  entry.requestId = requestId;
  entry.channelId = channelId;
  entry.peer = peer;
  entry.deadlineMs = millis() + std::min(timeoutMs, DirectFraming::kMaxTimeoutMs);
  entry.onResult = std::move(onResult);
  pendingRequests_.push_back(std::move(entry));

  if (!sendFrame(peer,
                 DirectFraming::Kind::Request,
                 channelId,
                 requestId,
                 payload,
                 payloadLen))
  {
    PendingResponseCb cb;
    if (takePending(requestId, cb) && cb)
    {
      cb(false, nullptr, 0);
    }
    return false;
  }
  return true;
}

void DirectComms::registerOneWayHandler(uint16_t channelId, OneWayHandler cb)
{
  if (!cb)
  {
    oneWayHandlers_.erase(channelId);
    return;
  }
  oneWayHandlers_[channelId] = std::make_shared<const OneWayHandler>(std::move(cb));
}

void DirectComms::registerRequestHandler(uint16_t channelId,
                                         RawRequestHandler handler)
{
  if (!handler)
  {
    requestHandlers_.erase(channelId);
    return;
  }
  requestHandlers_[channelId] = std::make_shared<const RawRequestHandler>(std::move(handler));
}

void DirectComms::handleIncoming(const TransportAddress &source,
                                 const TransportPacket &packet)
{
  if (packet.type != DirectFraming::kDirectPacketType)
  {
    return;
  }
  if (packet.len < sizeof(DirectFraming::Header) || packet.len > TransportPacket::kMaxData)
  {
    return;
  }

  DirectFraming::Header packed{};
  std::memcpy(&packed, packet.data, sizeof(packed));
  // Copy out of the packed header: references to its fields (std::map::find takes
  // one) would be misaligned, which faults on Xtensa.
  struct
  {
    uint8_t kind;
    uint16_t channelId;
    uint16_t requestId;
    uint8_t payloadLen;
  } const hdr{packed.kind, packed.channelId, packed.requestId, packed.payloadLen};

  const uint16_t expectedLen =
      static_cast<uint16_t>(sizeof(DirectFraming::Header)) + hdr.payloadLen;
  if (packet.len != expectedLen || hdr.payloadLen > DirectFraming::kMaxPayloadSize)
  {
    return;
  }

  const uint8_t *payload = packet.data + sizeof(DirectFraming::Header);

  switch (static_cast<DirectFraming::Kind>(hdr.kind))
  {
  case DirectFraming::Kind::OneWay:
  {
    const auto it = oneWayHandlers_.find(hdr.channelId);
    if (it == oneWayHandlers_.end())
    {
      return;
    }
    const std::shared_ptr<const OneWayHandler> handler = it->second;
    (*handler)(source, payload, hdr.payloadLen);
    break;
  }
  case DirectFraming::Kind::Request:
  {
    const auto it = requestHandlers_.find(hdr.channelId);
    if (it == requestHandlers_.end() || hdr.requestId == 0)
    {
      return;
    }
    const std::shared_ptr<const RawRequestHandler> handler = it->second;
    uint8_t respBuf[DirectFraming::kMaxPayloadSize];
    uint8_t respLen = 0;
    if (!(*handler)(source,
                    payload,
                    hdr.payloadLen,
                    respBuf,
                    static_cast<uint8_t>(sizeof(respBuf)),
                    respLen))
    {
      return;
    }
    sendFrame(source,
              DirectFraming::Kind::Response,
              hdr.channelId,
              hdr.requestId,
              respBuf,
              respLen);
    break;
  }
  case DirectFraming::Kind::Response:
  {
    const uint32_t now = millis();
    for (auto it = pendingRequests_.begin(); it != pendingRequests_.end(); ++it)
    {
      if (it->requestId != hdr.requestId || it->channelId != hdr.channelId)
      {
        continue;
      }
      // Only the peer that was asked may answer, unless the request was a broadcast.
      const bool askedAll = transport_ != nullptr && it->peer == transport_->broadcastAddress();
      if (source != it->peer && !askedAll)
      {
        return;
      }
      PendingResponseCb cb = std::move(it->onResult);
      const bool inTime = !deadlinePassed(now, it->deadlineMs);
      pendingRequests_.erase(it);
      if (cb)
      {
        cb(inTime, inTime ? payload : nullptr, inTime ? hdr.payloadLen : 0);
      }
      return;
    }
    break;
  }
  default:
    break;
  }
}

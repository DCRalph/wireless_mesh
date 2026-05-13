#include "Direct.h"

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
  pendingRequests_.clear();
}

void DirectComms::loop()
{
  if (pendingRequests_.empty())
  {
    return;
  }
  const uint32_t now = millis();

  // Drain first so a callback can re-enter request()/cancel without invalidating iteration.
  std::vector<PendingResponseCb> expired;
  for (auto it = pendingRequests_.begin(); it != pendingRequests_.end();)
  {
    if (static_cast<int32_t>(now - it->deadlineMs) >= 0)
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

uint16_t DirectComms::allocateRequestId()
{
  uint16_t id = nextRequestId_++;
  if (nextRequestId_ == 0)
  {
    nextRequestId_ = 1; // 0 is reserved for OneWay frames
  }
  return id;
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

  TransportPacket pkt{};
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
  if (!sendFrame(peer,
                 DirectFraming::Kind::Request,
                 channelId,
                 requestId,
                 payload,
                 payloadLen))
  {
    // Don't enqueue a pending request the radio refused.
    onResult(false, nullptr, 0);
    return false;
  }

  PendingRequest entry;
  entry.requestId = requestId;
  entry.channelId = channelId;
  entry.deadlineMs = millis() + timeoutMs;
  entry.onResult = std::move(onResult);
  pendingRequests_.push_back(std::move(entry));
  return true;
}

void DirectComms::registerOneWayHandler(uint16_t channelId, OneWayHandler cb)
{
  if (!cb)
  {
    oneWayHandlers_.erase(channelId);
    return;
  }
  oneWayHandlers_[channelId] = std::move(cb);
}

void DirectComms::registerRequestHandler(uint16_t channelId,
                                         RawRequestHandler handler)
{
  if (!handler)
  {
    requestHandlers_.erase(channelId);
    return;
  }
  requestHandlers_[channelId] = std::move(handler);
}

void DirectComms::handleIncoming(const TransportAddress &source,
                                 const TransportPacket &packet)
{
  if (packet.type != DirectFraming::kDirectPacketType)
  {
    return;
  }
  if (packet.len < sizeof(DirectFraming::Header))
  {
    return;
  }

  DirectFraming::Header hdr{};
  std::memcpy(&hdr, packet.data, sizeof(hdr));

  const uint16_t expectedLen =
      static_cast<uint16_t>(sizeof(DirectFraming::Header)) + hdr.payloadLen;
  if (packet.len != expectedLen)
  {
    return;
  }
  if (hdr.payloadLen > DirectFraming::kMaxPayloadSize)
  {
    return;
  }

  const uint8_t *payload = packet.data + sizeof(DirectFraming::Header);

  switch (static_cast<DirectFraming::Kind>(hdr.kind))
  {
  case DirectFraming::Kind::OneWay:
  {
    auto it = oneWayHandlers_.find(hdr.channelId);
    if (it != oneWayHandlers_.end() && it->second)
    {
      it->second(source, payload, hdr.payloadLen);
    }
    break;
  }
  case DirectFraming::Kind::Request:
  {
    auto it = requestHandlers_.find(hdr.channelId);
    if (it == requestHandlers_.end() || !it->second)
    {
      return;
    }
    uint8_t respBuf[DirectFraming::kMaxPayloadSize];
    uint8_t respLen = 0;
    if (!it->second(source,
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
    for (size_t i = 0; i < pendingRequests_.size(); ++i)
    {
      if (pendingRequests_[i].requestId == hdr.requestId &&
          pendingRequests_[i].channelId == hdr.channelId)
      {
        auto cb = std::move(pendingRequests_[i].onResult);
        pendingRequests_.erase(pendingRequests_.begin() + i);
        if (cb)
        {
          cb(true, payload, hdr.payloadLen);
        }
        return;
      }
    }
    break;
  }
  default:
    break;
  }
}

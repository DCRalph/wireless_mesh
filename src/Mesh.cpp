#include "Mesh.h"
#include <Arduino.h>
#include <algorithm>
#include <cstring>
#include <stdio.h>
#include <utility>
#include <vector>

#ifndef DEBUG_ESP_NOW
#define DEBUG_ESP_NOW 0
#endif

namespace
{
  template <typename T>
  bool appendStruct(std::vector<uint8_t> &out, const T &value)
  {
    const size_t oldSize = out.size();
    out.resize(oldSize + sizeof(T));
    memcpy(out.data() + oldSize, &value, sizeof(T));
    return true;
  }

  template <typename T>
  bool readStructAt(const uint8_t *payload, uint16_t payloadLen, size_t &offset, T &out)
  {
    if (payload == nullptr || offset + sizeof(T) > payloadLen)
    {
      return false;
    }
    memcpy(&out, payload + offset, sizeof(T));
    offset += sizeof(T);
    return true;
  }

  template <typename T>
  bool decodePayload(const MeshProtocol::DecodedFrame &frame, T &out)
  {
    size_t offset = 0;
    return readStructAt(frame.payload, frame.payloadLen, offset, out) &&
           offset == frame.payloadLen;
  }

  bool appendAddress(std::vector<uint8_t> &out, const TransportAddress &address)
  {
    if (!address.isValid())
    {
      return false;
    }
    out.push_back(address.length);
    out.insert(out.end(), address.data(), address.data() + address.length);
    return true;
  }

  bool readAddressAt(const uint8_t *payload,
                     uint16_t payloadLen,
                     size_t &offset,
                     TransportAddress &outAddress)
  {
    if (payload == nullptr || offset >= payloadLen)
    {
      return false;
    }
    const uint8_t addressLength = payload[offset++];
    if (addressLength == 0 || addressLength > TransportAddress::kMaxLength ||
        offset + addressLength > payloadLen)
    {
      return false;
    }
    outAddress = TransportAddress::fromBytes(payload + offset, addressLength);
    offset += addressLength;
    return outAddress.isValid();
  }

  struct __attribute__((packed)) HeartbeatHeader
  {
    uint32_t deviceId;
    uint8_t syncMode;
  };

  struct __attribute__((packed)) TimeRequestCmd
  {
    uint32_t requestTimestamp;
    uint32_t requesterDeviceId;
    uint32_t groupId;
  };

  struct __attribute__((packed)) TimeResponseCmd
  {
    uint32_t requestTimestamp;
    uint32_t masterTimestamp;
    uint32_t requesterDeviceId;
    uint32_t groupId;
  };

  struct __attribute__((packed)) GroupAnnounceHeader
  {
    uint32_t groupId;
    uint32_t masterDeviceId;
  };

  struct __attribute__((packed)) GroupJoinHeader
  {
    uint32_t groupId;
    uint32_t deviceId;
  };

  struct __attribute__((packed)) GroupInfoHeader
  {
    uint32_t groupId;
    uint32_t masterDeviceId;
    uint8_t memberCount;
  };

  struct __attribute__((packed)) GroupInfoMemberHeader
  {
    uint32_t deviceId;
  };

  struct HeartbeatCmd
  {
    uint32_t deviceId = 0;
    SyncMode syncMode = SyncMode::SOLO;
    TransportAddress address;
  };

  struct GroupAnnounceCmd
  {
    uint32_t groupId = 0;
    uint32_t masterDeviceId = 0;
    TransportAddress masterAddress;
  };

  struct GroupJoinCmd
  {
    uint32_t groupId = 0;
    uint32_t deviceId = 0;
    TransportAddress deviceAddress;
  };

  struct GroupInfoMember
  {
    uint32_t deviceId = 0;
    TransportAddress address;
  };

  struct GroupInfoCmd
  {
    uint32_t groupId = 0;
    uint32_t masterDeviceId = 0;
    std::vector<GroupInfoMember> members;
  };

  bool decodeHeartbeatCmd(const MeshProtocol::DecodedFrame &frame, HeartbeatCmd &out)
  {
    size_t offset = 0;
    HeartbeatHeader header{};
    if (!readStructAt(frame.payload, frame.payloadLen, offset, header) ||
        !readAddressAt(frame.payload, frame.payloadLen, offset, out.address) ||
        offset != frame.payloadLen)
    {
      return false;
    }
    out.deviceId = header.deviceId;
    out.syncMode = static_cast<SyncMode>(header.syncMode);
    return true;
  }

  bool decodeGroupAnnounceCmd(const MeshProtocol::DecodedFrame &frame, GroupAnnounceCmd &out)
  {
    size_t offset = 0;
    GroupAnnounceHeader header{};
    if (!readStructAt(frame.payload, frame.payloadLen, offset, header) ||
        !readAddressAt(frame.payload, frame.payloadLen, offset, out.masterAddress) ||
        offset != frame.payloadLen)
    {
      return false;
    }
    out.groupId = header.groupId;
    out.masterDeviceId = header.masterDeviceId;
    return true;
  }

  bool decodeGroupJoinCmd(const MeshProtocol::DecodedFrame &frame, GroupJoinCmd &out)
  {
    size_t offset = 0;
    GroupJoinHeader header{};
    if (!readStructAt(frame.payload, frame.payloadLen, offset, header) ||
        !readAddressAt(frame.payload, frame.payloadLen, offset, out.deviceAddress) ||
        offset != frame.payloadLen)
    {
      return false;
    }
    out.groupId = header.groupId;
    out.deviceId = header.deviceId;
    return true;
  }

  bool decodeGroupInfoCmd(const MeshProtocol::DecodedFrame &frame, GroupInfoCmd &out)
  {
    size_t offset = 0;
    GroupInfoHeader header{};
    if (!readStructAt(frame.payload, frame.payloadLen, offset, header))
    {
      return false;
    }

    out.groupId = header.groupId;
    out.masterDeviceId = header.masterDeviceId;
    out.members.clear();
    out.members.reserve(header.memberCount);

    for (uint8_t i = 0; i < header.memberCount; ++i)
    {
      GroupInfoMemberHeader memberHeader{};
      GroupInfoMember member{};
      if (!readStructAt(frame.payload, frame.payloadLen, offset, memberHeader) ||
          !readAddressAt(frame.payload, frame.payloadLen, offset, member.address))
      {
        return false;
      }
      member.deviceId = memberHeader.deviceId;
      out.members.push_back(member);
    }

    return offset == frame.payloadLen;
  }
} // namespace

// ============================================================
// MeshProtocol implementation
// ============================================================

namespace MeshProtocol
{
  bool decode(const TransportPacket &pkt, DecodedFrame &out)
  {
    if (pkt.type != MeshConstants::kMeshPacketType)
    {
      return false;
    }
    if (pkt.len < sizeof(FrameHeader))
    {
      return false;
    }

    FrameHeader header{};
    memcpy(&header, pkt.data, sizeof(header));
    if (header.version != MeshConstants::kProtocolVersion)
    {
      return false;
    }
    if (sizeof(header) + header.payloadLen > pkt.len)
    {
      return false;
    }

    out.kind = static_cast<MessageKind>(header.kind);
    out.command = header.command;
    out.flags = header.flags;
    out.originDeviceId = header.originDeviceId;
    out.sequence = header.sequence;
    out.hopLimit = header.hopLimit;
    out.payloadLen = header.payloadLen;
    out.payload = pkt.data + sizeof(header);
    return true;
  }

  bool encode(TransportPacket &pkt,
              MessageKind kind,
              uint8_t command,
              uint8_t flags,
              uint32_t originDeviceId,
              uint32_t sequence,
              uint8_t hopLimit,
              const uint8_t *payload,
              uint16_t payloadLen)
  {
    if (sizeof(FrameHeader) + payloadLen > sizeof(pkt.data))
    {
      return false;
    }

    pkt.type = MeshConstants::kMeshPacketType;
    FrameHeader header{};
    header.version = MeshConstants::kProtocolVersion;
    header.kind = static_cast<uint8_t>(kind);
    header.command = command;
    header.flags = flags;
    header.originDeviceId = originDeviceId;
    header.sequence = sequence;
    header.hopLimit = hopLimit;
    header.payloadLen = payloadLen;

    memcpy(pkt.data, &header, sizeof(header));
    if (payloadLen > 0 && payload != nullptr)
    {
      memcpy(pkt.data + sizeof(header), payload, payloadLen);
    }

    pkt.len = sizeof(header) + payloadLen;
    return true;
  }

  bool readPropertyEntryAt(const uint8_t *payload,
                           uint16_t payloadLen,
                           uint16_t offset,
                           uint16_t &nextOffset,
                           PropertyEntryHeader &outHeader,
                           const uint8_t *&outValue)
  {
    if (payload == nullptr || offset > payloadLen || payloadLen - offset < sizeof(PropertyEntryHeader))
    {
      return false;
    }
    memcpy(&outHeader, payload + offset, sizeof(PropertyEntryHeader));
    const uint16_t entryLen = static_cast<uint16_t>(sizeof(PropertyEntryHeader) + outHeader.valueLen);
    if (payloadLen - offset < entryLen)
    {
      return false;
    }

    outValue = payload + offset + sizeof(PropertyEntryHeader);
    nextOffset = static_cast<uint16_t>(offset + entryLen);
    return true;
  }

  bool appendPropertyEntry(std::vector<uint8_t> &out,
                           uint16_t key,
                           uint16_t revision,
                           const uint8_t *value,
                           uint8_t valueLen)
  {
    if (valueLen > MeshConstants::kMaxPropertyValueSize)
    {
      return false;
    }
    const size_t oldSize = out.size();
    out.resize(oldSize + sizeof(PropertyEntryHeader) + valueLen);
    PropertyEntryHeader header{};
    header.key = key;
    header.revision = revision;
    header.valueLen = valueLen;
    memcpy(out.data() + oldSize, &header, sizeof(header));
    if (valueLen > 0 && value != nullptr)
    {
      memcpy(out.data() + oldSize + sizeof(header), value, valueLen);
    }
    return true;
  }

  bool buildEventPayload(std::vector<uint8_t> &out,
                         uint16_t channelId,
                         const uint8_t *payload,
                         uint8_t payloadLen)
  {
    if (payloadLen > MeshConstants::kMaxEventPayloadSize)
    {
      return false;
    }
    out.resize(sizeof(EventHeader) + payloadLen);
    EventHeader header{};
    header.channelId = channelId;
    header.payloadLen = payloadLen;
    memcpy(out.data(), &header, sizeof(header));
    if (payloadLen > 0 && payload != nullptr)
    {
      memcpy(out.data() + sizeof(header), payload, payloadLen);
    }
    return true;
  }

  bool readEventPayload(const uint8_t *payload,
                        uint16_t payloadLen,
                        EventHeader &outHeader,
                        const uint8_t *&outEventPayload)
  {
    if (payload == nullptr || payloadLen < sizeof(EventHeader))
    {
      return false;
    }
    memcpy(&outHeader, payload, sizeof(outHeader));
    if (payloadLen < sizeof(EventHeader) + outHeader.payloadLen)
    {
      return false;
    }
    outEventPayload = payload + sizeof(EventHeader);
    return true;
  }

  bool isNewerRevision(uint16_t incoming, uint16_t stored)
  {
    if (incoming == stored)
    {
      return false;
    }
    const uint16_t diff = static_cast<uint16_t>(incoming - stored);
    return diff < 32768;
  }
} // namespace MeshProtocol

// ============================================================
// OffsetClock implementation
// ============================================================

void OffsetClock::reset()
{
  locked_ = false;
  offsetMs_ = 0;
  lastRttMs_ = 0;
}

void OffsetClock::setMasterReference()
{
  locked_ = true;
  offsetMs_ = 0;
  lastRttMs_ = 0;
}

bool OffsetClock::handleSample(uint32_t requestTimestamp, uint32_t masterTimestamp, uint32_t nowMillis)
{
  const uint32_t rtt = nowMillis - requestTimestamp;
  lastRttMs_ = rtt;
  if (rtt > MeshConstants::kMaxAcceptedRttMs)
  {
    return false;
  }

  const int32_t newOffset = static_cast<int32_t>(masterTimestamp) +
                            static_cast<int32_t>(rtt / 2) -
                            static_cast<int32_t>(nowMillis);
  if (!locked_)
  {
    offsetMs_ = newOffset;
    locked_ = true;
    return true;
  }

  // Light smoothing to reduce jitter while still tracking drift.
  offsetMs_ = (offsetMs_ * 3 + newOffset) / 4;
  return true;
}

bool OffsetClock::isLocked() const
{
  return locked_;
}

int32_t OffsetClock::offsetMillis() const
{
  return offsetMs_;
}

uint32_t OffsetClock::nowSyncedMillis(uint32_t localNow) const
{
  return locked_ ? localNow + offsetMs_ : localNow;
}

uint32_t OffsetClock::lastRttMillis() const
{
  return lastRttMs_;
}

PropertyStore::PropertyEntry *PropertyStore::findEntry(uint16_t key, uint32_t ownerDeviceId)
{
  for (auto &entry : entries_)
  {
    if (entry.key == key && entry.ownerDeviceId == ownerDeviceId)
    {
      return &entry;
    }
  }
  return nullptr;
}

const PropertyStore::PropertyEntry *PropertyStore::findEntry(uint16_t key, uint32_t ownerDeviceId) const
{
  for (const auto &entry : entries_)
  {
    if (entry.key == key && entry.ownerDeviceId == ownerDeviceId)
    {
      return &entry;
    }
  }
  return nullptr;
}

uint16_t PropertyStore::nextRevision(uint16_t current) const
{
  const uint16_t next = static_cast<uint16_t>(current + 1);
  return next == 0 ? 1 : next;
}

bool PropertyStore::setLocal(uint16_t key,
                             uint32_t localDeviceId,
                             const uint8_t *value,
                             uint8_t valueLen,
                             uint32_t nowMs,
                             PropertySnapshot &outSnapshot)
{
  if (valueLen > MeshConstants::kMaxPropertyValueSize || (valueLen > 0 && value == nullptr))
  {
    return false;
  }

  PropertyEntry *entry = findEntry(key, localDeviceId);
  if (entry == nullptr)
  {
    if (entries_.size() >= MeshConstants::kMaxPropertyEntries)
    {
      return false;
    }
    entries_.push_back(PropertyEntry{});
    entry = &entries_.back();
    entry->key = key;
    entry->ownerDeviceId = localDeviceId;
    entry->revision = 0;
  }

  entry->revision = nextRevision(entry->revision);
  entry->valueLen = valueLen;
  if (valueLen > 0)
  {
    memcpy(entry->value.data(), value, valueLen);
  }
  entry->lastUpdateMs = nowMs;

  outSnapshot.key = key;
  outSnapshot.ownerDeviceId = localDeviceId;
  outSnapshot.revision = entry->revision;
  outSnapshot.payload.assign(entry->value.data(), entry->value.data() + entry->valueLen);
  return true;
}

bool PropertyStore::applyRemote(uint16_t key,
                                uint32_t ownerDeviceId,
                                uint16_t revision,
                                const uint8_t *value,
                                uint8_t valueLen,
                                uint32_t nowMs,
                                PropertySnapshot &outSnapshot,
                                bool &outChanged)
{
  outChanged = false;
  if (revision == 0 || valueLen > MeshConstants::kMaxPropertyValueSize || (valueLen > 0 && value == nullptr))
  {
    return false;
  }

  PropertyEntry *entry = findEntry(key, ownerDeviceId);
  if (entry == nullptr)
  {
    if (entries_.size() >= MeshConstants::kMaxPropertyEntries)
    {
      return false;
    }
    entries_.push_back(PropertyEntry{});
    entry = &entries_.back();
    entry->key = key;
    entry->ownerDeviceId = ownerDeviceId;
    outChanged = true;
  }
  else if (!MeshProtocol::isNewerRevision(revision, entry->revision))
  {
    return true;
  }
  else
  {
    outChanged = true;
  }

  entry->revision = revision;
  entry->valueLen = valueLen;
  if (valueLen > 0)
  {
    memcpy(entry->value.data(), value, valueLen);
  }
  entry->lastUpdateMs = nowMs;

  outSnapshot.key = key;
  outSnapshot.ownerDeviceId = ownerDeviceId;
  outSnapshot.revision = revision;
  outSnapshot.payload.assign(entry->value.data(), entry->value.data() + entry->valueLen);
  return true;
}

bool PropertyStore::has(uint16_t key, uint32_t ownerDeviceId) const
{
  return findEntry(key, ownerDeviceId) != nullptr;
}

bool PropertyStore::get(uint16_t key, uint32_t ownerDeviceId, std::vector<uint8_t> &out) const
{
  const PropertyEntry *entry = findEntry(key, ownerDeviceId);
  if (entry == nullptr)
  {
    return false;
  }
  out.assign(entry->value.data(), entry->value.data() + entry->valueLen);
  return true;
}

uint16_t PropertyStore::getRevision(uint16_t key, uint32_t ownerDeviceId) const
{
  const PropertyEntry *entry = findEntry(key, ownerDeviceId);
  return entry != nullptr ? entry->revision : 0;
}

std::vector<PropertySnapshot> PropertyStore::getAll() const
{
  std::vector<PropertySnapshot> out;
  out.reserve(entries_.size());
  for (const auto &entry : entries_)
  {
    PropertySnapshot snapshot{};
    snapshot.key = entry.key;
    snapshot.ownerDeviceId = entry.ownerDeviceId;
    snapshot.revision = entry.revision;
    snapshot.payload.assign(entry.value.data(), entry.value.data() + entry.valueLen);
    out.push_back(std::move(snapshot));
  }
  return out;
}

std::vector<PropertySnapshot> PropertyStore::getOwnedBy(uint32_t ownerDeviceId) const
{
  std::vector<PropertySnapshot> out;
  for (const auto &entry : entries_)
  {
    if (entry.ownerDeviceId != ownerDeviceId || entry.revision == 0)
    {
      continue;
    }
    PropertySnapshot snapshot{};
    snapshot.key = entry.key;
    snapshot.ownerDeviceId = entry.ownerDeviceId;
    snapshot.revision = entry.revision;
    snapshot.payload.assign(entry.value.data(), entry.value.data() + entry.valueLen);
    out.push_back(std::move(snapshot));
  }
  return out;
}

void PropertyStore::clearRemoteEntries(uint32_t localDeviceId)
{
  for (auto it = entries_.begin(); it != entries_.end();)
  {
    if (it->ownerDeviceId != localDeviceId)
    {
      it = entries_.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

void PropertyStore::removeKey(uint16_t key)
{
  for (auto it = entries_.begin(); it != entries_.end();)
  {
    if (it->key == key)
    {
      it = entries_.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

void PropertyStore::cleanupStale(uint32_t nowMs, uint32_t staleMs, uint32_t localDeviceId)
{
  for (auto it = entries_.begin(); it != entries_.end();)
  {
    const bool isRemote = (it->ownerDeviceId != localDeviceId);
    const bool isStale = (nowMs - it->lastUpdateMs) > staleMs;
    if (isRemote && isStale)
    {
      it = entries_.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

SyncManager *SyncManager::getInstance()
{
  static SyncManager inst;
  return &inst;
}

SyncManager::SyncManager()
{
  ourDeviceId = 0;
  currentGroup = {};
  localAddress_.clear();
}

SyncManager::~SyncManager() {}

void SyncManager::begin()
{
  started_ = true;
  if (transport_ == nullptr)
  {
    return; // Application must call setTransport() before begin()
  }

  if (!transport_->isReady())
  {
    transport_->begin();
  }

  transport_->setReceiveCallback(
      [this](const TransportAddress &source, const TransportPacket &packet)
      { handleTransportFrame(source, packet); });

  localAddress_ = transport_->localAddress();

  ourDeviceId = generateDeviceId();
  nextSequence_ = esp_random();
  if (nextSequence_ == 0)
  {
    nextSequence_ = 1;
  }

  loadPreferences();
}

void SyncManager::loop()
{
  if (transport_ != nullptr)
  {
    transport_->loop();
  }

  uint32_t now = millis();
  if (now - lastHeartbeat >= MeshConstants::kHeartbeatInterval)
  {
    sendHeartbeat();
    lastHeartbeat = now;
  }

  checkDiscoveryCleanup(now);
  checkGroupCleanup(now);
  checkMemberTimeout(now);
  cleanupRelayCache(now);
  cleanupProperties(now);

  if (currentGroup.groupId != 0)
  {
    if (currentGroup.isMaster)
    {
      if (now - lastGrpAnnounce >= MeshConstants::kGroupAnnounceInterval)
      {
        sendGroupAnnounce();
        lastGrpAnnounce = now;
      }
      if (now - lastGrpInfo >= MeshConstants::kGroupInfoInterval)
      {
        sendGroupInfo();
        lastGrpInfo = now;
      }
    }
    else
    {
      if (now - lastTimeSync >= MeshConstants::kTimeSyncInterval)
      {
        requestTimeSync();
        lastTimeSync = now;
      }
    }
    reannounceProperties(now);
  }
  else
  {
    // Check join mode when not in a group
    if (syncMode == SyncMode::JOIN)
    {
      checkJoinMode(now);
    }
    else if (syncMode == SyncMode::AUTO)
    {
      checkAutoMode(now);
    }
  }
}

const std::map<std::string, DiscoveredDevice> &
SyncManager::getDiscoveredDevices() const
{
  return discoveredDevices;
}

std::vector<GroupAdvert> SyncManager::getDiscoveredGroups() const
{
  std::vector<GroupAdvert> out;
  for (auto &kv : discoveredGroups)
  {
    out.push_back(kv.second);
  }
  return out;
}

const GroupInfo &SyncManager::getGroupInfo() const
{
  return currentGroup;
}

bool SyncManager::isInGroup() const
{
  return currentGroup.groupId != 0;
}

bool SyncManager::isGroupMaster() const
{
  return currentGroup.isMaster;
}

uint32_t SyncManager::getGroupId() const
{
  return currentGroup.groupId;
}

uint32_t SyncManager::getDeviceId() const
{
  return ourDeviceId;
}

void SyncManager::createGroup(uint32_t groupId)
{
  if (groupId == 0)
    groupId = generateGroupId();
  const TransportAddress ourAddress = getOurAddress();
  if (!ourAddress.isValid())
  {
    return;
  }
  currentGroup.groupId = groupId;
  currentGroup.masterDeviceId = ourDeviceId;
  currentGroup.isMaster = true;
  currentGroup.members.clear();

  // Masters are immediately time synced (they are the reference).
  offsetClock_.setMasterReference();
  currentGroup.timeSynced = true;
  currentGroup.timeOffset = 0;

  // add self
  std::string ms = addressToString(ourAddress);
  GroupMember gm;
  gm.deviceId = ourDeviceId;
  gm.address = ourAddress;
  currentGroup.members[ms] = gm;
  sendGroupAnnounce();
  sendGroupInfo();
  if (onGroupCreated)
    onGroupCreated(currentGroup);
}

void SyncManager::joinGroup(uint32_t groupId)
{
  if (currentGroup.groupId == groupId)
    return;
  // leave old
  leaveGroup();
  auto it = discoveredGroups.find(groupId);
  if (it == discoveredGroups.end())
    return;
  auto &adv = it->second;
  const TransportAddress ourAddress = getOurAddress();
  if (!ourAddress.isValid())
  {
    return;
  }
  currentGroup.groupId = groupId;
  currentGroup.masterDeviceId = adv.masterDeviceId;
  currentGroup.isMaster = false;
  currentGroup.members.clear();

  // Reset sync state when joining a group (slaves need to sync)
  offsetClock_.reset();
  currentGroup.timeSynced = offsetClock_.isLocked();
  currentGroup.timeOffset = offsetClock_.offsetMillis();

  // add self
  std::string ms = addressToString(ourAddress);
  GroupMember gm;
  gm.deviceId = ourDeviceId;
  gm.address = ourAddress;
  currentGroup.members[ms] = gm;
  // send join
  GroupJoinHeader joinHeader{};
  joinHeader.groupId = groupId;
  joinHeader.deviceId = ourDeviceId;
  std::vector<uint8_t> payload;
  appendStruct(payload, joinHeader);
  appendAddress(payload, ourAddress);
  TransportPacket pkt{};
  if (buildLocalMeshPacket(pkt,
                           MeshProtocol::MessageKind::Group,
                           static_cast<uint8_t>(MeshProtocol::GroupCommand::Join),
                           payload.data(),
                           static_cast<uint16_t>(payload.size())))
  {
    sendPacketChecked(pkt, transport_->broadcastAddress(), "group-join");
  }

  // Request immediate time sync after joining
  requestTimeSync();
  lastTimeSync = millis();

  if (onGroupJoined)
    onGroupJoined(currentGroup);

  // Immediately publish our owned properties so existing members see our latest state.
  reannounceProperties(millis(), true);
}

void SyncManager::leaveGroup()
{
  if (currentGroup.groupId == 0)
    return;

  // Prepare the leave/disband packet
  GroupLeaveCmd leaveCmd;
  leaveCmd.groupId = currentGroup.groupId;
  leaveCmd.deviceId = ourDeviceId;
  TransportPacket pkt{};
  if (!buildLocalMeshPacket(pkt,
                            MeshProtocol::MessageKind::Group,
                            static_cast<uint8_t>(MeshProtocol::GroupCommand::Leave),
                            reinterpret_cast<const uint8_t *>(&leaveCmd),
                            sizeof(leaveCmd)))
  {
    return;
  }

  for (uint8_t attempt = 0; attempt < 3; ++attempt)
  {
    sendPacketChecked(pkt, transport_->broadcastAddress(), "group-leave");
  }
  if (currentGroup.isMaster)
  {
    Serial.println("[GroupLeave] Master disbanding group, notifying all slaves");
  }
  else
  {
    Serial.println("[GroupLeave] Notified group of departure");
  }

  offsetClock_.reset();
  currentGroup = {};
  propertyStore_.clearRemoteEntries(ourDeviceId);

  if (onGroupLeft)
    onGroupLeft();
}

// Sync Mode Management
void SyncManager::setSyncMode(SyncMode mode)
{
  if (syncMode == mode)
    return;

  SyncMode oldMode = syncMode;
  syncMode = mode;

  Serial.println(String("[SyncMode] Changing from ") + getSyncModeString(oldMode) + " to " + getSyncModeString());

  switch (mode)
  {
  case SyncMode::SOLO:
    // Leave any current group and stop all group operations
    if (currentGroup.groupId != 0)
    {
      leaveGroup();
    }
    Serial.println("[SyncMode] SOLO mode - no group interaction");
    break;

  case SyncMode::JOIN:
    // If not in a group, start looking for groups to join
    if (currentGroup.groupId == 0)
    {
      Serial.println("[SyncMode] JOIN mode - looking for groups to join");
      // checkJoinMode will be called in the next loop iteration
    }
    else
    {
      Serial.println("[SyncMode] JOIN mode - already in a group - leaving");
      // check if we are the master
      if (currentGroup.isMaster)
      {
        Serial.println("[SyncMode] JOIN mode - we are the master - leaving");
        leaveGroup();
      }
      else
      {
        Serial.println("[SyncMode] JOIN mode - we are a slave - staying in group");
      }
    }
    break;

  case SyncMode::HOST:
    // Create a group immediately
    if (currentGroup.groupId == 0)
    {
      Serial.println("[SyncMode] HOST mode - creating group immediately");
      createGroup();
    }
    else if (!currentGroup.isMaster)
    {
      Serial.println("[SyncMode] HOST mode - leaving current group and creating new one");
      leaveGroup();
      createGroup();
    }
    else
    {
      Serial.println("[SyncMode] HOST mode - already hosting a group");
    }
    break;

  case SyncMode::AUTO:
    if (currentGroup.groupId == 0)
    {
      Serial.println("[SyncMode] AUTO mode - waiting for nearby auto peers");
    }
    else if (currentGroup.isMaster)
    {
      Serial.println("[SyncMode] AUTO mode - we are the master - leaving");
      leaveGroup();
    }
    else
    {
      Serial.println("[SyncMode] AUTO mode - we are a slave - staying in group");
    }
    break;
  }

  saveSyncModePreferences();
}

SyncMode SyncManager::getSyncMode() const
{
  return syncMode;
}

String SyncManager::getSyncModeString()
{
  return getSyncModeString(syncMode);
}

String SyncManager::getSyncModeString(SyncMode mode)
{
  switch (mode)
  {
  case SyncMode::SOLO:
    return "SOLO";
  case SyncMode::JOIN:
    return "JOIN";
  case SyncMode::HOST:
    return "HOST";
  case SyncMode::AUTO:
    return "AUTO";
  default:
    return "UNKNOWN";
  }
}

void SyncManager::requestTimeSync()
{
  if (currentGroup.groupId == 0 || currentGroup.isMaster)
    return;
  uint32_t reqTs = millis();
  lastTimeReq = reqTs;

  // Serial.println("[TimeSync] Requesting time sync from master");

  TimeRequestCmd requestCmd;
  requestCmd.requestTimestamp = reqTs;
  requestCmd.requesterDeviceId = ourDeviceId;
  requestCmd.groupId = currentGroup.groupId;
  TransportPacket pkt{};
  if (!buildLocalMeshPacket(pkt,
                            MeshProtocol::MessageKind::Time,
                            static_cast<uint8_t>(MeshProtocol::TimeCommand::Request),
                            reinterpret_cast<const uint8_t *>(&requestCmd),
                            sizeof(requestCmd)))
  {
    return;
  }
  sendPacketChecked(pkt, transport_->broadcastAddress(), "time-request");
}

bool SyncManager::isTimeSynced() const
{
  if (currentGroup.isMaster && currentGroup.groupId != 0)
    return true;

  return offsetClock_.isLocked();
}

uint32_t SyncManager::getSyncedTime() const
{
  return offsetClock_.nowSyncedMillis(millis());
}

int32_t SyncManager::getTimeOffset() const
{
  return offsetClock_.offsetMillis();
}

void SyncManager::setRelayEnabled(bool enabled)
{
  relayEnabled_ = enabled;
  if (!relayEnabled_)
  {
    relaySeenFrames_.clear();
  }
}

bool SyncManager::isRelayEnabled() const
{
  return relayEnabled_;
}

uint32_t SyncManager::syncMillis()
{
  SyncManager *syncMgr = SyncManager::getInstance();
  if (syncMgr->isTimeSynced())
  {
    return syncMgr->getSyncedTime();
  }
  return millis();
}

void SyncManager::setDeviceDiscoveredCallback(
    std::function<void(const DiscoveredDevice &)> cb) { onDeviceDiscovered = cb; }

void SyncManager::setGroupFoundCallback(
    std::function<void(const GroupAdvert &)> cb) { onGroupFound = cb; }

void SyncManager::setGroupCreatedCallback(
    std::function<void(const GroupInfo &)> cb) { onGroupCreated = cb; }

void SyncManager::setGroupJoinedCallback(
    std::function<void(const GroupInfo &)> cb) { onGroupJoined = cb; }

void SyncManager::setGroupLeftCallback(std::function<void()> cb)
{
  onGroupLeft = cb;
}

void SyncManager::setTimeSyncCallback(
    std::function<void(uint32_t)> cb) { onTimeSynced = cb; }

void SyncManager::printDeviceInfo()
{
  Serial.println(F("\n=== DISCOVERED DEVICES ==="));

  if (discoveredDevices.empty())
  {
    Serial.println(F("No devices discovered."));
  }
  else
  {
    Serial.println(String(F("Found ")) + String(discoveredDevices.size()) + F(" device(s):"));
    Serial.println();

    int index = 1;
    for (const auto &devicePair : discoveredDevices)
    {
      const DiscoveredDevice &device = devicePair.second;

      Serial.println(String(F("Device ")) + String(index) + F(":"));
      Serial.println(String(F("  Device ID: 0x")) + String(device.deviceId, HEX));

      Serial.println(String(F("  Address: ")) + String(addressToString(device.address).c_str()));

      uint32_t timeSinceLastSeen = millis() - device.lastSeen;
      Serial.println(String(F("  Last Seen: ")) + String(timeSinceLastSeen) + F("ms ago"));

      // Check if this device is in our current group
      if (currentGroup.groupId != 0)
      {
        auto memberIt = currentGroup.members.find(devicePair.first);
        if (memberIt != currentGroup.members.end())
        {
          Serial.println(F("  Status: In current group"));
        }
        else
        {
          Serial.println(F("  Status: Not in current group"));
        }
      }
      else
      {
        Serial.println(F("  Status: No group context"));
      }

      Serial.println();
      index++;
    }
  }

  Serial.println(F("==========================="));
}

void SyncManager::printGroupInfo()
{
  Serial.println(F("\n=== GROUP INFORMATION ==="));

  if (currentGroup.groupId == 0)
  {
    Serial.println(F("Not currently in a group."));
  }
  else
  {
    Serial.println(String(F("Group ID: 0x")) + String(currentGroup.groupId, HEX));
    Serial.println(String(F("Master Device: 0x")) + String(currentGroup.masterDeviceId, HEX));
    Serial.println(String(F("Role: ")) + (currentGroup.isMaster ? "MASTER" : "SLAVE"));
    Serial.println(String(F("Time Synced: ")) + (isTimeSynced() ? "YES" : "NO"));

    if (isTimeSynced())
    {
      Serial.println(String(F("Time Offset: ")) + String(offsetClock_.offsetMillis()) + F("ms"));
      Serial.println(String(F("Synced Time: ")) + String(getSyncedTime()));
    }

    Serial.println(String(F("Group Members: ")) + String(currentGroup.members.size()));

    if (!currentGroup.members.empty())
    {
      Serial.println(F("\nMember Details:"));
      int index = 1;

      for (const auto &memberPair : currentGroup.members)
      {
        const GroupMember &member = memberPair.second;

        Serial.println(String(F("  Member ")) + String(index) + F(":"));
        Serial.println(String(F("    Device ID: 0x")) + String(member.deviceId, HEX));

        Serial.println(String(F("    Address: ")) + String(addressToString(member.address).c_str()));

        // Check if this is the master
        if (member.deviceId == currentGroup.masterDeviceId)
        {
          Serial.println(F("    Role: MASTER"));
        }
        else
        {
          Serial.println(F("    Role: SLAVE"));
        }

        // Check if this is us
        if (member.deviceId == ourDeviceId)
        {
          Serial.println(F("    Status: This device"));
        }
        else
        {
          Serial.println(F("    Status: Remote device"));

          // Try to find additional info from discovered devices
          auto discoveredIt = discoveredDevices.find(memberPair.first);
          if (discoveredIt != discoveredDevices.end())
          {
            uint32_t timeSinceLastSeen = millis() - discoveredIt->second.lastSeen;
            Serial.println(String(F("    Last Heartbeat: ")) + String(timeSinceLastSeen) + F("ms ago"));
          }
          else
          {
            Serial.println(F("    Last Heartbeat: Unknown"));
          }
        }

        Serial.println();
        index++;
      }
    }
  }

  // Also show discovered groups
  Serial.println(F("\nDiscovered Groups:"));
  if (discoveredGroups.empty())
  {
    Serial.println(F("No other groups discovered."));
  }
  else
  {
    int index = 1;
    for (const auto &groupPair : discoveredGroups)
    {
      const GroupAdvert &group = groupPair.second;

      Serial.println(String(F("  Group ")) + String(index) + F(":"));
      Serial.println(String(F("    Group ID: 0x")) + String(group.groupId, HEX));
      Serial.println(String(F("    Master Device: 0x")) + String(group.masterDeviceId, HEX));

      Serial.println(String(F("    Master Address: ")) + String(addressToString(group.masterAddress).c_str()));

      uint32_t timeSinceLastSeen = millis() - group.lastSeen;
      Serial.println(String(F("    Last Announce: ")) + String(timeSinceLastSeen) + F("ms ago"));

      if (group.groupId == currentGroup.groupId)
      {
        Serial.println(F("    Status: Current group"));
      }
      else
      {
        Serial.println(F("    Status: Available to join"));
      }

      Serial.println();
      index++;
    }
  }

  Serial.println(F("========================="));
}

void SyncManager::handleSyncPacket(const TransportAddress &source, const TransportPacket &packet)
{
  MeshProtocol::DecodedFrame frame{};
  if (!MeshProtocol::decode(packet, frame))
  {
    return;
  }

  const uint32_t now = millis();
  if (!rememberRelayFrame(frame.originDeviceId, frame.sequence, now))
  {
    return;
  }
  if (shouldForwardRelayFrame(frame))
  {
    forwardRelayFrame(frame);
  }

  switch (frame.kind)
  {
  case MeshProtocol::MessageKind::Discovery:
    if (frame.command == static_cast<uint8_t>(MeshProtocol::DiscoveryCommand::Heartbeat))
    {
      processHeartbeat(frame, source);
    }
    break;
  case MeshProtocol::MessageKind::Group:
    if (frame.command == static_cast<uint8_t>(MeshProtocol::GroupCommand::Announce))
    {
      processGroupAnnounce(frame, source);
    }
    else if (frame.command == static_cast<uint8_t>(MeshProtocol::GroupCommand::Join))
    {
      processGroupJoin(frame, source);
    }
    else if (frame.command == static_cast<uint8_t>(MeshProtocol::GroupCommand::Info))
    {
      processGroupInfo(frame, source);
    }
    else if (frame.command == static_cast<uint8_t>(MeshProtocol::GroupCommand::Leave))
    {
      processGroupLeave(frame, source);
    }
    break;
  case MeshProtocol::MessageKind::Time:
    if (frame.command == static_cast<uint8_t>(MeshProtocol::TimeCommand::Request))
    {
      processTimeRequest(frame, source);
    }
    else if (frame.command == static_cast<uint8_t>(MeshProtocol::TimeCommand::Response))
    {
      processTimeResponse(frame, source);
    }
    break;
  case MeshProtocol::MessageKind::Property:
    if (frame.command == static_cast<uint8_t>(MeshProtocol::PropertyCommand::Update))
    {
      processPropertyUpdate(frame, source);
    }
    else if (frame.command == static_cast<uint8_t>(MeshProtocol::PropertyCommand::Batch))
    {
      processPropertyBatch(frame, source);
    }
    break;
  case MeshProtocol::MessageKind::Event:
    if (frame.command == static_cast<uint8_t>(MeshProtocol::EventCommand::Emit))
    {
      processEventEmit(frame, source);
    }
    break;
  default:
    break;
  }
}

void SyncManager::processHeartbeat(const MeshProtocol::DecodedFrame &frame, const TransportAddress &)
{
  HeartbeatCmd heartbeatCmd;
  if (!decodeHeartbeatCmd(frame, heartbeatCmd))
  {
    return;
  }
  if (heartbeatCmd.deviceId == ourDeviceId)
  {
    return;
  }

  std::string ms = addressToString(heartbeatCmd.address);
  bool isNew = (discoveredDevices.find(ms) == discoveredDevices.end());
  DiscoveredDevice d{};
  d.deviceId = heartbeatCmd.deviceId;
  d.address = heartbeatCmd.address;
  d.lastSeen = millis();
  d.syncMode = heartbeatCmd.syncMode;
  discoveredDevices[ms] = d;
  if (isNew && onDeviceDiscovered)
    onDeviceDiscovered(d);
}

void SyncManager::processGroupAnnounce(const MeshProtocol::DecodedFrame &frame, const TransportAddress &)
{
  GroupAnnounceCmd announceCmd;
  if (!decodeGroupAnnounceCmd(frame, announceCmd))
  {
    return;
  }
  if (announceCmd.masterDeviceId == ourDeviceId)
  {
    return;
  }

  bool isNew = (discoveredGroups.find(announceCmd.groupId) == discoveredGroups.end());
  GroupAdvert adv;
  adv.groupId = announceCmd.groupId;
  adv.masterDeviceId = announceCmd.masterDeviceId;
  adv.masterAddress = announceCmd.masterAddress;
  adv.lastSeen = millis();
  discoveredGroups[announceCmd.groupId] = adv;
  if (isNew && onGroupFound)
    onGroupFound(adv);
}

void SyncManager::processGroupJoin(const MeshProtocol::DecodedFrame &frame, const TransportAddress &)
{
  if (!currentGroup.isMaster)
    return;

  GroupJoinCmd joinCmd;
  if (!decodeGroupJoinCmd(frame, joinCmd))
  {
    return;
  }

  if (joinCmd.groupId != currentGroup.groupId)
    return;

  std::string ms = addressToString(joinCmd.deviceAddress);
  GroupMember gm{
      joinCmd.deviceId,
      joinCmd.deviceAddress,
  };
  currentGroup.members[ms] = gm;
  sendGroupInfo();
  reannounceProperties(millis(), true);
}

void SyncManager::processGroupInfo(const MeshProtocol::DecodedFrame &frame, const TransportAddress &)
{
  if (currentGroup.isMaster)
    return;

  GroupInfoCmd infoCmd;
  if (!decodeGroupInfoCmd(frame, infoCmd))
  {
    return;
  }

  if (infoCmd.groupId != currentGroup.groupId)
    return;

  std::vector<uint32_t> oldMemberIds;
  oldMemberIds.reserve(currentGroup.members.size());
  for (const auto &memberKv : currentGroup.members)
  {
    oldMemberIds.push_back(memberKv.second.deviceId);
  }
  std::sort(oldMemberIds.begin(), oldMemberIds.end());

  currentGroup.masterDeviceId = infoCmd.masterDeviceId;
  currentGroup.members.clear();

  for (const GroupInfoMember &member : infoCmd.members)
  {
    std::string ms = addressToString(member.address);

    GroupMember gm{
        member.deviceId,
        member.address,
    };
    currentGroup.members[ms] = gm;
  }
  const TransportAddress ourAddress = getOurAddress();
  const std::string selfAddressKey = addressToString(ourAddress);
  if (ourAddress.isValid() && currentGroup.members.find(selfAddressKey) == currentGroup.members.end())
  {
    GroupMember selfMember{};
    selfMember.deviceId = ourDeviceId;
    selfMember.address = ourAddress;
    currentGroup.members[selfAddressKey] = selfMember;
  }

  std::vector<uint32_t> newMemberIds;
  newMemberIds.reserve(currentGroup.members.size());
  for (const auto &memberKv : currentGroup.members)
  {
    newMemberIds.push_back(memberKv.second.deviceId);
  }
  std::sort(newMemberIds.begin(), newMemberIds.end());

  // Only force publish when membership actually changed.
  if (oldMemberIds != newMemberIds)
  {
    reannounceProperties(millis(), true);
  }
}

void SyncManager::processGroupLeave(const MeshProtocol::DecodedFrame &frame, const TransportAddress &)
{
  GroupLeaveCmd leaveCmd;
  if (!decodePayload(frame, leaveCmd))
  {
    return;
  }

  // Ignore if not for our current group
  if (leaveCmd.groupId != currentGroup.groupId)
    return;

  if (currentGroup.isMaster)
  {
    // We're the master: a slave is leaving
    for (auto it = currentGroup.members.begin(); it != currentGroup.members.end(); ++it)
    {
      if (it->second.deviceId == leaveCmd.deviceId)
      {
        Serial.println("[GroupLeave] Device 0x" +
                       String(leaveCmd.deviceId, HEX) +
                       " left the group");
        currentGroup.members.erase(it);
        // Broadcast updated group info to remaining members
        sendGroupInfo();
        reannounceProperties(millis(), true);
        break;
      }
    }
  }
  else
  {
    // We're a slave: check if master is disbanding
    if (leaveCmd.deviceId == currentGroup.masterDeviceId)
    {
      Serial.println("[GroupDisband] Master 0x" +
                     String(leaveCmd.deviceId, HEX) +
                     " disbanded the group. Leaving group...");
      offsetClock_.reset();
      uint32_t gid = currentGroup.groupId;
      currentGroup = {};
      propertyStore_.clearRemoteEntries(ourDeviceId);
      // Remove the now-dead group from discovery
      discoveredGroups.erase(gid);
      if (onGroupLeft)
        onGroupLeft();
    }
  }
}

void SyncManager::processTimeRequest(const MeshProtocol::DecodedFrame &frame, const TransportAddress &source)
{
  if (!currentGroup.isMaster)
    return;

  TimeRequestCmd requestCmd;
  if (!decodePayload(frame, requestCmd))
  {
    return;
  }
  if (requestCmd.requesterDeviceId == 0 || requestCmd.requesterDeviceId == ourDeviceId || requestCmd.groupId != currentGroup.groupId)
  {
    return;
  }
  if (!isGroupMember(requestCmd.requesterDeviceId) ||
      !sourceMatchesDevice(source, requestCmd.requesterDeviceId, isRelayedFrame(frame)))
  {
    return;
  }

  uint32_t now = millis();

  Serial.println("[TimeSync] Master processing time request, responding with time: " + String(now));

  TimeResponseCmd responseCmd;
  responseCmd.requestTimestamp = requestCmd.requestTimestamp;
  responseCmd.masterTimestamp = now;
  responseCmd.requesterDeviceId = requestCmd.requesterDeviceId;
  responseCmd.groupId = requestCmd.groupId;
  TransportPacket pkt{};
  if (buildLocalMeshPacket(pkt,
                           MeshProtocol::MessageKind::Time,
                           static_cast<uint8_t>(MeshProtocol::TimeCommand::Response),
                           reinterpret_cast<const uint8_t *>(&responseCmd),
                           sizeof(responseCmd)))
  {
    sendPacketChecked(pkt, transport_->broadcastAddress(), "time-response");
  }
}

void SyncManager::processTimeResponse(const MeshProtocol::DecodedFrame &frame, const TransportAddress &source)
{
  if (currentGroup.isMaster || currentGroup.groupId == 0)
    return;

  TimeResponseCmd responseCmd;
  if (!decodePayload(frame, responseCmd))
  {
    return;
  }
  if (responseCmd.requesterDeviceId != ourDeviceId || responseCmd.groupId != currentGroup.groupId)
  {
    return;
  }
  if (frame.originDeviceId != currentGroup.masterDeviceId)
  {
    return;
  }
  if (!isGroupMember(frame.originDeviceId) ||
      !sourceMatchesDevice(source, frame.originDeviceId, isRelayedFrame(frame)))
  {
    return;
  }

  uint32_t now = millis();
  const bool accepted = offsetClock_.handleSample(responseCmd.requestTimestamp, responseCmd.masterTimestamp, now);
  if (!accepted)
  {
    Serial.println("[TimeSync] Ignored sample with high RTT");
    return;
  }

  currentGroup.timeSynced = offsetClock_.isLocked();
  currentGroup.timeOffset = offsetClock_.offsetMillis();

  if (onTimeSynced)
    onTimeSynced(getSyncedTime());
}

void SyncManager::sendHeartbeat()
{
  const TransportAddress ourAddress = getOurAddress();
  if (!ourAddress.isValid())
  {
    return;
  }
  HeartbeatHeader heartbeatHeader{};
  heartbeatHeader.deviceId = ourDeviceId;
  heartbeatHeader.syncMode = static_cast<uint8_t>(syncMode);
  std::vector<uint8_t> payload;
  appendStruct(payload, heartbeatHeader);
  appendAddress(payload, ourAddress);
  TransportPacket pkt{};
  if (buildLocalMeshPacket(pkt,
                           MeshProtocol::MessageKind::Discovery,
                           static_cast<uint8_t>(MeshProtocol::DiscoveryCommand::Heartbeat),
                           payload.data(),
                           static_cast<uint16_t>(payload.size())))
  {
    sendPacketChecked(pkt, transport_->broadcastAddress(), "heartbeat");
  }
#ifdef DEBUG_ESP_NOW
  if (DEBUG_ESP_NOW >= 2)
  {
    Serial.println("[Heartbeat] Sent heartbeat");
  }
#endif
}

void SyncManager::sendGroupAnnounce()
{
  if (!currentGroup.isMaster)
    return;
  const TransportAddress ourAddress = getOurAddress();
  if (!ourAddress.isValid())
  {
    return;
  }
  GroupAnnounceHeader announceHeader{};
  announceHeader.groupId = currentGroup.groupId;
  announceHeader.masterDeviceId = currentGroup.masterDeviceId;
  std::vector<uint8_t> payload;
  appendStruct(payload, announceHeader);
  appendAddress(payload, ourAddress);
  TransportPacket pkt{};
  if (buildLocalMeshPacket(pkt,
                           MeshProtocol::MessageKind::Group,
                           static_cast<uint8_t>(MeshProtocol::GroupCommand::Announce),
                           payload.data(),
                           static_cast<uint16_t>(payload.size())))
  {
    sendPacketChecked(pkt, transport_->broadcastAddress(), "group-announce");
  }
}

void SyncManager::sendGroupInfo()
{
  if (!currentGroup.isMaster)
    return;
  GroupInfoCmd infoCmd;
  infoCmd.groupId = currentGroup.groupId;
  infoCmd.masterDeviceId = currentGroup.masterDeviceId;
  std::vector<uint8_t> payload;
  GroupInfoHeader infoHeader{};
  infoHeader.groupId = infoCmd.groupId;
  infoHeader.masterDeviceId = infoCmd.masterDeviceId;
  infoHeader.memberCount = static_cast<uint8_t>(currentGroup.members.size());
  appendStruct(payload, infoHeader);

  for (auto &kv : currentGroup.members)
  {
    GroupInfoMemberHeader memberHeader{};
    memberHeader.deviceId = kv.second.deviceId;
    appendStruct(payload, memberHeader);
    if (!appendAddress(payload, kv.second.address))
    {
      return;
    }
  }
  TransportPacket pkt{};
  if (buildLocalMeshPacket(pkt,
                           MeshProtocol::MessageKind::Group,
                           static_cast<uint8_t>(MeshProtocol::GroupCommand::Info),
                           payload.data(),
                           static_cast<uint16_t>(payload.size())))
  {
    sendPacketChecked(pkt, transport_->broadcastAddress(), "group-info");
  }
}

void SyncManager::checkDiscoveryCleanup(uint32_t now)
{
  std::vector<std::string> rm;
  for (auto &kv : discoveredDevices)
  {
    if (now - kv.second.lastSeen > MeshConstants::kDiscoveryTimeout)
    {
      rm.push_back(kv.first);
    }
  }
  for (auto &key : rm)
    discoveredDevices.erase(key);
}

void SyncManager::checkGroupCleanup(uint32_t now)
{
  std::vector<uint32_t> rm;
  for (auto &kv : discoveredGroups)
  {
    if (now - kv.second.lastSeen > MeshConstants::kGroupDiscoveryTimeout)
    {
      rm.push_back(kv.first);
    }
  }
  for (auto gid : rm)
  {
    discoveredGroups.erase(gid);
    if (!currentGroup.isMaster &&
        currentGroup.groupId == gid)
    {
      leaveGroup();
    }
  }
}

void SyncManager::checkMemberTimeout(uint32_t now)
{
  // Only check member timeouts if we're in a group
  if (currentGroup.groupId == 0)
    return;

  std::vector<std::string> membersToRemove;

  // Check each group member for timeout
  for (const auto &memberPair : currentGroup.members)
  {
    const std::string &macStr = memberPair.first;
    const GroupMember &member = memberPair.second;

    // Skip checking our own device
    if (member.deviceId == ourDeviceId)
      continue;

    // Check if this member is still in discovered devices and hasn't timed out
    auto discoveredIt = discoveredDevices.find(macStr);
    if (discoveredIt == discoveredDevices.end() ||
        (now - discoveredIt->second.lastSeen > MeshConstants::kGroupMemberTimeout))
    {
      // Member has timed out - mark for removal
      membersToRemove.push_back(macStr);
      Serial.println("[MemberTimeout] Removing timed out member: Device 0x" +
                     String(member.deviceId, HEX) + " (address: " + String(macStr.c_str()) + ")");
    }
  }

  // Remove timed out members
  bool membersWereRemoved = false;
  for (const std::string &macStr : membersToRemove)
  {
    currentGroup.members.erase(macStr);
    membersWereRemoved = true;
  }

  // If we're the master and members were removed, broadcast updated group info
  if (currentGroup.isMaster && membersWereRemoved)
  {
    Serial.println("[MemberTimeout] " + String(membersToRemove.size()) +
                   " member(s) removed. Current group size: " + String(currentGroup.members.size()));
    sendGroupInfo();
    reannounceProperties(millis(), true);
  }
}

void SyncManager::checkJoinMode(uint32_t)
{
  if (currentGroup.groupId != 0)
    return;

  // In JOIN mode, immediately join any discovered group
  if (!discoveredGroups.empty())
  {
    Serial.println("[JoinMode] Found group, joining immediately...");
    joinGroup(discoveredGroups.begin()->first);
  }
}

void SyncManager::checkAutoMode(uint32_t now)
{
  if (currentGroup.groupId != 0)
  {
    return;
  }

  uint32_t smallestAutoDeviceId = ourDeviceId;
  uint32_t autoDeviceCount = (syncMode == SyncMode::AUTO) ? 1u : 0u;
  for (const auto &entry : discoveredDevices)
  {
    const DiscoveredDevice &device = entry.second;
    if (device.syncMode != SyncMode::AUTO)
    {
      continue;
    }
    if (now - device.lastSeen > MeshConstants::kDiscoveryTimeout)
    {
      continue;
    }
    ++autoDeviceCount;
    if (device.deviceId < smallestAutoDeviceId)
    {
      smallestAutoDeviceId = device.deviceId;
    }
  }

  if (autoDeviceCount < 2)
  {
    return;
  }

  if (ourDeviceId == smallestAutoDeviceId)
  {
    if (currentGroup.groupId == 0)
    {
      Serial.println("[AutoMode] We are the elected master; creating AUTO group");
      const uint32_t autoGroupId = MeshConstants::kAutoGroupId | (esp_random() & 0xFFFFu);
      createGroup(autoGroupId);
    }
    return;
  }

  uint32_t discoveredAutoGroupId = 0;
  for (const auto &kv : discoveredGroups)
  {
    if ((kv.first & MeshConstants::kAutoGroupIdMask) != MeshConstants::kAutoGroupId)
    {
      continue;
    }
    if (now - kv.second.lastSeen > MeshConstants::kGroupDiscoveryTimeout)
    {
      continue;
    }
    discoveredAutoGroupId = kv.first;
    break;
  }
  if (discoveredAutoGroupId == 0)
  {
    return;
  }

  Serial.println("[AutoMode] AUTO group discovered; joining");
  joinGroup(discoveredAutoGroupId);
}

void SyncManager::cleanupRelayCache(uint32_t now)
{
  for (auto it = relaySeenFrames_.begin(); it != relaySeenFrames_.end();)
  {
    if (now - it->second > MeshConstants::kRelaySeenTtlMs)
    {
      it = relaySeenFrames_.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

uint32_t SyncManager::generateDeviceId()
{
  if (ourDeviceId != 0)
  {
    return ourDeviceId;
  }

  if (deviceIdProvider_)
  {
    const uint32_t providedId = deviceIdProvider_();
    if (providedId != 0)
    {
      return providedId;
    }
  }
  const uint32_t randomId = esp_random();
  return randomId == 0 ? 1u : randomId;
}

uint32_t SyncManager::generateGroupId()
{
  return random(1, UINT32_MAX);
}

std::string SyncManager::addressToString(const TransportAddress &address) const
{
  std::string s;
  for (uint8_t i = 0; i < address.length; ++i)
  {
    char buf[4];
    snprintf(buf, sizeof(buf), "%02X", address.bytes[i]);
    s += buf;
    if (i + 1 < address.length)
    {
      s += ":";
    }
  }
  return s;
}

TransportAddress SyncManager::getOurAddress() const
{
  if (localAddress_.isValid())
  {
    return localAddress_;
  }
  if (transport_ == nullptr)
  {
    return {};
  }
  return transport_->localAddress();
}

void SyncManager::loadPreferences()
{
  if (loadModeCb_)
  {
    const uint8_t savedMode = loadModeCb_();
    syncMode = static_cast<SyncMode>(savedMode);
  }
  else
  {
    syncMode = SyncMode::SOLO;
  }

  Serial.println("[SyncManager] Loaded preferences:");
  Serial.println(String("  Sync Mode: ") + getSyncModeString());
}

void SyncManager::saveSyncModePreferences()
{
  if (saveModeCb_)
  {
    saveModeCb_(static_cast<uint8_t>(syncMode));
  }
  Serial.println(String("[SyncManager] Sync mode preferences saved: ") + getSyncModeString());
}

void SyncManager::printSyncModeInfo()
{
  Serial.println(F("\n=== SYNC MODE INFO ==="));
  Serial.println(String("Current Mode: ") + getSyncModeString());

  switch (syncMode)
  {
  case SyncMode::SOLO:
    Serial.println(F("  - No group interaction"));
    Serial.println(F("  - Won't host or join groups"));
    break;
  case SyncMode::JOIN:
    Serial.println(F("  - Looking for groups to join"));
    Serial.println(String(F("  - Discovered groups: ")) + String(discoveredGroups.size()));
    break;
  case SyncMode::HOST:
    Serial.println(F("  - Hosting mode"));
    if (currentGroup.groupId != 0 && currentGroup.isMaster)
    {
      Serial.println(String(F("  - Currently hosting group: 0x")) + String(currentGroup.groupId, HEX));
      Serial.println(String(F("  - Group members: ")) + String(currentGroup.members.size()));
    }
    else
    {
      Serial.println(F("  - Ready to create group"));
    }
    break;
  case SyncMode::AUTO:
    Serial.println(F("  - Auto mode"));
    Serial.println(F("  - Forms AUTO group when 2+ AUTO peers are nearby"));
    Serial.println(F("  - Lowest deviceId becomes master"));
    break;
  }

  Serial.println(F("======================="));
}

void SyncManager::setTransport(ITransport *transport)
{
  transport_ = transport;
  localAddress_.clear();
}

ITransport *SyncManager::getTransport() const
{
  return transport_;
}

void SyncManager::setModePersistence(std::function<uint8_t()> loadCb,
                                     std::function<void(uint8_t)> saveCb)
{
  loadModeCb_ = std::move(loadCb);
  saveModeCb_ = std::move(saveCb);
}

void SyncManager::setDeviceIdProvider(std::function<uint32_t()> provider)
{
  deviceIdProvider_ = std::move(provider);
}

bool SyncManager::buildLocalMeshPacket(TransportPacket &pkt,
                                       MeshProtocol::MessageKind kind,
                                       uint8_t command,
                                       const uint8_t *payload,
                                       uint16_t payloadLen,
                                       uint8_t flags,
                                       uint8_t hopLimit)
{
  const uint32_t sequence = nextSequence_++;
  uint8_t effectiveFlags = flags;
  uint8_t effectiveHopLimit = hopLimit;
  if (!relayEnabled_)
  {
    effectiveFlags |= MeshConstants::kFlagNoRelay;
    effectiveHopLimit = 1;
  }
  if (!rememberRelayFrame(ourDeviceId, sequence, millis()))
  {
    return false;
  }

  return MeshProtocol::encode(pkt,
                              kind,
                              command,
                              effectiveFlags,
                              ourDeviceId,
                              sequence,
                              effectiveHopLimit,
                              payload,
                              payloadLen);
}

bool SyncManager::rememberRelayFrame(uint32_t originDeviceId, uint32_t sequence, uint32_t now)
{
  const uint64_t key = relayFrameKey(originDeviceId, sequence);
  if (relaySeenFrames_.find(key) != relaySeenFrames_.end())
  {
    return false;
  }

  if (relaySeenFrames_.size() >= MeshConstants::kRelaySeenMax)
  {
    cleanupRelayCache(now);
    if (relaySeenFrames_.size() >= MeshConstants::kRelaySeenMax)
    {
      auto oldest = relaySeenFrames_.begin();
      for (auto it = relaySeenFrames_.begin(); it != relaySeenFrames_.end(); ++it)
      {
        if (it->second < oldest->second)
        {
          oldest = it;
        }
      }
      relaySeenFrames_.erase(oldest);
    }
  }
  relaySeenFrames_[key] = now;
  return true;
}

bool SyncManager::shouldForwardRelayFrame(const MeshProtocol::DecodedFrame &frame) const
{
  if (!relayEnabled_)
  {
    return false;
  }
  if ((frame.flags & MeshConstants::kFlagNoRelay) != 0)
  {
    return false;
  }
  return frame.hopLimit > 1;
}

void SyncManager::forwardRelayFrame(const MeshProtocol::DecodedFrame &frame)
{
  TransportPacket relayPkt{};
  if (!MeshProtocol::encode(relayPkt,
                            frame.kind,
                            frame.command,
                            frame.flags,
                            frame.originDeviceId,
                            frame.sequence,
                            frame.hopLimit - 1,
                            frame.payload,
                            frame.payloadLen))
  {
    return;
  }

  sendPacketChecked(relayPkt, transport_->broadcastAddress(), "relay-forward");
}

uint64_t SyncManager::relayFrameKey(uint32_t originDeviceId, uint32_t sequence) const
{
  return (static_cast<uint64_t>(originDeviceId) << 32) | sequence;
}

RawPropertyHandle SyncManager::propertyRaw(uint16_t key, uint8_t maxLen)
{
  if (!registerProperty(key, maxLen))
  {
    return RawPropertyHandle();
  }
  return RawPropertyHandle(this, key, maxLen);
}

bool SyncManager::unregisterProperty(uint16_t key)
{
  auto it = propertyRegistry_.find(key);
  if (it == propertyRegistry_.end())
  {
    return false;
  }
  propertyRegistry_.erase(it);
  propertyStore_.removeKey(key);
  return true;
}

EventHandle<void> SyncManager::event(uint16_t channelId)
{
  if (!registerEvent(channelId, 0))
  {
    return EventHandle<void>();
  }
  return EventHandle<void>(this, channelId);
}

bool SyncManager::unregisterEvent(uint16_t channelId)
{
  auto it = eventRegistry_.find(channelId);
  if (it == eventRegistry_.end())
  {
    return false;
  }
  eventRegistry_.erase(it);
  return true;
}

std::vector<PropertySnapshot> SyncManager::getDiscoveredProperties() const
{
  return propertyStore_.getAll();
}

bool SyncManager::registerProperty(uint16_t key, uint8_t maxLen)
{
  if (maxLen > MeshConstants::kMaxPropertyValueSize)
  {
    return false;
  }
  auto it = propertyRegistry_.find(key);
  if (it == propertyRegistry_.end())
  {
    PropertyRegistration registration{};
    registration.maxLen = maxLen;
    propertyRegistry_[key] = std::move(registration);
    return true;
  }
  if (it->second.maxLen != maxLen)
  {
    return false;
  }
  return true;
}

bool SyncManager::registerEvent(uint16_t channelId, uint8_t maxLen)
{
  if (maxLen > MeshConstants::kMaxEventPayloadSize)
  {
    return false;
  }
  auto it = eventRegistry_.find(channelId);
  if (it == eventRegistry_.end())
  {
    EventRegistration registration{};
    registration.maxLen = maxLen;
    eventRegistry_[channelId] = std::move(registration);
    return true;
  }
  if (it->second.maxLen != maxLen)
  {
    return false;
  }
  return true;
}

bool SyncManager::setPropertyBytes(uint16_t key, const uint8_t *data, uint8_t len)
{
  if (currentGroup.groupId == 0 || len > MeshConstants::kMaxPropertyValueSize || (len > 0 && data == nullptr))
  {
    return false;
  }

  auto regIt = propertyRegistry_.find(key);
  if (regIt == propertyRegistry_.end() || len > regIt->second.maxLen)
  {
    return false;
  }

  PropertySnapshot snapshot{};
  if (!propertyStore_.setLocal(key, ourDeviceId, data, len, millis(), snapshot))
  {
    return false;
  }

  std::vector<uint8_t> payload;
  if (!MeshProtocol::appendPropertyEntry(payload, key, snapshot.revision, data, len))
  {
    return false;
  }

  TransportPacket pkt{};
  if (!buildLocalMeshPacket(pkt,
                            MeshProtocol::MessageKind::Property,
                            static_cast<uint8_t>(MeshProtocol::PropertyCommand::Update),
                            payload.data(),
                            static_cast<uint16_t>(payload.size())))
  {
    return false;
  }
  if (!sendPacketChecked(pkt, transport_->broadcastAddress(), "property-update"))
  {
    return false;
  }
  notifyPropertyChange(key, ourDeviceId, data, len);
  return true;
}

bool SyncManager::getPropertyBytesForOwner(uint16_t key, uint32_t ownerDeviceId, std::vector<uint8_t> &out) const
{
  return propertyStore_.get(key, ownerDeviceId, out);
}

bool SyncManager::hasPropertyValue(uint16_t key, uint32_t ownerDeviceId) const
{
  return propertyStore_.has(key, ownerDeviceId);
}

uint16_t SyncManager::getPropertyRevision(uint16_t key, uint32_t ownerDeviceId) const
{
  return propertyStore_.getRevision(key, ownerDeviceId);
}

void SyncManager::addPropertyChangeCallback(uint16_t key, std::function<void(uint32_t, const uint8_t *, uint8_t)> cb)
{
  if (!cb)
  {
    return;
  }
  auto it = propertyRegistry_.find(key);
  if (it == propertyRegistry_.end())
  {
    return;
  }
  it->second.callbacks.push_back(std::move(cb));
}

void SyncManager::clearPropertyChangeCallbacks(uint16_t key)
{
  auto it = propertyRegistry_.find(key);
  if (it == propertyRegistry_.end())
  {
    return;
  }
  it->second.callbacks.clear();
}

bool SyncManager::emitEventBytes(uint16_t channelId, const uint8_t *payload, uint8_t payloadLen)
{
  if (currentGroup.groupId == 0 || payloadLen > MeshConstants::kMaxEventPayloadSize || (payloadLen > 0 && payload == nullptr))
  {
    return false;
  }
  auto it = eventRegistry_.find(channelId);
  if (it == eventRegistry_.end() || payloadLen > it->second.maxLen)
  {
    return false;
  }

  std::vector<uint8_t> wirePayload;
  if (!MeshProtocol::buildEventPayload(wirePayload, channelId, payload, payloadLen))
  {
    return false;
  }

  TransportPacket pkt{};
  if (!buildLocalMeshPacket(pkt,
                            MeshProtocol::MessageKind::Event,
                            static_cast<uint8_t>(MeshProtocol::EventCommand::Emit),
                            wirePayload.data(),
                            static_cast<uint16_t>(wirePayload.size())))
  {
    return false;
  }
  if (!sendPacketChecked(pkt, transport_->broadcastAddress(), "event-emit"))
  {
    return false;
  }
  notifyEvent(channelId, ourDeviceId, payload, payloadLen);
  return true;
}

void SyncManager::addEventCallback(uint16_t channelId, std::function<void(uint32_t, const uint8_t *, uint8_t)> cb)
{
  if (!cb)
  {
    return;
  }
  auto it = eventRegistry_.find(channelId);
  if (it == eventRegistry_.end())
  {
    return;
  }
  it->second.callbacks.push_back(std::move(cb));
}

void SyncManager::clearEventCallbacks(uint16_t channelId)
{
  auto it = eventRegistry_.find(channelId);
  if (it == eventRegistry_.end())
  {
    return;
  }
  it->second.callbacks.clear();
}

void SyncManager::notifyPropertyChange(uint16_t key, uint32_t ownerDeviceId, const uint8_t *data, uint8_t len)
{
  auto it = propertyRegistry_.find(key);
  if (it == propertyRegistry_.end())
  {
    return;
  }
  for (auto &cb : it->second.callbacks)
  {
    cb(ownerDeviceId, data, len);
  }
}

void SyncManager::notifyEvent(uint16_t channelId, uint32_t fromDeviceId, const uint8_t *payload, uint8_t payloadLen)
{
  auto it = eventRegistry_.find(channelId);
  if (it == eventRegistry_.end())
  {
    return;
  }
  for (auto &cb : it->second.callbacks)
  {
    cb(fromDeviceId, payload, payloadLen);
  }
}

bool SyncManager::applyIncomingPropertyEntry(uint16_t key, uint16_t revision, uint32_t ownerDeviceId, const uint8_t *value, uint8_t valueLen)
{
  auto regIt = propertyRegistry_.find(key);
  if (regIt == propertyRegistry_.end() || valueLen > regIt->second.maxLen)
  {
    return false;
  }

  PropertySnapshot snapshot{};
  bool changed = false;
  if (!propertyStore_.applyRemote(key, ownerDeviceId, revision, value, valueLen, millis(), snapshot, changed))
  {
    return false;
  }
  if (changed)
  {
    notifyPropertyChange(key, ownerDeviceId, value, valueLen);
  }
  return true;
}

void SyncManager::processPropertyUpdate(const MeshProtocol::DecodedFrame &frame, const TransportAddress &source)
{
  if (currentGroup.groupId == 0 || !isGroupMember(frame.originDeviceId) ||
      !sourceMatchesDevice(source, frame.originDeviceId, isRelayedFrame(frame)))
  {
    return;
  }
  MeshProtocol::PropertyEntryHeader entry{};
  const uint8_t *value = nullptr;
  uint16_t next = 0;
  if (!MeshProtocol::readPropertyEntryAt(frame.payload, frame.payloadLen, 0, next, entry, value))
  {
    return;
  }
  if (next != frame.payloadLen)
  {
    return;
  }
  applyIncomingPropertyEntry(entry.key, entry.revision, frame.originDeviceId, value, entry.valueLen);
}

void SyncManager::processPropertyBatch(const MeshProtocol::DecodedFrame &frame, const TransportAddress &source)
{
  if (currentGroup.groupId == 0 || frame.payloadLen < 1 || !isGroupMember(frame.originDeviceId) ||
      !sourceMatchesDevice(source, frame.originDeviceId, isRelayedFrame(frame)))
  {
    return;
  }
  const uint8_t count = frame.payload[0];
  uint16_t offset = 1;
  for (uint8_t i = 0; i < count; ++i)
  {
    MeshProtocol::PropertyEntryHeader entry{};
    const uint8_t *value = nullptr;
    uint16_t next = 0;
    if (!MeshProtocol::readPropertyEntryAt(frame.payload, frame.payloadLen, offset, next, entry, value))
    {
      return;
    }
    applyIncomingPropertyEntry(entry.key, entry.revision, frame.originDeviceId, value, entry.valueLen);
    offset = next;
  }
}

void SyncManager::processEventEmit(const MeshProtocol::DecodedFrame &frame, const TransportAddress &source)
{
  if (currentGroup.groupId == 0 || !isGroupMember(frame.originDeviceId) ||
      !sourceMatchesDevice(source, frame.originDeviceId, isRelayedFrame(frame)))
  {
    return;
  }
  MeshProtocol::EventHeader header{};
  const uint8_t *payload = nullptr;
  if (!MeshProtocol::readEventPayload(frame.payload, frame.payloadLen, header, payload))
  {
    return;
  }
  auto regIt = eventRegistry_.find(header.channelId);
  if (regIt == eventRegistry_.end() || header.payloadLen > regIt->second.maxLen)
  {
    return;
  }
  notifyEvent(header.channelId, frame.originDeviceId, payload, header.payloadLen);
}

void SyncManager::cleanupProperties(uint32_t now)
{
  if (now - lastPropertyCleanup < MeshConstants::kPropertyCleanupIntervalMs)
  {
    return;
  }
  propertyStore_.cleanupStale(now, MeshConstants::kPropertyStaleMs, ourDeviceId);
  lastPropertyCleanup = now;
}

void SyncManager::reannounceProperties(uint32_t now, bool force)
{
  if (!force && (now - lastPropertyReannounce < MeshConstants::kPropertyReannounceIntervalMs))
  {
    return;
  }
  lastPropertyReannounce = now;

  const std::vector<PropertySnapshot> owned = propertyStore_.getOwnedBy(ourDeviceId);
  if (owned.empty())
  {
    return;
  }

  std::vector<uint8_t> payload;
  payload.push_back(0);
  uint8_t count = 0;

  auto flushBatch = [&]()
  {
    if (count == 0)
    {
      return;
    }
    payload[0] = count;
    TransportPacket pkt{};
    if (buildLocalMeshPacket(pkt,
                             MeshProtocol::MessageKind::Property,
                             static_cast<uint8_t>(MeshProtocol::PropertyCommand::Batch),
                             payload.data(),
                             static_cast<uint16_t>(payload.size())))
    {
      sendPacketChecked(pkt, transport_->broadcastAddress(), "property-batch");
    }
    payload.clear();
    payload.push_back(0);
    count = 0;
  };

  for (const auto &prop : owned)
  {
    const uint16_t entrySize = static_cast<uint16_t>(sizeof(MeshProtocol::PropertyEntryHeader) + prop.payload.size());
    if (entrySize + 1 > MeshConstants::kMaxMeshPayload)
    {
      continue;
    }
    if (payload.size() + entrySize > MeshConstants::kMaxMeshPayload)
    {
      flushBatch();
    }
    if (MeshProtocol::appendPropertyEntry(payload,
                                          prop.key,
                                          prop.revision,
                                          prop.payload.empty() ? nullptr : prop.payload.data(),
                                          static_cast<uint8_t>(prop.payload.size())))
    {
      ++count;
    }
  }
  flushBatch();
}

void RawPropertyHandle::set(const uint8_t *data, uint8_t len) const
{
  if (manager_ == nullptr)
  {
    return;
  }
  manager_->setPropertyBytes(key_, data, len);
}

uint8_t RawPropertyHandle::get(uint8_t *outBuf, uint8_t bufSize) const
{
  if (manager_ == nullptr)
  {
    return 0;
  }
  return get(manager_->getDeviceId(), outBuf, bufSize);
}

uint8_t RawPropertyHandle::get(uint32_t deviceId, uint8_t *outBuf, uint8_t bufSize) const
{
  if (manager_ == nullptr || outBuf == nullptr || bufSize == 0 || deviceId == 0)
  {
    return 0;
  }
  std::vector<uint8_t> bytes;
  if (!manager_->getPropertyBytesForOwner(key_, deviceId, bytes))
  {
    return 0;
  }
  const uint8_t toCopy = static_cast<uint8_t>(std::min<size_t>(bytes.size(), bufSize));
  if (toCopy > 0)
  {
    memcpy(outBuf, bytes.data(), toCopy);
  }
  return toCopy;
}

bool RawPropertyHandle::has(uint32_t deviceId) const
{
  return manager_ != nullptr && manager_->hasPropertyValue(key_, deviceId);
}

void RawPropertyHandle::onChange(std::function<void(uint32_t deviceId, const uint8_t *data, uint8_t len)> cb) const
{
  if (manager_ == nullptr || !cb)
  {
    return;
  }
  manager_->addPropertyChangeCallback(
      key_,
      [cb = std::move(cb)](uint32_t deviceId, const uint8_t *data, uint8_t len)
      { cb(deviceId, data, len); });
}

uint16_t RawPropertyHandle::revision() const
{
  if (manager_ == nullptr)
  {
    return 0;
  }
  return manager_->getPropertyRevision(key_, manager_->getDeviceId());
}

uint16_t RawPropertyHandle::revision(uint32_t deviceId) const
{
  if (manager_ == nullptr || deviceId == 0)
  {
    return 0;
  }
  return manager_->getPropertyRevision(key_, deviceId);
}

void EventHandle<void>::emit() const
{
  if (manager_ == nullptr)
  {
    return;
  }
  manager_->emitEventBytes(channelId_, nullptr, 0);
}

void EventHandle<void>::onEvent(std::function<void(uint32_t fromDevice)> cb) const
{
  if (manager_ == nullptr || !cb)
  {
    return;
  }
  manager_->addEventCallback(
      channelId_,
      [cb = std::move(cb)](uint32_t fromDevice, const uint8_t *, uint8_t)
      { cb(fromDevice); });
}

void SyncManager::handleTransportFrame(const TransportAddress &source, const TransportPacket &packet)
{
  if (packet.len > sizeof(packet.data))
  {
    return;
  }
  if (sizeof(MeshProtocol::FrameHeader) > packet.len)
  {
    return;
  }
  handleSyncPacket(source, packet);
}

bool SyncManager::getMasterAddress(TransportAddress &outAddress) const
{
  auto it = discoveredGroups.find(currentGroup.groupId);
  if (it == discoveredGroups.end())
  {
    return false;
  }
  outAddress = it->second.masterAddress;
  return true;
}

bool SyncManager::isGroupMember(uint32_t deviceId) const
{
  if (currentGroup.groupId == 0)
  {
    return false;
  }
  if (deviceId == ourDeviceId)
  {
    return true;
  }
  for (const auto &kv : currentGroup.members)
  {
    if (kv.second.deviceId == deviceId)
    {
      return true;
    }
  }
  return false;
}

bool SyncManager::isRelayedFrame(const MeshProtocol::DecodedFrame &frame) const
{
  if ((frame.flags & MeshConstants::kFlagNoRelay) != 0)
  {
    return false;
  }
  return frame.hopLimit < MeshConstants::kDefaultRelayHopLimit;
}

bool SyncManager::sourceMatchesDevice(const TransportAddress &source, uint32_t deviceId, bool isRelayed) const
{
  if (isRelayed)
  {
    // Relays preserve originDeviceId but replace the source address with the relay sender address.
    return true;
  }
  if (deviceId == ourDeviceId)
  {
    return source == getOurAddress();
  }
  for (const auto &kv : currentGroup.members)
  {
    if (kv.second.deviceId == deviceId)
    {
      return source == kv.second.address;
    }
  }
  return false;
}

bool SyncManager::sendPacketChecked(const TransportPacket &pkt, const TransportAddress &peer, const char *context) const
{
  if (transport_ == nullptr)
  {
    return false;
  }
  if (transport_->sendPacket(pkt, peer) == 0)
  {
    return true;
  }
#if DEBUG_ESP_NOW == 1
  Serial.println(String("[MeshSend] Failed: ") + String(context != nullptr ? context : "unknown"));
#endif
  return false;
}
// Host tests for wireless_mesh's Direct and Mesh layers. A fake transport records
// what the device sends and injects frames from a scripted master. Build with
// ASan/UBSan: several cases exist to catch lifetime bugs, not just wrong values.
#include <Direct.h>
#include <Mesh.h>

#include <cassert>
#include <vector>

uint32_t fakeMillis = 1000;

namespace
{
  const uint8_t kOurMac[6] = {0x02, 0, 0, 0, 0, 0x01};
  const uint8_t kMasterMac[6] = {0x02, 0, 0, 0, 0, 0x10};
  const uint8_t kOtherMac[6] = {0x02, 0, 0, 0, 0, 0x20};
  constexpr uint32_t kOurId = 0x01;
  constexpr uint32_t kMasterId = 0x10;
  constexpr uint32_t kOtherId = 0x20;
  constexpr uint32_t kGroup = 0x5000;

  struct Sent
  {
    TransportAddress peer;
    TransportPacket pkt;
  };

  class FakeTransport : public ITransport
  {
  public:
    std::vector<Sent> sent;
    ReceiveCallback rx;
    // When set, sendPacket hands the frame straight back, like a synchronous transport.
    bool loopback = false;

    bool begin() override { return true; }
    void end() override {}
    void loop() override {}
    bool isReady() const override { return true; }
    TransportAddress localAddress() const override { return TransportAddress::fromMac(kOurMac); }
    TransportAddress broadcastAddress() const override
    {
      static const uint8_t b[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
      return TransportAddress::fromMac(b);
    }
    int sendPacket(const TransportPacket &p, const TransportAddress &peer) override
    {
      sent.push_back({peer, p});
      if (loopback && rx)
        rx(localAddress(), p);
      return 0;
    }
    void setReceiveCallback(ReceiveCallback cb) override { rx = std::move(cb); }
  };

  FakeTransport transport;
  uint32_t seq = 100;

  template <typename T>
  void put(std::vector<uint8_t> &b, T v)
  {
    const uint8_t *p = reinterpret_cast<const uint8_t *>(&v);
    b.insert(b.end(), p, p + sizeof(T));
  }
  void putMac(std::vector<uint8_t> &b, const uint8_t *mac)
  {
    b.push_back(6);
    b.insert(b.end(), mac, mac + 6);
  }

  void inject(const uint8_t *fromMac, uint32_t origin, MeshProtocol::MessageKind kind, uint8_t cmd,
              const std::vector<uint8_t> &payload)
  {
    TransportPacket pkt;
    assert(MeshProtocol::encode(pkt, kind, cmd, MeshConstants::kFlagNoRelay, origin, ++seq, 1,
                                payload.data(), static_cast<uint16_t>(payload.size())));
    transport.rx(TransportAddress::fromMac(fromMac), pkt);
  }

  size_t countSent(MeshProtocol::MessageKind kind, uint8_t cmd)
  {
    size_t n = 0;
    for (const Sent &s : transport.sent)
    {
      MeshProtocol::DecodedFrame f{};
      if (MeshProtocol::decode(s.pkt, f) && f.kind == kind && f.command == cmd)
        ++n;
    }
    return n;
  }

  void announce(uint32_t group, uint32_t masterId, const uint8_t *mac)
  {
    std::vector<uint8_t> b;
    put(b, group);
    put(b, masterId);
    putMac(b, mac);
    inject(mac, masterId, MeshProtocol::MessageKind::Group, 1, b);
  }

  void heartbeat(uint32_t id, const uint8_t *mac, SyncMode mode)
  {
    std::vector<uint8_t> b;
    put(b, id);
    put(b, static_cast<uint8_t>(mode));
    putMac(b, mac);
    inject(mac, id, MeshProtocol::MessageKind::Discovery, 1, b);
  }

  // Roster from `origin`, listing the given members.
  void groupInfo(uint32_t origin, const uint8_t *fromMac, std::vector<std::pair<uint32_t, const uint8_t *>> members)
  {
    std::vector<uint8_t> b;
    put(b, kGroup);
    put(b, kMasterId);
    put(b, static_cast<uint8_t>(members.size()));
    for (auto &m : members)
    {
      put(b, m.first);
      putMac(b, m.second);
    }
    inject(fromMac, origin, MeshProtocol::MessageKind::Group, 3, b);
  }

  void advance(uint32_t ms, SyncManager *mesh)
  {
    for (uint32_t t = 0; t < ms; t += 100)
    {
      fakeMillis += 100;
      mesh->loop();
    }
  }
} // namespace

static void testDirect()
{
  DirectComms *direct = DirectComms::getInstance();
  direct->begin(&transport);
  const TransportAddress peer = TransportAddress::fromMac(kOtherMac);

  struct Req { uint8_t v; };
  struct Resp { uint8_t v; };
  auto rpc = direct->rpc<Req, Resp>(0x0101);

  // Reply crafted as if from `from`, for the last request sent.
  auto reply = [&](const uint8_t *from, uint8_t v)
  {
    const TransportPacket req = transport.sent.back().pkt;
    DirectFraming::Header h{};
    memcpy(&h, req.data, sizeof h);
    h.kind = static_cast<uint8_t>(DirectFraming::Kind::Response);
    h.payloadLen = 1;
    TransportPacket resp;
    resp.type = DirectFraming::kDirectPacketType;
    resp.len = sizeof h + 1;
    memcpy(resp.data, &h, sizeof h);
    resp.data[sizeof h] = v;
    transport.rx(TransportAddress::fromMac(from), resp);
  };

  int ok = 0, fail = 0, last = -1;
  auto cb = [&](bool success, const Resp &r)
  { success ? ++ok : ++fail; last = r.v; };

  // A reply from a peer we did not ask is ignored; the right peer completes it.
  rpc.request(peer, {1}, 200, cb);
  reply(kMasterMac, 9);
  assert(ok == 0 && fail == 0);
  reply(kOtherMac, 7);
  assert(ok == 1 && last == 7);

  // A reply drained after the deadline fails instead of succeeding late.
  rpc.request(peer, {1}, 200, cb);
  fakeMillis += 250;
  reply(kOtherMac, 7);
  assert(ok == 1 && fail == 1);

  // end() completes what is pending.
  rpc.request(peer, {1}, 200, cb);
  direct->end();
  assert(fail == 2);
  direct->loop();
  assert(fail == 2);

  // Synchronous transport: the reply arrives inside sendPacket() and still counts.
  rpc.onRequest([](const TransportAddress &, const Req &q, Resp &r)
                { r.v = static_cast<uint8_t>(q.v + 1); return true; });
  transport.loopback = true;
  rpc.request(transport.localAddress(), {41}, 200, cb);
  transport.loopback = false;
  assert(ok == 2 && last == 42);

  // A handler that replaces its own registration mid-call stays valid (ASan).
  auto ch = direct->channel<Req>(0x0102);
  int calls = 0;
  ch.onReceive([&, ch](const TransportAddress &, const Req &)
               {
                 ++calls;
                 ch.onReceive([&](const TransportAddress &, const Req &) { calls += 10; });
                 std::vector<int> scribble(64, calls); // reuse freed memory if the old handler died
                 (void)scribble; });
  ch.sendTo(peer, {1});
  transport.rx(peer, transport.sent.back().pkt);
  transport.rx(peer, transport.sent.back().pkt);
  assert(calls == 11);

  // An empty handler unregisters.
  ch.onReceive(nullptr);
  transport.rx(peer, transport.sent.back().pkt);
  assert(calls == 11);

  printf("direct: ok\n");
}

static void testOffsetClock()
{
  OffsetClock clock;
  // Master up ~35 days, us just booted: the smoothing used to overflow int32.
  assert(clock.handleSample(1000, 3000000000u, 1010));
  const int32_t first = clock.offsetMillis();
  assert(clock.handleSample(2000, 3000001000u, 2010));
  assert(clock.nowSyncedMillis(2010) - 3000001005u < 3u);
  (void)first;
  printf("offset clock: ok\n");
}

static void testMesh()
{
  SyncManager *mesh = SyncManager::getInstance();
  mesh->setTransport(&transport);
  mesh->setDeviceIdProvider([] { return kOurId; });
  mesh->setModePersistence([] { return static_cast<uint8_t>(SyncMode::JOIN); }, [](uint8_t) {});
  mesh->begin();
  // DirectComms owned the callback in testDirect; mesh->begin() took it over.

  heartbeat(kMasterId, kMasterMac, SyncMode::HOST);
  announce(kGroup, kMasterId, kMasterMac);
  mesh->loop(); // JOIN mode joins at once
  assert(mesh->isInGroup() && mesh->getGroupId() == kGroup);
  const size_t joins = countSent(MeshProtocol::MessageKind::Group, 2);
  assert(joins == 1);

  // The master's roster leaves us out (first join lost): we join again, rate limited.
  groupInfo(kMasterId, kMasterMac, {{kMasterId, kMasterMac}});
  assert(countSent(MeshProtocol::MessageKind::Group, 2) == joins); // within kJoinRetryMs
  fakeMillis += MeshConstants::kJoinRetryMs;
  groupInfo(kMasterId, kMasterMac, {{kMasterId, kMasterMac}});
  assert(countSent(MeshProtocol::MessageKind::Group, 2) == joins + 1);

  // A roster from anyone but our master is ignored.
  groupInfo(kOtherId, kOtherMac, {{kOtherId, kOtherMac}, {kOurId, kOurMac}});
  groupInfo(kMasterId, kMasterMac, {{kMasterId, kMasterMac}, {kOurId, kOurMac}});
  assert(mesh->getGroupInfo().members.size() == 2);
  groupInfo(kOtherId, kOtherMac, {{kOtherId, kOtherMac}, {kOurId, kOurMac}});
  for (auto &m : mesh->getGroupInfo().members)
    assert(m.second.deviceId != kOtherId);

  // A leave naming the master but sent by someone else does not disband us.
  {
    std::vector<uint8_t> b;
    put(b, kGroup);
    put(b, kMasterId);
    inject(kOtherMac, kOtherId, MeshProtocol::MessageKind::Group, 4, b);
    assert(mesh->isInGroup());
  }

  // Properties: an unchanged value re-announced every 5 s stays, and is reported once.
  auto prop = mesh->property<uint32_t>(0x7001);
  int changes = 0;
  prop.onChange([&](uint32_t, uint32_t) { ++changes; });
  auto batch = [&](std::vector<std::vector<uint8_t>> entries, int truncateLast = 0)
  {
    std::vector<uint8_t> b;
    b.push_back(static_cast<uint8_t>(entries.size()));
    for (auto &e : entries)
      b.insert(b.end(), e.begin(), e.end());
    b.resize(b.size() - truncateLast);
    inject(kMasterMac, kMasterId, MeshProtocol::MessageKind::Property, 2, b);
  };
  auto entry = [](uint16_t key, uint16_t rev, uint32_t value)
  {
    std::vector<uint8_t> e;
    put(e, key);
    put(e, rev);
    e.push_back(4);
    put(e, value);
    return e;
  };
  for (int i = 0; i < 10; ++i)
  {
    heartbeat(kMasterId, kMasterMac, SyncMode::HOST);
    announce(kGroup, kMasterId, kMasterMac);
    groupInfo(kMasterId, kMasterMac, {{kMasterId, kMasterMac}, {kOurId, kOurMac}});
    batch({entry(0x7001, 1, 55)});
    advance(5000, mesh);
  }
  assert(changes == 1 && prop.get(kMasterId) == 55);

  // A batch whose second entry is truncated applies nothing.
  batch({entry(0x7001, 2, 66), entry(0x7001, 3, 77)}, 2);
  assert(prop.get(kMasterId) == 55 && changes == 1);

  // Time sync: only a response to the outstanding request, once.
  mesh->requestTimeSync();
  const uint32_t reqTs = fakeMillis;
  auto timeResp = [&](uint32_t req, uint32_t masterTs)
  {
    std::vector<uint8_t> b;
    put(b, req);
    put(b, masterTs);
    put(b, kOurId);
    put(b, kGroup);
    inject(kMasterMac, kMasterId, MeshProtocol::MessageKind::Time, 2, b);
  };
  timeResp(reqTs - 1, 5000);
  assert(!mesh->isTimeSynced());
  fakeMillis += 10;
  timeResp(reqTs, 900000);
  assert(mesh->isTimeSynced());
  const int32_t offset = mesh->getTimeOffset();
  timeResp(reqTs, 1);
  assert(mesh->getTimeOffset() == offset);

  // Events: a listener that unregisters the channel mid-notify is safe (ASan).
  auto ev = mesh->event<uint8_t>(0x7100);
  int evCalls = 0;
  ev.onEvent([&](uint32_t, uint8_t) { ++evCalls; mesh->unregisterEvent(0x7100); });
  ev.onEvent([&](uint32_t, uint8_t) { evCalls += 100; });
  {
    std::vector<uint8_t> b;
    put(b, static_cast<uint16_t>(0x7100));
    b.push_back(1);
    b.push_back(9);
    inject(kMasterMac, kMasterId, MeshProtocol::MessageKind::Event, 1, b);
  }
  assert(evCalls == 1);

  // As master, the roster stops at what one frame can carry and keeps going out.
  mesh->leaveGroup();
  mesh->setSyncMode(SyncMode::HOST);
  assert(mesh->isGroupMaster());
  const uint32_t ourGroup = mesh->getGroupId();
  for (uint32_t i = 0; i < 30; ++i)
  {
    uint8_t mac[6] = {0x02, 1, 0, 0, 0, static_cast<uint8_t>(i)};
    std::vector<uint8_t> b;
    put(b, ourGroup);
    put(b, 0x1000 + i);
    putMac(b, mac);
    inject(mac, 0x1000 + i, MeshProtocol::MessageKind::Group, 2, b);
  }
  assert(mesh->getGroupInfo().members.size() == MeshConstants::kMaxGroupMembers);
  // Members joined without a heartbeat survive the sweep in the next loop.
  const size_t infosBefore = countSent(MeshProtocol::MessageKind::Group, 3);
  advance(2100, mesh);
  assert(mesh->getGroupInfo().members.size() == MeshConstants::kMaxGroupMembers);
  assert(countSent(MeshProtocol::MessageKind::Group, 3) > infosBefore);
  for (const Sent &s : transport.sent)
    assert(s.pkt.len <= TransportPacket::kMaxData);

  printf("mesh: ok\n");
}

int main()
{
  testDirect();
  testOffsetClock();
  testMesh();
  printf("wireless_mesh host tests passed\n");
}

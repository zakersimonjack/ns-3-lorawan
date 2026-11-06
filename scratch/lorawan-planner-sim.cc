/**
 * lorawan-planner-sim — generic ns-3 LoRaWAN simulation driven by a plan file
 * exported from ms-lora-network-planner.
 *
 * Channel = per-(gateway, end-device) path gains from Sionna RT, injected via
 * MatrixPropagationLossModel (no stochastic fading, no extra margins — the
 * analytic planner already accounts for those). What this adds on top of the
 * analytic link budget is MAC-level reality: duty cycle, collisions between
 * uplinks, capture effect, demodulator limits per gateway, SX1276
 * sensitivity/interference model.
 *
 * INPUT (line-based text; '#' comments; whitespace-separated fields):
 *   SIM <sim_time_s> <app_period_s> <payload_bytes> <start_stagger_s>
 *   SB <start_hz> <end_hz> <duty_frac> <max_tx_dbm>   regulatory sub-band
 *   CH <idx> <freq_hz>                                uplink channel list
 *   DR <sf> <dr> [<bw_hz>]                            full SF->DR map entry
 *                                                      (bw defaults 125 kHz)
 *   C5 <idx> <freq_hz>                                500 kHz channel list
 *                                                      (SF11/12 devices, US915)
 *   GWC <gw_idx> <freq_hz> [<freq_hz> ...]            this gateway's listen
 *                                                      set (default: all CH)
 *   SG <sigma_db>       per-packet log-normal shadowing sigma (dB, 0 = off).
 *                      ADR needs fading statistics — on a deterministic
 *                      channel it squeezes links to the sensitivity cliff
 *                      and oscillates; the signal layer provides the MEAN
 *                      (path gain + margins), this adds the variance.
 *   AD <0|1>           ADR: 1 = the network server re-tunes each device's
 *                      SF/power from real SNR (the S-line SF is only the
 *                      seed); 0 = static assignment (planner's choice)
 *   G <id> <x> <y> <z>
 *   S <id> <x> <y> <z> <sf> <tx_power_dbm>
 *   P <sensor_id> <gateway_id> <path_gain_db>      (path gain <= 0 dB;
 *                                                   -200 means unreachable)
 *
 * OUTPUT: JSON on stdout:
 *   { sim_time_s, app_period_s, payload_bytes,
 *     sent_total, received_total, pdr,
 *     sensors:[{id, address, sf, tx_power_dbm, sent, received, pdr}],
 *     gateways:[{id, received}] }
 *
 * Usage: ./ns3 run scratch_lorawan-planner-sim --input=/path/to/plan.txt
 * (run once per input; binary is reused via --no-build)
 */
#include "ns3/class-a-end-device-lorawan-mac.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/double.h"
#include "ns3/end-device-lorawan-mac.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/integer.h"
#include "ns3/logical-lora-channel.h"
#include "ns3/lora-channel.h"
#include "ns3/lora-device-address.h"
#include "ns3/lora-frame-header.h"
#include "ns3/lora-helper.h"
#include "ns3/lora-net-device.h"
#include "ns3/lora-phy.h"
#include "ns3/lorawan-mac-header.h"
#include "ns3/lorawan-mac-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/network-module.h"
#include "ns3/network-server-helper.h"
#include "ns3/forwarder-helper.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/pointer.h"
#include "ns3/point-to-point-module.h"
#include "ns3/position-allocator.h"
#include "ns3/propagation-module.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <array>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE("LorawanPlannerSim");

struct GwIn
{
    int id;
    double x, y, z;
};

struct EdIn
{
    int id;
    double x, y, z;
    int sf;
    double txPowerDbm;
};

struct PgIn
{
    int ed, gw;
    double pgDb;
};

// ---- per-device counters --------------------------------------------------
static std::vector<uint64_t> g_edSent;         // PHY StartSending per ED
static std::vector<uint64_t> g_edRecv;         // DISTINCT packets received per ED
// packet UIDs already counted per ED — an uplink heard by N gateways must
// count ONCE for the device (network-server dedup); without this the PDR
// scaled with gateway overlap (200-300% with 2-3 covering gateways)
static std::vector<std::set<uint64_t>> g_edRxUids;
static std::vector<uint64_t> g_gwRecv;         // PHY ReceivedPacket per GW
static std::map<uint32_t, int> g_addrToEd;     // NwkAddr -> ED vector index
static uint64_t g_recvEvents = 0;              // gateway reception events total
// per-reason drop counters (gateway PHY, for diagnosis)
static uint64_t g_dropUnderSens = 0, g_dropInterfered = 0, g_dropNoMoreRx = 0,
                g_dropTxing = 0, g_dropWrongFreq = 0, g_dropWrongSf = 0;

static void
CountDrop(Ptr<const Packet> p)
{
    (void)p;
}
static void
DropUnderSens(Ptr<const Packet> p, uint32_t s)
{
    (void)s;
    (void)p;
    g_dropUnderSens++;
}
static void
DropInterfered(Ptr<const Packet> p, uint32_t s)
{
    (void)s;
    (void)p;
    g_dropInterfered++;
}
static void
DropNoMoreRx(Ptr<const Packet> p, uint32_t s)
{
    (void)s;
    (void)p;
    g_dropNoMoreRx++;
}
static void
DropTxing(Ptr<const Packet> p, uint32_t s)
{
    (void)s;
    (void)p;
    g_dropTxing++;
}

static void
EdTxCallback(int edIdx, Ptr<const Packet> packet, unsigned iface)
{
    (void)packet;
    (void)iface;
    g_edSent[edIdx]++;
}

static void
GwRxCallback(int gwIdx, Ptr<const Packet> packet, unsigned iface)
{
    (void)iface;
    g_gwRecv[gwIdx]++;
    g_recvEvents++;
    // attribute the packet to its end device via the frame header address
    Ptr<Packet> copy = packet->Copy();
    LorawanMacHeader macHdr;
    copy->RemoveHeader(macHdr);
    LoraFrameHeader frHdr;
    frHdr.SetAsUplink();
    copy->RemoveHeader(frHdr);
    uint32_t addr = frHdr.GetAddress().GetNwkAddr();
    auto it = g_addrToEd.find(addr);
    if (it != g_addrToEd.end() && g_edRxUids[it->second].insert(packet->GetUid()).second)
        g_edRecv[it->second]++;
}

int
main(int argc, char* argv[])
{
    std::string inputPath = "planner-input.txt";
    CommandLine cmd(__FILE__);
    cmd.AddValue("input", "Path to the planner export file", inputPath);
    cmd.Parse(argc, argv);

    // ------------------------------------------------------------ parse input
    double simTimeS = 600, appPeriodS = 60, startStaggerS = 1.0;
    int payloadBytes = 23;
    std::vector<GwIn> gws;
    std::vector<EdIn> eds;
    std::vector<PgIn> pgs;
    // explicit channel plan (empty = module EU default, three channels)
    // sub-band: {start, {end, duty, maxTx}}
    std::vector<std::pair<double, std::array<double, 3>>> subBands;
    std::map<int, uint32_t> chanByIdx;
    std::vector<uint32_t> chan500;           // 500 kHz channels (SF11/12)
    std::map<int, int> sfToDr;               // sf -> dr
    std::map<int, uint32_t> drBw;            // dr -> bandwidth Hz
    std::map<int, std::vector<uint32_t>> gwListen;
    bool adrOn = false;
    double shadowSigmaDb = 0.0;
    std::set<int> wideEds;                   // sensor indices flagged "500k"

    std::ifstream in(inputPath);
    if (!in.is_open())
    {
        std::cerr << "cannot open input file: " << inputPath << std::endl;
        return 1;
    }
    std::string line;
    while (std::getline(in, line))
    {
        // strip comments
        auto hash = line.find('#');
        if (hash != std::string::npos)
            line = line.substr(0, hash);
        std::istringstream ss(line);
        std::string tag;
        if (!(ss >> tag) || tag.empty())
            continue;
        if (tag == "SIM")
            ss >> simTimeS >> appPeriodS >> payloadBytes >> startStaggerS;
        else if (tag == "SB")
        {
            double s0, s1, duty, maxTx;
            ss >> s0 >> s1 >> duty >> maxTx;
            subBands.push_back({s0, {s1, duty, maxTx}});
        }
        else if (tag == "CH")
        {
            int idx;
            uint32_t freq;
            ss >> idx >> freq;
            chanByIdx[idx] = freq;
        }
        else if (tag == "DR")
        {
            int sf, dr;
            ss >> sf >> dr;
            sfToDr[sf] = dr;
            if (ss.peek() == ' ')
            {
                uint32_t bw;
                ss >> bw;
                drBw[dr] = bw;
            }
        }
        else if (tag == "C5")
        {
            int idx;
            uint32_t freq;
            ss >> idx >> freq;
            if (static_cast<size_t>(idx) >= chan500.size())
                chan500.resize(idx + 1);
            chan500[idx] = freq;
        }
        else if (tag == "SG")
            ss >> shadowSigmaDb;
        else if (tag == "AD")
        {
            int v;
            ss >> v;
            adrOn = (v != 0);
        }
        else if (tag == "GWC")
        {
            int gi;
            ss >> gi;
            uint32_t f;
            auto& vec = gwListen[gi];
            while (ss >> f)
                vec.push_back(f);
        }
        else if (tag == "G")
        {
            GwIn g{};
            ss >> g.id >> g.x >> g.y >> g.z;
            gws.push_back(g);
        }
        else if (tag == "S")
        {
            EdIn e{};
            ss >> e.id >> e.x >> e.y >> e.z >> e.sf >> e.txPowerDbm;
            std::string flag;
            if (ss >> flag)
                if (flag == "500k")
                    wideEds.insert(e.id);
            eds.push_back(e);
        }
        else if (tag == "P")
        {
            PgIn p{};
            ss >> p.ed >> p.gw >> p.pgDb;
            pgs.push_back(p);
        }
    }
    if (gws.empty() || eds.empty())
    {
        std::cerr << "input needs at least one G and one S line" << std::endl;
        return 1;
    }
    std::cout << "# plan: " << eds.size() << " EDs, " << gws.size()
              << " GWs, " << pgs.size() << " links, sim " << simTimeS << "s"
              << std::endl;

    g_edSent.assign(eds.size(), 0);
    g_edRecv.assign(eds.size(), 0);
    g_edRxUids.assign(eds.size(), {});
    g_gwRecv.assign(gws.size(), 0);

    // ---------------------------------------------------------------- nodes
    NodeContainer endDevices;
    endDevices.Create(eds.size());
    NodeContainer gateways;
    gateways.Create(gws.size());

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    {
        Ptr<ListPositionAllocator> edAlloc = CreateObject<ListPositionAllocator>();
        for (const auto& e : eds)
            edAlloc->Add(Vector(e.x, e.y, e.z));
        mobility.SetPositionAllocator(edAlloc);
        mobility.Install(endDevices);
        Ptr<ListPositionAllocator> gwAlloc = CreateObject<ListPositionAllocator>();
        for (const auto& g : gws)
            gwAlloc->Add(Vector(g.x, g.y, g.z));
        mobility.SetPositionAllocator(gwAlloc);
        mobility.Install(gateways);
    }

    // --------------------------------------------- channel from path-gain matrix
    Ptr<MatrixPropagationLossModel> matrixLoss = CreateObject<MatrixPropagationLossModel>();
    Ptr<PropagationLossModel> loss = matrixLoss;
    if (shadowSigmaDb > 0)
    {
        // per-packet shadowing: normal jitter in dB around the signal
        // layer's deterministic mean (fixed RNG seed → reproducible)
        Ptr<RandomPropagationLossModel> shadow = CreateObject<RandomPropagationLossModel>();
        Ptr<NormalRandomVariable> rv = CreateObject<NormalRandomVariable>();
        rv->SetAttribute("Mean", DoubleValue(0.0));
        rv->SetAttribute("Variance", DoubleValue(shadowSigmaDb * shadowSigmaDb));
        shadow->SetAttribute("Variable", PointerValue(rv));
        matrixLoss->SetNext(shadow);
    }
    matrixLoss->SetDefaultLoss(200.0); // pairs without a P line = unreachable
    {
        auto edMob = std::vector<Ptr<MobilityModel>>();
        for (auto it = endDevices.Begin(); it != endDevices.End(); ++it)
            edMob.push_back((*it)->GetObject<MobilityModel>());
        int gwIdx = 0;
        for (auto it = gateways.Begin(); it != gateways.End(); ++it, ++gwIdx)
        {
            Ptr<MobilityModel> gwMob = (*it)->GetObject<MobilityModel>();
            for (const auto& p : pgs)
            {
                if (p.gw != gwIdx)
                    continue;
                // SetLoss takes a positive attenuation; path gain is <= 0 dB
                matrixLoss->SetLoss(gwMob, edMob[p.ed], std::max(0.0, -p.pgDb));
            }
        }
    }
    Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();
    Ptr<LoraChannel> channel = CreateObject<LoraChannel>(matrixLoss, delay);

    // ------------------------------------------------------------ end devices
    // EU region provides the DR tables / sub-band machinery; the explicit
    // plan below overrides channels, sub-bands and (if needed) the SF->DR map
    LorawanMacHelper macHelper;
    LoraHelper helper;
    helper.EnablePacketTracking();

    LoraPhyHelper phyHelper;
    phyHelper.SetChannel(channel);
    phyHelper.SetDeviceType(LoraPhyHelper::ED);
    macHelper.SetDeviceType(LorawanMacHelper::ED_A);

    uint8_t nwkId = 54;
    uint32_t nwkAddr = 1864;
    Ptr<LoraDeviceAddressGenerator> addrGen =
        CreateObject<LoraDeviceAddressGenerator>(nwkId, nwkAddr);
    macHelper.SetAddressGenerator(addrGen);

    NetDeviceContainer edNetDevices = helper.Install(phyHelper, macHelper, endDevices);

    // per-device SF / TX power / channel plan / traces / address book
    std::vector<uint32_t> edAddr(eds.size());
    for (uint32_t i = 0; i < endDevices.GetN(); ++i)
    {
        Ptr<LoraNetDevice> dev = DynamicCast<LoraNetDevice>(edNetDevices.Get(i));
        Ptr<EndDeviceLorawanMac> mac = DynamicCast<EndDeviceLorawanMac>(dev->GetMac());
        // DR comes from the exported SF→DR map — never assume the EU
        // arithmetic (12−SF): US915 maps SF9→DR1/SF10→DR0/SF12→DR8 etc.
        int dr = 12 - eds[i].sf; // EU fallback when the plan ships no map
        if (sfToDr.count(eds[i].sf))
            dr = sfToDr[eds[i].sf];
        // SetDataRate (method) not SetAttribute("DataRate") — the attribute's
        // checker caps DR at 5, which rejects US915's DR8/DR9 (500 kHz)
        mac->SetDataRate(static_cast<uint8_t>(dr));
        mac->SetTransmissionPowerDbm(eds[i].txPowerDbm);
        // ADR on: the network server owns SF/power (S-line SF is only the
        // seed); ADR off: static assignment from the planner's link budget
        mac->SetAttribute("ADR", BooleanValue(adrOn));
        if (!chanByIdx.empty())
        {
            // wide-band devices (US915 SF11/12) live on the C5 channel list
            const bool wide = wideEds.count(static_cast<int>(i)) && !chan500.empty();
            const auto& myChans = wide
                ? std::vector<uint32_t>(chan500.begin(), chan500.end())
                : [&]{ std::vector<uint32_t> v;
                       for (const auto& [idx, f] : chanByIdx) v.push_back(f);
                       return v; }();
            Ptr<LogicalLoraChannelHelper> ch = mac->GetLogicalLoraChannelHelper();
            ch->Resize(static_cast<uint8_t>(myChans.size()));
            uint8_t slot = 0;
            for (uint32_t f : myChans)
            {
                ch->SetChannel(slot, Create<LogicalLoraChannel>(f, 0, 9));
                slot++;
            }
            for (const auto& sb : subBands)
                ch->AddSubBand(Create<SubBand>(
                    static_cast<uint32_t>(sb.first),
                    static_cast<uint32_t>(sb.second[0]),
                    sb.second[1],
                    sb.second[2]));
        }
        if (!sfToDr.empty())
        {
            // rebuild SF / bandwidth / max-payload tables over the full DR range
            size_t maxDr = 5;
            for (const auto& [sf, dr] : sfToDr)
                maxDr = std::max(maxDr, static_cast<size_t>(dr));
            std::vector<uint8_t> sfForDr(maxDr + 1, 7);
            std::vector<uint32_t> bwForDr(maxDr + 1, 125000);
            std::vector<uint32_t> maxPayload(maxDr + 1, 230);
            const std::map<int, std::pair<uint8_t, uint32_t>> euDefault = {
                {12, {0, 59}}, {11, {1, 59}}, {10, {2, 59}},
                {9, {3, 123}}, {8, {4, 230}}, {7, {5, 230}},
            };
            for (const auto& [sf, dr] : sfToDr)
            {
                sfForDr[dr] = static_cast<uint8_t>(sf);
                maxPayload[dr] = euDefault.count(sf) ? euDefault.at(sf).second : 59;
                if (drBw.count(dr))
                    bwForDr[dr] = drBw.at(dr);
            }
            mac->SetSfForDataRate(sfForDr);
            mac->SetBandwidthForDataRate(bwForDr);
            mac->SetMaxMacPayloadForDataRate(maxPayload);
        }
        edAddr[i] = DynamicCast<EndDeviceLorawanMac>(mac)->GetDeviceAddress().GetNwkAddr();
        g_addrToEd[edAddr[i]] = i;
        dev->GetPhy()->TraceConnectWithoutContext(
            "StartSending", MakeBoundCallback(&EdTxCallback, static_cast<int>(i)));
    }

    // ---------------------------------------------------------------- gateways
    phyHelper.SetDeviceType(LoraPhyHelper::GW);
    macHelper.SetDeviceType(LorawanMacHelper::GW);
    helper.Install(phyHelper, macHelper, gateways);
    {
        int gwIdx = 0;
        for (auto it = gateways.Begin(); it != gateways.End(); ++it, ++gwIdx)
        {
            Ptr<LoraNetDevice> dev = DynamicCast<LoraNetDevice>((*it)->GetDevice(0));
            // the gateway MAC enforces downlink duty cycle through its own
            // channel helper — register our sub-bands there too, otherwise
            // any downlink-side lookup of a non-EU frequency is out-of-band
            if (!subBands.empty())
            {
                auto gwMac = DynamicCast<GatewayLorawanMac>(dev->GetMac());
                auto gwCh = gwMac->GetLogicalLoraChannelHelper();
                for (const auto& sb : subBands)
                    gwCh->AddSubBand(Create<SubBand>(
                        static_cast<uint32_t>(sb.first),
                        static_cast<uint32_t>(sb.second[0]),
                        sb.second[1],
                        sb.second[2]));
            }
            auto gwPhy = DynamicCast<GatewayLoraPhy>(dev->GetPhy());
            if (!chanByIdx.empty())
            {
                // listen set = GWC override (or the full plan) + the 500 kHz
                // channels when wide-band devices exist; the PHY caps at 8
                // frequencies (SX1301-class) — trim plan channels from the
                // tail to make room
                std::vector<uint32_t> listen;
                auto it2 = gwListen.find(gwIdx);
                if (it2 != gwListen.end())
                    listen = it2->second;
                else
                    for (const auto& [idx, f] : chanByIdx)
                        listen.push_back(f);
                if (!wideEds.empty())
                {
                    size_t nWide = chan500.size();
                    if (listen.size() + nWide > 8)
                        listen.resize(8 >= nWide ? 8 - nWide : 0);
                    for (uint32_t f : chan500)
                        listen.push_back(f);
                }
                if (listen.size() > 8)
                    listen.resize(8);
                gwPhy->ClearFrequencies();
                for (uint32_t f : listen)
                    gwPhy->AddFrequency(f);
            }
            dev->GetPhy()->TraceConnectWithoutContext(
                "ReceivedPacket", MakeBoundCallback(&GwRxCallback, gwIdx));
            dev->GetPhy()->TraceConnectWithoutContext("LostPacketBecauseUnderSensitivity", MakeCallback(&DropUnderSens));
            dev->GetPhy()->TraceConnectWithoutContext("LostPacketBecauseInterference", MakeCallback(&DropInterfered));
            dev->GetPhy()->TraceConnectWithoutContext("LostPacketBecauseNoMoreReceivers", MakeCallback(&DropNoMoreRx));
            dev->GetPhy()->TraceConnectWithoutContext("NoReceptionBecauseTransmitting", MakeCallback(&DropTxing));
        }
    }

    // ------------------------------------------------------- traffic (uplinks)
    // per-device install with RANDOM phase in [0, period): fixed intervals
    // with aligned or integer-staggered starts artificially remove the
    // collisions this simulation exists to measure. Fixed seed = reproducible.
    RngSeedManager::SetSeed(1);
    Ptr<UniformRandomVariable> startRv = CreateObjectWithAttributes<UniformRandomVariable>(
        "Min", DoubleValue(0.0), "Max", DoubleValue(appPeriodS));
    for (uint32_t i = 0; i < endDevices.GetN(); ++i)
    {
        PeriodicSenderHelper appHelper;
        appHelper.SetPeriod(Seconds(appPeriodS));
        appHelper.SetPacketSize(payloadBytes);
        ApplicationContainer app = appHelper.Install(endDevices.Get(i));
        app.Start(Seconds(startRv->GetValue()));
        app.Stop(Seconds(simTimeS));
    }

    // ---------------------------------------------------------- network server
    // Semtech-recommended ADR safety margin: without it the module consumes
    // the full SNR margin and parks links on the sensitivity cliff
    if (adrOn)
        Config::SetDefault("ns3::AdrComponent::MarginOffset", DoubleValue(15.0));
    NetworkServerHelper nsHelper;
    ForwarderHelper forHelper;
    Ptr<Node> networkServer = CreateObject<Node>();
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
    p2p.SetChannelAttribute("Delay", StringValue("2ms"));
    P2PGwRegistration_t gwRegistration;
    for (auto it = gateways.Begin(); it != gateways.End(); ++it)
    {
        auto container = p2p.Install(networkServer, *it);
        auto serverP2PNetDev = DynamicCast<PointToPointNetDevice>(container.Get(0));
        gwRegistration.emplace_back(serverP2PNetDev, *it);
    }
    nsHelper.SetGatewaysP2P(gwRegistration);
    nsHelper.SetEndDevices(endDevices);
    nsHelper.Install(networkServer);
    forHelper.Install(gateways);

    // ----------------------------------------------------------------- run it
    Simulator::Stop(Seconds(simTimeS + 5));
    Simulator::Run();

    // final SF per device: invert the SF->DR map on the DR each device
    // settled at (with ADR this is the network's choice, not our seed)
    std::vector<int> finalSf(eds.size(), -1);
    for (uint32_t i = 0; i < endDevices.GetN(); ++i)
    {
        Ptr<LoraNetDevice> dev = DynamicCast<LoraNetDevice>(edNetDevices.Get(i));
        uint8_t dr = DynamicCast<EndDeviceLorawanMac>(dev->GetMac())->GetDataRate();
        for (const auto& [sf, d] : sfToDr)
            if (d == dr)
                finalSf[i] = sf;
        if (finalSf[i] < 0)
            finalSf[i] = 12 - dr; // EU fallback
    }

    // -------------------------------------------------------------- JSON out
    uint64_t sent = 0, recv = 0;
    for (size_t i = 0; i < eds.size(); ++i)
    {
        sent += g_edSent[i];
        recv += g_edRecv[i];
    }
    std::ostringstream js;
    js << "{\n";
    js << "  \"sim_time_s\": " << simTimeS << ",\n";
    js << "  \"app_period_s\": " << appPeriodS << ",\n";
    js << "  \"payload_bytes\": " << payloadBytes << ",\n";
    js << "  \"sent_total\": " << sent << ",\n";
    js << "  \"received_total\": " << recv << ",\n";
    js << "  \"pdr\": " << (sent ? static_cast<double>(recv) / sent : 0.0) << ",\n";
    js << "  \"sensors\": [\n";
    for (size_t i = 0; i < eds.size(); ++i)
    {
        double pdr = g_edSent[i] ? static_cast<double>(g_edRecv[i]) / g_edSent[i] : 0.0;
        int sfOut = (finalSf[i] > 0) ? finalSf[i] : eds[i].sf;
        js << "    {\"id\": " << eds[i].id << ", \"address\": " << edAddr[i]
           << ", \"sf\": " << sfOut << ", \"tx_power_dbm\": " << eds[i].txPowerDbm
           << ", \"sent\": " << g_edSent[i] << ", \"received\": " << g_edRecv[i]
           << ", \"pdr\": " << pdr << "}" << (i + 1 < eds.size() ? "," : "") << "\n";
    }
    js << "  ],\n";
    js << "  \"gateways\": [\n";
    for (size_t i = 0; i < gws.size(); ++i)
    {
        js << "    {\"id\": " << gws[i].id << ", \"received\": " << g_gwRecv[i] << "}"
           << (i + 1 < gws.size() ? "," : "") << "\n";
    }
    js << "  ],\n";
    js << "  \"drops\": {\"under_sensitivity\": " << g_dropUnderSens
       << ", \"interfered\": " << g_dropInterfered
       << ", \"no_more_receivers\": " << g_dropNoMoreRx
       << ", \"gateway_tx\": " << g_dropTxing << "}\n";
    js << "}\n";
    std::cout << js.str();

    Simulator::Destroy();
    return 0;
}

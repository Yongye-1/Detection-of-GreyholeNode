#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/udp-echo-helper.h"
#include "ns3/wifi-module.h"
#include <map>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("GreyholeDetection");

enum NodeStatus {
    NO_STATUS,
    POSITIVE_STATUS,
    NEGATIVE_STATUS
};

class WatchdogNode;

class GreyholeNode : public Application {
public:
    GreyholeNode();
    virtual ~GreyholeNode();

    void Setup(Ptr<Node> node, double dropProbability);

private:
    virtual void StartApplication(void);
    virtual void StopApplication(void);

    void ReceivePacket(Ptr<Socket> socket);

    Ptr<Socket> m_socket;
    Ptr<Node> m_node;
    double m_dropProbability;
};

GreyholeNode::GreyholeNode()
    : m_socket(0),
      m_node(0),
      m_dropProbability(0.5) // 丢包率设置为50%
{
}

GreyholeNode::~GreyholeNode()
{
    m_socket = 0;
}

void GreyholeNode::Setup(Ptr<Node> node, double dropProbability)
{
    m_node = node;
    m_dropProbability = dropProbability;
}

void GreyholeNode::StartApplication(void)
{
    NS_LOG_UNCOND("Starting GreyholeNode application on node " << m_node->GetId());
    if (m_socket == 0)
    {
        m_socket = Socket::CreateSocket(m_node, TypeId::LookupByName("ns3::UdpSocketFactory"));
        m_socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), 9));
        m_socket->SetRecvCallback(MakeCallback(&GreyholeNode::ReceivePacket, this));
    }
}

void GreyholeNode::StopApplication(void)
{
    NS_LOG_UNCOND("Stopping GreyholeNode application on node " << m_node->GetId());
    if (m_socket != 0)
    {
        m_socket->Close();
        m_socket = 0;
    }
}

void GreyholeNode::ReceivePacket(Ptr<Socket> socket)
{
    Ptr<Packet> packet = socket->Recv();

    if (packet)
    {
        double randomValue = (double)rand() / (RAND_MAX);
        if (randomValue > m_dropProbability)
        {
            socket->Send(packet);
        }
        else
        {
            NS_LOG_UNCOND("Packet dropped by greyhole node: " << m_node->GetId());
        }
    }
}

class WatchdogNode : public Application {
public:
    WatchdogNode();
    virtual ~WatchdogNode();

    void Setup(Ptr<Node> node, double gamma);

private:
    virtual void StartApplication(void);
    virtual void StopApplication(void);

    void MonitorNode();
    void ProcessEvent(NodeStatus event);

    Ptr<Node> m_node;
    std::map<Ptr<Node>, NodeStatus> m_neighborStatus;
    double m_gamma;
    double m_reputation;
    double m_threshold;
    EventId m_event;
    uint32_t m_monitorCount;
    const uint32_t m_maxMonitorCount = 10;

    uint32_t m_receivedPackets;
    uint32_t m_sentPackets;
    double m_packetLossRate;
};

bool allNodesConverged = false;
std::vector<bool> nodesStatus;
double convergenceTime = 0.0;

WatchdogNode::WatchdogNode()
    : m_node(0),
      m_gamma(0.5),
      m_reputation(0),
      m_threshold(1.0),
      m_monitorCount(0),
      m_receivedPackets(0),
      m_sentPackets(0),
      m_packetLossRate(0.0)
{
}

WatchdogNode::~WatchdogNode()
{
}

void WatchdogNode::Setup(Ptr<Node> node, double gamma)
{
    m_node = node;
    m_gamma = gamma;
}

void WatchdogNode::StartApplication(void)
{
    NS_LOG_UNCOND("Starting WatchdogNode application on node " << m_node->GetId());
    m_event = Simulator::Schedule(Seconds(1.0), &WatchdogNode::MonitorNode, this);
}

void WatchdogNode::StopApplication(void)
{
    NS_LOG_UNCOND("Stopping WatchdogNode application on node " << m_node->GetId());
    Simulator::Cancel(m_event);
}

void WatchdogNode::MonitorNode()
{
    if (m_monitorCount >= m_maxMonitorCount || allNodesConverged)
    {
        NS_LOG_UNCOND("Watchdog node " << m_node->GetId() << " has reached max monitor count or all nodes have converged.");
        m_packetLossRate = 1.0 - ((double)m_receivedPackets / m_sentPackets);
        NS_LOG_UNCOND("Watchdog node " << m_node->GetId() << " packet loss rate: " << m_packetLossRate);
        return;
    }

    NS_LOG_UNCOND("Watchdog node " << m_node->GetId() << " monitoring neighbors.");

    NodeStatus event = NO_STATUS;
    double randomValue = (double)rand() / (RAND_MAX);

    if (randomValue < 0.33)
    {
        event = POSITIVE_STATUS;
    }
    else if (randomValue < 0.66)
    {
        event = NEGATIVE_STATUS;
    }

    ProcessEvent(event);

    m_monitorCount++;
    m_event = Simulator::Schedule(Seconds(1.0), &WatchdogNode::MonitorNode, this);
}

void WatchdogNode::ProcessEvent(NodeStatus event)
{
    switch (event)
    {
    case POSITIVE_STATUS:
        m_reputation += 1.0;
        NS_LOG_UNCOND("Watchdog node " << m_node->GetId() << " detected a positive event. Reputation: " << m_reputation);
        nodesStatus[m_node->GetId()] = true;
        break;
    case NEGATIVE_STATUS:
        m_reputation -= 1.0;
        NS_LOG_UNCOND("Watchdog node " << m_node->GetId() << " detected a negative event. Reputation: " << m_reputation);
        nodesStatus[m_node->GetId()] = true;
        break;
    case NO_STATUS:
    default:
        NS_LOG_UNCOND("Watchdog node " << m_node->GetId() << " has no sufficient information.");
        break;
    }

    if (m_reputation >= m_threshold)
    {
        NS_LOG_UNCOND("Node " << m_node->GetId() << " state: POSITIVE_STATUS");
    }
    else if (m_reputation < -m_threshold)
    {
        NS_LOG_UNCOND("Node " << m_node->GetId() << " state: NEGATIVE_STATUS");
    }
    else
    {
        NS_LOG_UNCOND("Node " << m_node->GetId() << " state: NO_STATUS");
    }

    bool allNodesHaveInfo = true;
    for (uint32_t i = 0; i < nodesStatus.size(); ++i)
    {
        if (!nodesStatus[i])
        {
            allNodesHaveInfo = false;
            break;
        }
    }

    if (allNodesHaveInfo && !allNodesConverged)
    {
        allNodesConverged = true;
        convergenceTime = Simulator::Now().GetSeconds();
        NS_LOG_UNCOND("All nodes have converged at time: " << convergenceTime << " seconds");
    }
}

// 全局变量用于统计数据包数量
uint32_t g_totalPacketsSent = 0;
uint32_t g_totalPacketsReceived = 0;

void PacketSentCallback(Ptr<const Packet> packet) {
    g_totalPacketsSent++;
}

void PacketReceivedCallback(Ptr<const Packet> packet, const Address &addr) {
    g_totalPacketsReceived++;
}

int main(int argc, char *argv[])
{
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);

    CommandLine cmd;
    cmd.Parse(argc, argv);

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy = YansWifiPhyHelper::Default();
    phy.SetChannel(channel.Create());

    WifiHelper wifi;
    wifi.SetRemoteStationManager("ns3::AarfWifiManager");

    NqosWifiMacHelper mac = NqosWifiMacHelper::Default();
    mac.SetType("ns3::AdhocWifiMac");

    NodeContainer nodes;
    nodes.Create(27); // 24 normal nodes + 1 greyhole node + 1 source node + 1 sink node

    NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

       MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "MinX", DoubleValue(0.0),
                                  "MinY", DoubleValue(0.0),
                                  "DeltaX", DoubleValue(5.0),
                                  "DeltaY", DoubleValue(5.0),
                                  "GridWidth", UintegerValue(7),
                                  "LayoutType", StringValue("RowFirst"));
    
    // 设置移动模型
    mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                              "Bounds", RectangleValue(Rectangle(0, 105, 0, 105)));
    mobility.Install(nodes);

    InternetStackHelper stack;
    stack.Install(nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    // 配置灰洞节点
    Ptr<GreyholeNode> greyholeNodeApp = CreateObject<GreyholeNode>();
    greyholeNodeApp->Setup(nodes.Get(25), 0.05); // 灰洞节点丢包率设置为5%
    nodes.Get(25)->AddApplication(greyholeNodeApp);
    greyholeNodeApp->SetStartTime(Seconds(1.0));
    greyholeNodeApp->SetStopTime(Seconds(30.0));

    nodesStatus.resize(nodes.GetN(), false);

    // 配置看门狗节点
    for (uint32_t i = 0; i < 24; ++i)
    {
        Ptr<WatchdogNode> watchdogNodeApp = CreateObject<WatchdogNode>();
        watchdogNodeApp->Setup(nodes.Get(i), 0.5);
        nodes.Get(i)->AddApplication(watchdogNodeApp);
        watchdogNodeApp->SetStartTime(Seconds(1.0));
        watchdogNodeApp->SetStopTime(Seconds(30.0));
    }

    // 配置UDP Echo服务器（目的端）
    UdpEchoServerHelper echoServer(9);
    ApplicationContainer serverApps = echoServer.Install(nodes.Get(26)); // 目的端在节点集合的最后一个位置
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(30.0));

   // 配置UDP Echo客户端（源端）
UdpEchoClientHelper echoClient(interfaces.GetAddress(0), 9);
echoClient.SetAttribute("MaxPackets", UintegerValue(1000));
echoClient.SetAttribute("Interval", TimeValue(Seconds(0.01)));
echoClient.SetAttribute("PacketSize", UintegerValue(1024)); // 设置数据包大小为1024字节


    ApplicationContainer clientApps = echoClient.Install(nodes.Get(24)); // 源端在节点集合的倒数第二个位置
    clientApps.Start(Seconds(2.0));
    clientApps.Stop(Seconds(30.0));

    // 设置回调函数，统计发送和接收的数据包数量
    Config::ConnectWithoutContext("/NodeList/*/ApplicationList/*/$ns3::UdpClient/Sent",
                                  MakeCallback(&PacketSentCallback));
    Config::ConnectWithoutContext("/NodeList/26/ApplicationList/*/$ns3::UdpServer/Rx",
                                  MakeCallback(&PacketReceivedCallback));

    Simulator::Stop(Seconds(30.0));
    AnimationInterface anim("first.xml");
    Simulator::Run();
    Simulator::Destroy();

    NS_LOG_UNCOND("Simulation finished. Convergence time: " << convergenceTime << " seconds");
    NS_LOG_UNCOND("Total packets sent from source node: " << g_totalPacketsSent);
    NS_LOG_UNCOND("Total packets received by sink node: " << g_totalPacketsReceived);
    NS_LOG_UNCOND("Packet loss rate: " << (1.0 - (double)g_totalPacketsReceived / g_totalPacketsSent));

    return 0;
}


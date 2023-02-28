/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 SEBASTIEN DERONNE
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Sebastien Deronne <sebastien.deronne@gmail.com>
 */

#include <ctime>
#include <sstream>
#include "ns3/command-line.h"
#include "ns3/pointer.h"
#include "ns3/config.h"
#include "ns3/uinteger.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/wifi-acknowledgment.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/wifi-net-device.h"
#include "ns3/qos-txop.h"
#include "ns3/wifi-mac.h"
#include "ns3/stats-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/stats-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/TimestampTag.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/idApp.h"
#include "ns3/traffic-control-layer.h"
#include "ns3/traffic-control-helper.h"

using namespace ns3;
using namespace std;

int nRxMpdu = 0;
double totalRxMpduBytes = 0;
double lastTotalRx = 0;
int n = 0;
double throughputBytes = 0;
double maxThroughput = 0;
static Time macDelay{Seconds(0)};
//static Time IpL4Delay{Seconds(0)};
static double nmac = 0;

struct singleStaDelayAndThroughput{
  int nPkt;
  Time singleStaDelay;
  //Time singleStaIpL4Delay;
  double singleStaThrouput;
} stat[64];

uint8_t ipAddrBuffer[4];
uint8_t macAddrBuffer[6];
uint32_t nSta;
//./ns3 run 'scratch/scenario5.cc --useExtendedBlockAck=true --simulationTime=1' 2>scenaria5-sumimo.log
//./ns3 run 'scratch/scenario5.cc --useExtendedBlockAck=true --dlAckType=AGGR-MU-BAR --simulationTime=1' 2>scenaria5-ofdma.log

NS_LOG_COMPONENT_DEFINE("scenario5");
///////////////////////////////////////////////////////////////////////////////////////////////////
/******************App trace**********************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////
/******************UDP trace**********************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////
/******************IpL4 trace**********************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////
/*void ApMacTxTrace(std::string context,Ptr<const Packet> packet) //traceAP端发送包
{
  if (packet->GetSize() > 1000)
  {
    // NS_LOG_UNCOND("ApMactx: " << packet->GetSize());
    TimestampTag timestamp;                         //timestap used to calculate the latency of each packet
    timestamp.SetTimestamp(Simulator::Now());
    packet->AddByteTag(timestamp);
  }
}

void StaMacRxTrace(std::string context,Ptr<const Packet> packet) //STA端接收包
{
  if (packet->GetSize() > 1000)
  {
    TimestampTag timestamp;
    packet->FindFirstMatchingByteTag(timestamp);
    Time txtime = timestamp.GetTimestamp();
    //NS_LOG_UNCOND("txTime: " << txtime.As(Time::MS) << " rxTime: " << Simulator::Now().As(Time::MS) << " delay: " << (Simulator::Now() - txtime).As(Time::MS));
    IpL4Delay += (Simulator::Now() - txtime);        //计算时延
    nmac += 1;

    LlcSnapHeader llc;
    Ipv4Header ipHdr;
    Ptr<Packet> copy = packet->Copy();
    copy->RemoveHeader(llc);
    copy->PeekHeader(ipHdr);
    ipHdr.GetDestination().Serialize(ipAddrBuffer);     //获取IP地址最后一位，用来区分STA
    //std::cout<<int(ipAddrBuffer[3])<<" delay: "<<(Simulator::Now() - txtime).As(Time::MS)<<std::endl;
    stat[int(ipAddrBuffer[3])-2].singleStaDelay += (Simulator::Now() - txtime);         //统计每个STA的总时延
    stat[int(ipAddrBuffer[3])-2].nPkt += 1;   //统计每个STA接收包的总数
  }
}
 */

///////////////////////////////////////////////////////////////////////////////////////////////////
/******************MAC trace**********************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////
void traceMacRxMpdu(std::string context, Ptr<const WifiMpdu> q)   //trace接收MPDU，计算吞吐
{
  // NS_LOG_UNCOND("At time " << Simulator::Now().GetSeconds() << " a " << q->GetSize() << " bytes mpdu was acked at Ap , packetSize = " << q->GetPacketSize());
  Ptr<const Packet> packet = q->GetPacket();
  const WifiMacHeader *macHdr = &q->GetHeader();
  if (macHdr->IsQosData() || macHdr->IsData())
  {
    nRxMpdu += 1;
    totalRxMpduBytes += q->GetSize();

    macHdr->GetAddr2().CopyTo(macAddrBuffer);
    nSta = int(macAddrBuffer[5]);   //获取MAC地址的最后一位，用来区分STA
    if (nSta == 1) // 发送方是AP，此时要用接收地址
    {
      macHdr->GetAddr1().CopyTo(macAddrBuffer);
      nSta = int(macAddrBuffer[5]);
      stat[nSta-2].singleStaThrouput += q->GetPacketSize();
    }
    else
    {
      stat[nSta-2].singleStaThrouput += q->GetPacketSize();
    }
  }
}

void ApMacTxTrace(std::string context,Ptr<const Packet> packet) //traceAP端发送包
{
  if (packet->GetSize() > 1000)
  {
    // NS_LOG_UNCOND("ApMactx: " << packet->GetSize());
    TimestampTag timestamp;                         //timestap used to calculate the latency of each packet
    timestamp.SetTimestamp(Simulator::Now());
    packet->AddByteTag(timestamp);
  }
}

void StaMacRxTrace(std::string context,Ptr<const Packet> packet) //STA端接收包
{
  if (packet->GetSize() > 1000)
  {
    TimestampTag timestamp;
    packet->FindFirstMatchingByteTag(timestamp);
    Time txtime = timestamp.GetTimestamp();
    //NS_LOG_UNCOND("txTime: " << txtime.As(Time::MS) << " rxTime: " << Simulator::Now().As(Time::MS) << " delay: " << (Simulator::Now() - txtime).As(Time::MS));
    macDelay += (Simulator::Now() - txtime);        //计算时延
    nmac += 1;

    LlcSnapHeader llc;
    Ipv4Header ipHdr;
    Ptr<Packet> copy = packet->Copy();
    copy->RemoveHeader(llc);
    copy->PeekHeader(ipHdr);
    ipHdr.GetDestination().Serialize(ipAddrBuffer);     //获取IP地址最后一位，用来区分STA
    //std::cout<<int(ipAddrBuffer[3])<<" delay: "<<(Simulator::Now() - txtime).As(Time::MS)<<std::endl;
    stat[int(ipAddrBuffer[3])-2].singleStaDelay += (Simulator::Now() - txtime);         //统计每个STA的总时延
    stat[int(ipAddrBuffer[3])-2].nPkt += 1;   //统计每个STA接收包的总数
  }
}

void traceBackOff(std::string context, const uint32_t value)  //MAC退避值
{
  //NS_LOG_UNCOND(context<<" backOff: "<<value);
}

void traceMacQueueDrop(Ptr<const WifiMpdu> item)    //数据包在队列中被丢弃
{
 // std::cout << "at time: " << Simulator::Now().GetSeconds() << " Mac Dropped a pakcet, size = " << item->GetPacketSize() << " SeqN: " << item->GetHeader().GetSequenceNumber() << " from: " << item->GetHeader().GetAddr2() << " to: " << item->GetHeader().GetAddr1() << " type: " << item->GetHeader().GetTypeString() << std::endl;
}

void traceMacQueueExpired(Ptr<const WifiMpdu> item) //数据包在队列中由于超时被丢弃
{
 // std::cout << "at time: " << Simulator::Now().GetSeconds() << " Mac a pakcet expired, size = " << item->GetPacketSize() << " SeqN: " << item->GetHeader().GetSequenceNumber() << " from: " << item->GetHeader().GetAddr2() << " to: " << item->GetHeader().GetAddr1() << " type: " << item->GetHeader().GetTypeString() << std::endl;
}

void traceMacTxDrop(Ptr<const Packet> packet)               //MAC层丢包在进入队列之前被丢弃
{
 // std::cout << "at time: " << Simulator::Now().GetSeconds() << " a packet dropped before queueing, size = "<<packet->GetSize()<<std::endl;
}

void traceMacRxDrop(Ptr<const Packet> packet)               //数据包通过物理层后被MAC层丢弃
{
 // std::cout << "at time: " << Simulator::Now().GetSeconds() << " a packet dropped after phy layer, size = "<<packet->GetSize()<<std::endl;
  //NS_LOG_UNCOND("at time: " << Simulator::Now().GetSeconds() << " a packet dropped after phy layer, size = "<<packet->GetSize());
}

void traceApMacQueueN(uint32_t oldValue, uint32_t newValue) //MAC队列的当前长度
{
  /* if(newValue == 15999)
  {
    std::cout<<Simulator::Now()<<" AP Mac queue numbers changed from " << oldValue << " to " << newValue<<std::endl;
  } */
}


//////////////////////////////////////////////////////
/******************PHY trace*************************/
//////////////////////////////////////////////////////
void tracePhyDrop(std::string context, Ptr<const Packet> packet, WifiPhyRxfailureReason reason) //物理层丢包
{
  Ptr<Packet> copy = packet->Copy();
  WifiMacHeader macheader;
  copy->PeekHeader(macheader);
  //NS_LOG_UNCOND("Time: " << Simulator::Now().GetSeconds() << context << " a packet has been dropped , size: " << copy->GetSize() << " TxAddr: " << macheader.GetAddr2() << " RxAddr: " << macheader.GetAddr1() << " type: " << macheader.GetTypeString() << " reason: " << reason);
}

//////////////////////////////////////////////////////
/****************Calculate throughput****************/
//////////////////////////////////////////////////////
void CalculateThroughput()  //每100ms统计一次吞吐量并输出
{
  Time now = Simulator::Now();                                     
  double cur = (totalRxMpduBytes - lastTotalRx) * (double)8 / 1e5; 
  NS_LOG_UNCOND(now.GetSeconds() << "s: \t" << cur << " Mbit/s");
  std::cout << now.GetSeconds() << "s: \t" << cur << " Mbit/s" << std::endl;
  if (n > 0)
  {
    throughputBytes += (totalRxMpduBytes - lastTotalRx);
  }
  ++n;
  lastTotalRx = totalRxMpduBytes;
  if (cur > maxThroughput)
  {
    maxThroughput = cur;
  }
  Simulator::Schedule(MilliSeconds(100), &CalculateThroughput);
}

void setTrace()   //连接计算吞吐和时延的trace
{
  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/AckedMpdu", MakeCallback(&traceMacRxMpdu));
  Config::Connect("/NodeList/0/DeviceList/0/$ns3::WifiNetDevice/Mac/MacTx", MakeCallback(&ApMacTxTrace));
  Config::Connect("/NodeList/[1-64]/DeviceList/0/$ns3::WifiNetDevice/Mac/MacRx", MakeCallback(&StaMacRxTrace));
}

void setEdcaTimer()
{
  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/BeMuEdcaTimer", TimeValue(MicroSeconds(2088960)));
  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/BkMuEdcaTimer", TimeValue(MicroSeconds(2088960)));
  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/ViMuEdcaTimer", TimeValue(MicroSeconds(2088960)));
  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/VoMuEdcaTimer", TimeValue(MicroSeconds(2088960)));
}

int main(int argc, char *argv[])
{

  //LogComponentEnable("PhyEntity", LogLevel(LOG_LEVEL_INFO | LOG_PREFIX_TIME));
  //LogComponentEnable("WifiPhy", LogLevel(LOG_LEVEL_INFO | LOG_PREFIX_TIME));
  //LogComponentEnable("InterferenceHelper", LogLevel(LOG_LEVEL_INFO | LOG_PREFIX_TIME));
  //LogComponentEnable("HeFrameExchangeManager", LogLevel(LOG_LEVEL_FUNCTION | LOG_PREFIX_TIME));
  //LogComponentEnable("RrMultiUserScheduler", LogLevel(LOG_LEVEL_FUNCTION | LOG_PREFIX_TIME));
  //LogComponentEnable("MultiUserScheduler", LogLevel(LOG_LEVEL_FUNCTION | LOG_PREFIX_TIME));
  //LogComponentEnable("FrameExchangeManager", LogLevel(LOG_LEVEL_FUNCTION | LOG_PREFIX_TIME));
  //LogComponentEnable("QosFrameExchangeManager", LogLevel(LOG_LEVEL_FUNCTION | LOG_PREFIX_TIME));
  //LogComponentEnable("QosTxop", LogLevel(LOG_LEVEL_FUNCTION | LOG_PREFIX_TIME));
  //LogComponentEnable("HtFrameExchangeManager", LogLevel(LOG_LEVEL_INFO | LOG_PREFIX_TIME));
  //LogComponentEnable("FrameExchangeManager", LogLevel(LOG_LEVEL_INFO | LOG_PREFIX_TIME));
  //LogComponentEnable("Txop", LogLevel(LOG_LEVEL_FUNCTION | LOG_PREFIX_TIME));
  //LogComponentEnable("idApps", LogLevel(LOG_LEVEL_INFO | LOG_PREFIX_TIME));
  //LogComponentEnable("WifiRemoteStationManager", LogLevel(LOG_LEVEL_FUNCTION | LOG_PREFIX_TIME));
  //LogComponentEnable("ApWifiMac",LogLevel(LOG_LEVEL_INFO|LOG_PREFIX_TIME));
  //LogComponentEnable("FqCoDelQueueDisc",LogLevel(LOG_LEVEL_INFO|LOG_PREFIX_TIME));
  //LogComponentEnable("IdealWifiManager",LogLevel(LOG_LEVEL_INFO|LOG_PREFIX_TIME));

  double simulationTime{1};                                          //change
  uint32_t payloadSize = 1440;
  uint32_t payloadSize_udp=1464;
  bool tracing{true};

  // 802.11 ax
  std::size_t axnStations{64};
  int axGi{800};
  bool useExtendedBlockAck{false};
  std::string dlAckSeqType{"NO-OFDMA"};
  int axMcs{11};

  CommandLine cmd(__FILE__);
  cmd.AddValue("tracing", "true if need pcap files", tracing);

  cmd.AddValue("axGuardInterval", "guardInterval", axGi);
  cmd.AddValue("simulationTime", "Simulation time in seconds", simulationTime);
  cmd.AddValue("useExtendedBlockAck", "Enable/disable use of extended BACK", useExtendedBlockAck);
  cmd.AddValue("dlAckType", "Ack sequence type for DL OFDMA (NO-OFDMA, ACK-SU-FORMAT, MU-BAR, AGGR-MU-BAR)",
               dlAckSeqType);

  cmd.AddValue("axMcs", "if set, limit testing to a specific MCS (0-11)", axMcs);
  cmd.AddValue("payloadSize", "The application payload size in bytes", payloadSize);
  cmd.Parse(argc, argv);

  Config::SetDefault("ns3::TcpL4Protocol::SocketType", TypeIdValue(TcpNewReno::GetTypeId()));   //这个算法被我修改过，屏蔽了满启动和拥塞控制过程，竞争窗口会一直维持在初始值不变，除非发生了拥塞
  Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(payloadSize));
  Config::SetDefault("ns3::TcpSocket::InitialCwnd", UintegerValue(250));        //初始竞争窗口
  Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(2896000));
  Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(2896000));
  Config::SetDefault("ns3::TcpSocket::DelAckCount", UintegerValue(8));
  Config::SetDefault("ns3::TcpSocketBase::MinRto", TimeValue (Seconds (10.0)));  //TCP最小重传等待时间，我发现这个值小了导致部分STA的吞吐特别低

  //This method overrides the initial value of the matching attribute. This method cannot fail: it will crash if the input attribute name or value is invalid.
  /********ax*********/
  NodeContainer wifiApNode;
  wifiApNode.Create(1);

  NodeContainer axWifiStaNodes;
  axWifiStaNodes.Create(axnStations);

  NetDeviceContainer apDevice;
  NetDeviceContainer axStaDevices;
  WifiMacHelper axMac;
  WifiHelper axWifi,axWifi1;

  axWifi.SetStandard(WIFI_STANDARD_80211ax);
  axWifi1.SetStandard(WIFI_STANDARD_80211ax);

  if (dlAckSeqType != "NO-OFDMA")
  {
    axMac.SetMultiUserScheduler("ns3::RrMultiUserScheduler",
                                "EnableUlOfdma", BooleanValue(true),
                                "EnableBsrp", BooleanValue(true),
                                "NStations", UintegerValue (4),
                                "UseCentral26TonesRus", BooleanValue (false));
  }

  if (dlAckSeqType == "ACK-SU-FORMAT")
  {
    Config::SetDefault("ns3::WifiDefaultAckManager::DlMuAckSequenceType",
                       EnumValue(WifiAcknowledgment::DL_MU_BAR_BA_SEQUENCE));
  }
  else if (dlAckSeqType == "MU-BAR")
  {
    Config::SetDefault("ns3::WifiDefaultAckManager::DlMuAckSequenceType",
                       EnumValue(WifiAcknowledgment::DL_MU_TF_MU_BAR));
  }
  else if (dlAckSeqType == "AGGR-MU-BAR")
  {
    Config::SetDefault("ns3::WifiDefaultAckManager::DlMuAckSequenceType",
                       EnumValue(WifiAcknowledgment::DL_MU_AGGREGATE_TF));
  }
  else if (dlAckSeqType != "NO-OFDMA")
  {
    NS_ABORT_MSG("Invalid DL ack sequence type (must be NO-OFDMA, ACK-SU-FORMAT, MU-BAR or AGGR-MU-BAR)");
  }

  axWifi.SetRemoteStationManager("ns3::IdealWifiManager",
                                 "RtsCtsThreshold", StringValue((dlAckSeqType == "NO-OFDMA")?"0":"7000000")); //开启OFDMA之后不能启用RTS/CTS
    
  axWifi1.SetRemoteStationManager("ns3::IdealWifiManager",
                                 "RtsCtsThreshold", StringValue((dlAckSeqType == "NO-OFDMA")?"0":"7000000")); //开启OFDMA之后不能启用RTS/CTS

  axWifi.ConfigHeOptions("GuardInterval", TimeValue(NanoSeconds(axGi)), 
                         "MpduBufferSize", UintegerValue(useExtendedBlockAck ? 256 : 64));    //ax的保护间隔和MPDU Buffer大小

  axWifi1.ConfigHeOptions("GuardInterval", TimeValue(NanoSeconds(axGi)), 
                         "MpduBufferSize", UintegerValue(useExtendedBlockAck ? 256 : 64));    //ax的保护间隔和MPDU Buffer大小

  Ssid ssid = Ssid("scenario5");
  /*
   * SingleModelSpectrumChannel cannot be used with 802.11ax because two
   * spectrum models are required: one with 78.125 kHz bands for HE PPDUs
   * and one with 312.5 kHz bands for, e.g., non-HT PPDUs (for more details,
   * see issue #408 (CLOSED))
   */
  Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();

  Ptr<LogDistancePropagationLossModel> lossmodel = CreateObject<LogDistancePropagationLossModel>();
  spectrumChannel->AddPropagationLossModel(lossmodel);

  SpectrumWifiPhyHelper phy;
  phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
  phy.SetChannel(spectrumChannel);

  std::string channelStr("{42, 80, BAND_5GHZ, 0}");

  phy.DisablePreambleDetectionModel();

  phy.Set("ChannelSettings", StringValue(channelStr));
  phy.Set("FixedPhyBand", BooleanValue(true));
  phy.Set("RxNoiseFigure", DoubleValue(1));

  phy.Set("RxGain", DoubleValue(3));
  phy.Set("TxGain", DoubleValue(3));

  phy.Set("Antennas", UintegerValue(4));
  phy.Set("MaxSupportedTxSpatialStreams", UintegerValue(4));
  phy.Set("MaxSupportedRxSpatialStreams", UintegerValue(2));
  phy.Set("TxPowerStart", DoubleValue(23));
  phy.Set("TxPowerEnd", DoubleValue(23));

  axMac.SetType("ns3::ApWifiMac",
                "EnableBeaconJitter", BooleanValue(false),
                "Ssid", SsidValue(ssid),
                "VI_MaxAmpduSize", UintegerValue(122879),
                "BE_MaxAmpduSize", UintegerValue(122879));
  apDevice = axWifi.Install(phy, axMac, wifiApNode);

  phy.Set("Antennas", UintegerValue(2));
  phy.Set("MaxSupportedTxSpatialStreams", UintegerValue(2));
  phy.Set("MaxSupportedRxSpatialStreams", UintegerValue(2));
  phy.Set("TxPowerStart", DoubleValue(20));
  phy.Set("TxPowerEnd", DoubleValue(20));

  axMac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssid),
                "VI_MaxAmpduSize", UintegerValue(122879),
                "BE_MaxAmpduSize", UintegerValue(122879));

  axStaDevices = axWifi1.Install(phy, axMac, axWifiStaNodes);

  Ptr<NetDevice> dev = wifiApNode.Get(0)->GetDevice(0);
  Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice>(dev);
  PointerValue ptr;
  Ptr<QosTxop> edca;
  Ptr<WifiMacQueue> queue;
  
  wifi_dev->GetMac()->TraceConnectWithoutContext("MacTxDrop",MakeCallback(&traceMacTxDrop));
  wifi_dev->GetMac()->TraceConnectWithoutContext("MacRxDrop",MakeCallback(&traceMacRxDrop));
  
  wifi_dev->GetMac()->GetAttribute("VI_Txop", ptr);
  edca = ptr.Get<QosTxop>();
  edca->SetTxopLimit(MicroSeconds(4096)); //设置Txop limit
  queue = edca->GetWifiMacQueue();
  queue->TraceConnectWithoutContext("Drop", MakeCallback(&traceMacQueueDrop));
  queue->TraceConnectWithoutContext("Expired", MakeCallback(&traceMacQueueExpired));
  //queue->TraceConnectWithoutContext("PacketsInQueue", MakeCallback(&traceApMacQueueN));
//queue->SetMaxDelay(MicroSeconds(20000));
//queue->SetMaxSize(QueueSize ("640p"));

//Config::Set("/NodeList/[i]/DeviceList/[i]/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/Txop/Queue/DropPolice", EnumValue(WifiMacQueue::DROP_OLDEST));
  for(std::size_t i=0; i<axnStations; i++)
  {
    Ptr<NetDevice> staDev = axWifiStaNodes.Get(i)->GetDevice(0);
    Ptr<WifiNetDevice> sta_wifi_dev = DynamicCast<WifiNetDevice>(staDev);
    sta_wifi_dev->GetMac()->SetAttribute("WaitBeaconTimeout",TimeValue (MilliSeconds (120+(i*10))));  //make sure every sta can associate successfully
  }

  /* RngSeedManager::SetSeed(1);
  RngSeedManager::SetRun(1);
  int64_t streamNumber = 100;
  streamNumber += acWifi.AssignStreams(apDevice, streamNumber);
  streamNumber += acWifi.AssignStreams(acStaDevices, streamNumber); */

  // mobility.image.png
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                "X", StringValue("7.5"),
                                "Y", StringValue("7.5"),
                                "Rho", StringValue("ns3::UniformRandomVariable[Min=0|Max=0]"));
  mobility.Install(wifiApNode);
  
    mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                "X", StringValue("7.5"),
                                "Y", StringValue("7.5"),
                                "Rho", StringValue("ns3::UniformRandomVariable[Min=0.5|Max=7.5]"));
  for (std::size_t i = 0; i < 64; i++)
  {
    mobility.Install(axWifiStaNodes.Get(i));
  }

  /* Internet stack*/
  InternetStackHelper stack;
  stack.Install(wifiApNode);
  stack.Install(axWifiStaNodes);

  Ipv4AddressHelper address;
  address.SetBase("192.168.1.0", "255.255.255.0");
  Ipv4InterfaceContainer apNodeInterface;
  Ipv4InterfaceContainer axStaNodeInterfaces;

  apNodeInterface = address.Assign(apDevice);
  axStaNodeInterfaces = address.Assign(axStaDevices);

  /* TrafficControlHelper apTch;
  apTch.Uninstall(apDevice);
  TrafficControlHelper staTch;
  staTch.Uninstall(axStaDevices); */

  /*  Setting applications */
  ApplicationContainer serverApp;
  uint16_t port1 = 1603;
  uint16_t port2 = 1604;
  
  /*udp*/
  for(std::size_t i=0; i<32; i++)
  {
    Ptr<Node> appSource = NodeList::GetNode(0);
    Ptr<idSender> sender = CreateObject<idSender>();
    InetSocketAddress dest(axStaNodeInterfaces.GetAddress(i),port1);
    DataRate rate("3750kb/s");
    dest.SetTos(0xb8);// UDP
    sender->SetRemote(dest);
    sender->SetDataRate(rate);
    sender->SetTrafficType(1);             //0 if tcp, 1 if udp
    sender->SetInterval(Seconds(10));       //第一个数据包和整体业务之间的时间间隔
    appSource->AddApplication(sender);
    sender->SetStartTime(Seconds(1.0+double(i*0.1))); //应用启动时间
    sender->SetPktSize(payloadSize_udp);
  }
  
  for (std::size_t i = 0; i < 32; i++)
  {
    Address localAddress(InetSocketAddress(axStaNodeInterfaces.GetAddress(i), port1));
    PacketSinkHelper packetSinkHelper("ns3::UdpSocketFactory", localAddress);
    serverApp = packetSinkHelper.Install(axWifiStaNodes.Get(i));
    serverApp.Start(Seconds(0.0));
    serverApp.Stop(Seconds(simulationTime + 10));
  }
/*tcp*/
  for(std::size_t i=32; i<64; i++)
  {
    Ptr<Node> appSource = NodeList::GetNode(0);
    Ptr<idSender> sender = CreateObject<idSender>();
    InetSocketAddress dest(axStaNodeInterfaces.GetAddress(i),port2);
    DataRate rate("308750kb/s");
    dest.SetTos(0xb8);
    sender->SetRemote(dest);
    sender->SetDataRate(rate);
    sender->SetTrafficType(0);             //0 if tcp, 1 if udp
    sender->SetInterval(Seconds(10));       //第一个数据包和整体业务之间的时间间
    appSource->AddApplication(sender);
    sender->SetStartTime(Seconds(1.0+double(i*0.1))); //应用启动时间
    sender->SetPktSize(payloadSize);
  }
  
  for (std::size_t i = 32; i < 64; i++)
  {
    Address localAddress(InetSocketAddress(axStaNodeInterfaces.GetAddress(i), port2));
    PacketSinkHelper packetSinkHelper("ns3::TcpSocketFactory", localAddress);
    serverApp = packetSinkHelper.Install(axWifiStaNodes.Get(i));
    serverApp.Start(Seconds(0.0));
    serverApp.Stop(Seconds(simulationTime + 10));
  }
  if (tracing)
  {
    phy.EnablePcap("scenario-5" , apDevice.Get(0));
  }

  Simulator::Schedule(Seconds(1.1), &CalculateThroughput);
  Simulator::Schedule(Seconds(10),&setTrace);
  Simulator::Schedule(Seconds(10),&setEdcaTimer);

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop", MakeCallback(&tracePhyDrop));
  
  //Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/VI_Txop/BackoffTrace", MakeCallback(&traceBackOff));
  //Config::Connect("/NodeList/0/DeviceList/0/$ns3::WifiNetDevice/Mac/MacTx", MakeCallback(&ApMacTxTrace));
  //Config::Connect("/NodeList/1/DeviceList/0/$ns3::WifiNetDevice/Mac/MacTx", MakeCallback(&ApMacTxTrace));


  /* dev = axWifiStaNodes.Get(46)->GetDevice(0);
  wifi_dev = DynamicCast<WifiNetDevice>(dev);
  wifi_dev->GetMac()->TraceConnectWithoutContext("MacRx", MakeCallback(&StaMacRxTrace));
 */

  Simulator::Schedule(Seconds(0), &Ipv4GlobalRoutingHelper::PopulateRoutingTables);

  Simulator::Stop(Seconds(simulationTime + 10));
  Simulator::Run();
  Simulator::Destroy();

  for(int i=0;i<64;i++)
  {
    if(stat[i].nPkt != 0)
    {
      NS_LOG_UNCOND(" macDelay of STA"<<i+1<<": \t"<<(stat[i].singleStaDelay / stat[i].nPkt).GetMilliSeconds());
      //NS_LOG_UNCOND(" IpL4Delay of STA"<<i+1<<": \t"<<(stat[i].singleStaIpL4Delay / stat[i].nPkt).GetMilliSeconds());
        //NS_LOG_UNCOND((stat[i].singleStaDelay / stat[i].nPkt).GetMilliSeconds());
    }
    else
    {
      NS_LOG_UNCOND("Delay of STA"<<i+1<<": \t"<<"0");
    }
  }
  for(int i=0;i<64;i++)
  {
    NS_LOG_UNCOND("Throughput of STA "<<i+1<<":  "<<stat[i].singleStaThrouput * (double) 8 / (simulationTime * 1e6));
      //NS_LOG_UNCOND(stat[i].singleStaThrouput * (double) 8 / (simulationTime * 1e6));
  }
  std::cout << "Mac Delay: " << (macDelay / nmac).As(Time::MS) << std::endl;                                                                                                                                                                           
  std::cout << "over" << std::endl;

  return 0;
}

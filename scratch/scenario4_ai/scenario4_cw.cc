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
#include "ns3/wifi-remote-station-manager.h"
#include "ns3/wifi-psdu.h"

using namespace ns3;
using namespace std;

int nRxMpdu = 0;
double totalRxMpduBytes = 0;
double lastTotalRx = 0;
/// 增加的一些变量 ////
double lastTotalRx_large_period = 0;
double SingleStaRx[64];
double SingleStaLastRx[64];
double SingleStaLastRx_large_period[64];
extern double SingleStacur[64];
double total_SingleStacur[64];

/// 计算时延的部分 ////
Time LastMacDelay_large_period;
Time singleStaLastDelay[64];
Time singleStaLastDelay_large_period[64];
extern double SingleStaDelay[64];
double Lastnmac_large_period;
double LastnPkt[64];
double LastnPkt_large_period[64];
///////////

int n = 0;
double throughputBytes = 0;
double maxThroughput = 0;
static int CWnd = 0;
static Time macDelay{Seconds(0)};
static double nmac = 0;

struct singleStaDelayAndThroughput{
  int nPkt;
  Time singleStaDelay;
  double singleStaThrouput;
} stat[64];

uint8_t ipAddrBuffer[4];
uint8_t macAddrBuffer[6];
uint32_t nSta;
//./ns3 run 'scratch/scenario3.cc --useExtendedBlockAck=true --dlAckType=AGGR-MU-BAR' 2>scenaria3.log
//////////////////////////////////////////
///////拓扑结点、设备、容器//////////////////
//////////////////////////////////////////

static NetDeviceContainer apDevice, acStaDevices, axStaDevices{};
static NodeContainer wifiApNode;//一个
static NodeContainer acWifiStaNodes, axWifiStaNodes;//64个

//////////////////////////////////
////增加计算吞吐量及丢包率的变量///////
//////////////////////////////////
static int apMacTxPackets = 0;
static int apMacTxBytes = 0;
static int apMacRxBytes = 0;
static int ALL_staMacRxPackets = 0;
static int last_apMacTxPackets = 0;
static int last_ALL_staMacRxPackets = 0;

//下行传输 AP维护的对应STA的发包接收包数目
static int apMacRxPackets_sta[64];//丢包率统计
static int apMacTxPackets_sta[64];//丢包率统计
static int last_apMacRxPackets_sta[64];//丢包率统计
static int last_apMacTxPackets_sta[64];//丢包率统计


//STA的发包接收包数目
static int staMacTxPackets[64];//丢包率统计
static int staMacRxPackets[64];//丢包率统计
static int last_staMacTxPackets[64];//丢包率统计
static int last_staMacRxPackets[64];//丢包率统计

static double totalDataRx = 0;

//////////////////////////////////////////
///////extern变量 多文件可用数据/////////////
//////////////////////////////////////////

const int M = 5;
extern uint16_t m_ns3ai_id = 2334;
extern std::string dlAckSeqType;
extern bool APdown_rewardFlag;
extern bool APup_rewardFlag;
extern double error_radio;            //定义于h文件中
//extern double error_radio_M[65][M+1];      //AP总的丢包率（存储了M个历史长度）
extern double totalThroughput;
extern double Throughput_1s;
extern double Delay_1s;
extern double sta_error_radio[64];//定义于h文件中
extern double ap_error_radio[64];//定义于h文件中
extern double net_error_radio[64];
extern double t_idle;
extern bool sta_begin_action;
extern int Ampdusize[65]; 

///////////////////////////////////
////时间变量、计算空闲时间占空比的部分///
///////////////////////////////////
double simulationTime{10};
int timestep    ;             //100ms
int timestep_large ;         //1s
Time last_start_time{Seconds(10)};  //记录上一次TXOP的开始时间，初始化为10s，因为是从10s开始统计
Time total_duration{Seconds(0)};   //记录一个调度周期内总的信道繁忙时间
Time last_duration{Seconds(0)};    //记录上一次的duration
Time zero_time{Seconds(0)}; 
Time total_Txop{Seconds(0.1)};

////////////////
////画图的部分///
////////////////
int graph_num = 0;
double Delay_graph[200];
double throughput_graph[200];


NS_LOG_COMPONENT_DEFINE("scenario4");

//////////////////////////////////////////////////////
/******************TXOP trace*************************/
//////////////////////////////////////////////////////
void traceTxop(std::string context,Time start_time, Time duration,uint8_t linkID)    //统计占空比
{     
   // NS_LOG_UNCOND("at time: " << Simulator::Now().GetNanoSeconds()<<"start time: " << start_time.GetNanoSeconds() <<" "<<"duration: "<<duration.GetNanoSeconds());
    if(start_time.GetNanoSeconds()!=last_start_time.GetNanoSeconds())
    {   total_Txop = total_Txop+(start_time-last_start_time);
        total_duration = total_duration + last_duration;
     //   NS_LOG_UNCOND("total_duration: " << total_duration.GetNanoSeconds() <<" "<<"last_duration: "<<last_duration.GetNanoSeconds());
        last_duration=duration;

    }
    else if(start_time == last_start_time)
    {
        if(duration.GetNanoSeconds()>=last_duration.GetNanoSeconds())
        {
            last_duration=duration;
         //   NS_LOG_UNCOND("total_duration: " << total_duration.GetNanoSeconds() <<" "<<"last_duration: "<<last_duration.GetNanoSeconds());
        }
    }
    last_start_time=start_time;
  //NS_LOG_UNCOND("at time: " << Simulator::Now().GetSeconds());
  //start_time.GetNanoSeconds
}

void traceVICw(uint32_t cw, uint8_t linkID)
	{
		std::cout<<"VI Txop Cw: "<<cw<<std::endl;
	}
///////////////////////////////////////////////////////////////////////////////////////////////////
/******************App trace**********************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////
/******************TCP trace*************************/
//////////////////////////////////////////////////////
void traceTcpCWND(std::string context, uint32_t oldValue, uint32_t newValue)    //TCP层的拥塞窗口大小
{
  if (oldValue != newValue)
  {
    //NS_LOG_UNCOND(context << "\t" <<Simulator::Now().GetSeconds() << " s   "<<"CWnd changed from "<<oldValue<<" to "<<newValue);
  }
  CWnd = newValue;
}

void traceAdvWnd(uint32_t oldValue, uint32_t newValue)                          //这个和下面那个都差不多，是接收方的接收窗口大小
{
  // std::cout<<"AdvWnd changed from "<<oldValue<<" to "<<newValue<<std::endl;
}

void traceRWnd(uint32_t oldValue, uint32_t newValue)
{
  // std::cout<<Simulator::Now().GetSeconds()<<"s   "<<"RWnd changed from "<<oldValue<<" to "<<newValue<<std::endl;
}

void traceTcpBIF(std::string context,uint32_t oldValue, uint32_t newValue)
{
  if(oldValue != newValue)
  {
    //NS_LOG_UNCOND(Simulator::Now().GetSeconds()<<"s   "<<" BytesInFlight: "<<newValue<<" availableWindow: "<<(CWnd-newValue)/1440);
  }
}

void traceSocket()                                                              //这个是我找问题的时候绑定的trace，所有socket的trace都必须在程序运行之后才能绑定，因为运行前没有创建这个对象，所以要用schedule
{
  Config::Connect("/NodeList/[1-64]/$ns3::TcpL4Protocol/SocketList/*/CongestionWindow", MakeCallback(&traceTcpCWND));
  //Config::ConnectWithoutContext("/NodeList/0/$ns3::TcpL4Protocol/SocketList/*/AdvWND", MakeCallback(&traceAdvWnd));
  //Config::ConnectWithoutContext("/NodeList/0/$ns3::TcpL4Protocol/SocketList/*/RWND", MakeCallback(&traceRWnd));
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/******************UDP trace**********************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////


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
    { //下行传输
      macHdr->GetAddr1().CopyTo(macAddrBuffer);
      nSta = int(macAddrBuffer[5]);
      stat[nSta-2].singleStaThrouput += q->GetPacketSize();
      SingleStaRx[nSta - 2] += q->GetPacketSize();  
      staMacRxPackets[nSta - 2]++;                     //sta接收到的包数目
      ALL_staMacRxPackets++;
    } //上行传输
    else
    {
      stat[nSta-2].singleStaThrouput += q->GetPacketSize();
      SingleStaRx[nSta - 2] += q->GetPacketSize();  
      apMacRxPackets_sta[nSta - 2]++; 
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
  // NS_LOG_UNCOND(context<<" backOff: "<<value);
}

void traceMacQueueDrop(Ptr<const WifiMpdu> item)    //数据包在队列中被丢弃
{
  std::cout << "at time: " << Simulator::Now().GetSeconds() << " Mac Dropped a pakcet, size = " << item->GetPacketSize() << " SeqN: " << item->GetHeader().GetSequenceNumber() << " from: " << item->GetHeader().GetAddr2() << " to: " << item->GetHeader().GetAddr1() << " type: " << item->GetHeader().GetTypeString() << std::endl;
}

void traceMacQueueExpired(Ptr<const WifiMpdu> item) //数据包在队列中由于超时被丢弃
{
  std::cout << "at time: " << Simulator::Now().GetSeconds() << " Mac a pakcet expired, size = " << item->GetPacketSize() << " SeqN: " << item->GetHeader().GetSequenceNumber() << " from: " << item->GetHeader().GetAddr2() << " to: " << item->GetHeader().GetAddr1() << " type: " << item->GetHeader().GetTypeString() << std::endl;
}

void traceMacTxDrop(Ptr<const Packet> packet)               //MAC层丢包在进入队列之前被丢弃
{
  std::cout << "at time: " << Simulator::Now().GetSeconds() << " a packet dropped before queueing, size = "<<packet->GetSize()<<std::endl;
}

void traceMacRxDrop(Ptr<const Packet> packet)               //数据包通过物理层后被MAC层丢弃
{
  std::cout << "at time: " << Simulator::Now().GetSeconds() << " a packet dropped after phy layer, size = "<<packet->GetSize()<<std::endl;
  NS_LOG_UNCOND("at time: " << Simulator::Now().GetSeconds() << " a packet dropped after phy layer, size = "<<packet->GetSize());
}

void traceApMacQueueN(uint32_t oldValue, uint32_t newValue) //MAC队列的当前长度
{
  /* if(newValue == 15999)
  {
    std::cout<<Simulator::Now()<<" AP Mac queue numbers changed from " << oldValue << " to " << newValue<<std::endl;
  } */
}
void traceRtsFailed(Mac48Address macaddr)
{
  NS_LOG_UNCOND("RTS failed: "<<macaddr);
}
void traceDataFailed(Mac48Address macaddr)
{
  NS_LOG_UNCOND("Data failed: "<<macaddr);
}
void traceRtsFinalFailed(Mac48Address macaddr)
{
  NS_LOG_UNCOND("RTS finally failed: "<<macaddr);
}
void traceDataFinalFailed(Mac48Address macaddr)
{
  NS_LOG_UNCOND("Data finally failed: "<<macaddr);
}

//////////////////////////////////////////////////////
/******************PHY trace*************************/
//////////////////////////////////////////////////////
//////////////
////修改函数///
//////////////

void
tracePhyTx(std::string context, WifiConstPsduMap psdus, WifiTxVector txVector, double txPowerW)
{
    for (const auto& psdu : psdus)
    {
        for (auto& mpdu : *PeekPointer(psdu.second))
        {
            WifiMacHeader hdr = mpdu->GetHeader();
            if (hdr.IsQosData() && hdr.HasData())
            {
              hdr.GetAddr2().CopyTo(macAddrBuffer); //发送地址
              nSta = int(macAddrBuffer[5]);
              if (nSta == 1)   //发送者为AP时
              {
                hdr.GetAddr1().CopyTo(macAddrBuffer);
                nSta = int(macAddrBuffer[5]);  
                apMacTxPackets_sta[nSta - 2]++;
                apMacTxPackets ++;
              }
             //上行传输
              else
              {
                staMacTxPackets[nSta - 2]++;                     //sta发送的包数目      
              }
                
            }         
        }
    }
}

void tracePhyDrop(std::string context, Ptr<const Packet> packet, WifiPhyRxfailureReason reason) //物理层丢包
{
  Ptr<Packet> copy = packet->Copy();
  WifiMacHeader macheader;
  copy->PeekHeader(macheader);
  //NS_LOG_UNCOND("Time: " << Simulator::Now().GetSeconds() << context << " a packet has been dropped , size: " << copy->GetSize() << " TxAddr: " << macheader.GetAddr2() << " RxAddr: " << macheader.GetAddr1() << " type: " << macheader.GetTypeString() << " reason: " << reason);
}
void traceRate(uint64_t oldValue, uint64_t newValue)
{
  NS_LOG_UNCOND("Rate: "<<newValue);
}

//////////////////////////////////////////////////////
/****************Calculate BER****************////////
//////////////////////////////////////////////////////
void Calculate_BER()
{ 
  ///////////////////////////
  //////计算AP总的丢包率///////
  ///////////////////////////
  int aptx=apMacTxPackets-last_apMacTxPackets;
  int starx=ALL_staMacRxPackets-last_ALL_staMacRxPackets;
  if(aptx==0)
  {
   error_radio = 0 ;//     计算AP的总丢包率
  }
  else
  {
   error_radio = 1-starx*1.0/aptx;//     计算AP的总丢包率
  }
  if(error_radio<0)
  {
    error_radio = 0;
  }
  last_apMacTxPackets = apMacTxPackets;
  last_ALL_staMacRxPackets = ALL_staMacRxPackets;

  ///////////////////////////
  //////计算STA的丢包率///////
  ///////////////////////////
  int k = 0;
  while(k<64)
  {
    int aprx = apMacRxPackets_sta[k]- last_apMacRxPackets_sta[k];
    int statx = staMacTxPackets[k]- last_staMacTxPackets[k];
    int aptx_= apMacTxPackets_sta[k]- last_apMacTxPackets_sta[k];
    int starx_= staMacRxPackets[k]- last_staMacRxPackets[k];
    if(statx==0)
    {
     sta_error_radio[k] = 0;
    }
    else
    {
     sta_error_radio[k] = 1-aprx*1.0/statx;
    }

    if(aptx_==0)
    {
     ap_error_radio[k] = 0;
    }
    else
    {
     ap_error_radio [k] = 1-starx_*1.0/aptx_;
    }
    
    if((aptx_ + statx)==0)
    {
     net_error_radio[k] = 0;
    }
    else
    {
     net_error_radio[k] = 1-(aprx + starx_)*1.0/(aptx_ + statx);
    }

    if(net_error_radio[k]<0)
    {
      net_error_radio[k]=0;
    }

    if(sta_error_radio[k]<0)
    {
      sta_error_radio[k]=0;
    }
  
    last_apMacRxPackets_sta[k] = apMacRxPackets_sta[k];
    last_staMacTxPackets[k] = staMacTxPackets[k];
    last_apMacTxPackets_sta[k] = apMacTxPackets_sta[k];
    last_staMacRxPackets[k] = staMacRxPackets[k];
    if(ap_error_radio [k]>=0&&ap_error_radio [k]<=1)
    { 
    }
    else
    {
      ap_error_radio [k] = error_radio;
    }
    // std::cerr << Simulator::Now().GetMilliSeconds() << " aptx为" << aptx <<  std::endl;
    // std::cerr << Simulator::Now().GetMilliSeconds() << " starx为" << starx <<  std::endl;
    // std::cerr << Simulator::Now().GetMilliSeconds() << " apMacRxPackets_sta[k]- last_apMacRxPackets_sta[k]为" << apMacRxPackets_sta[k]- last_apMacRxPackets_sta[k] <<  std::endl;
    // std::cerr << Simulator::Now().GetMilliSeconds() << " staMacTxPackets[k]- last_staMacTxPackets[k]为" << staMacTxPackets[k]- last_staMacTxPackets[k] <<  std::endl;
    // std::cerr << Simulator::Now().GetMilliSeconds() << " staMacRxPackets[k]- last_staMacRxPackets[k]为" << staMacRxPackets[k]- last_staMacRxPackets[k] <<  std::endl;
    // std::cerr << Simulator::Now().GetMilliSeconds() << " apMacTxPackets_sta[k]- last_apMacTxPackets_sta[k]为" << apMacTxPackets_sta[k]- last_apMacTxPackets_sta[k] <<  std::endl;
    // std::cerr << Simulator::Now().GetMilliSeconds() << " apMacRxPackets_sta[k]为" << apMacRxPackets_sta[k]<<  std::endl;
    // std::cerr << Simulator::Now().GetMilliSeconds() << " staMacTxPackets[k]为" << staMacTxPackets[k] <<  std::endl;
    // std::cerr << Simulator::Now().GetMilliSeconds() << " staMacRxPackets[k]为" << staMacRxPackets[k]<<  std::endl;
    // std::cerr << Simulator::Now().GetMilliSeconds() << " apMacTxPackets_sta[k]为" << apMacTxPackets_sta[k] <<  std::endl;
    // std::cerr << Simulator::Now().GetMilliSeconds() << " sta_error_radio为" << sta_error_radio[k] <<  std::endl;
    // std::cerr << Simulator::Now().GetMilliSeconds() << " ap_error_radio为" << ap_error_radio [k] <<  std::endl;
    // std::cerr << Simulator::Now().GetMilliSeconds() << " net_error_radio为" << net_error_radio[k] <<  std::endl;
    k++;

  }

}

//////////////////////////////////////////////////////
/****************Calculate throughput****************/
//////////////////////////////////////////////////////

void CalculateThroughput_large_period()
{
 Time now = Simulator::Now();       
 double Throughput = (totalRxMpduBytes - lastTotalRx_large_period) * (double)8 / (1e3 * timestep_large); /* Convert Application RX Packets to MBits. */
 Throughput_1s = Throughput;
 std::cout << now.GetSeconds() << "s的大间隔观测周期总吞吐量为: \t" << Throughput << " Mbit/s" << std::endl;
 if((nmac - Lastnmac_large_period)==0)
 {
  Delay_1s = 0;
 }
 else
 {
  Delay_1s = ((macDelay - LastMacDelay_large_period) * 1.0 / (nmac - Lastnmac_large_period)).GetSeconds()*1000;
 }
 std::cout << now.GetSeconds() << "total Delay: " << Delay_1s << " Ms" << std::endl;
 
 Delay_graph[graph_num] = Delay_1s;
 throughput_graph[graph_num] = Throughput;
 graph_num++;

 lastTotalRx_large_period = totalRxMpduBytes;
 LastMacDelay_large_period = macDelay;
 Lastnmac_large_period = nmac;
 

//  int j = 0; 
//  while(j<64)
//   {
//       std::cout << now.GetSeconds() << "STA"<<j+1<<": " << SingleStacur[j] << " Mbit/s" << std::endl;
//       j++;
//   }
  ///////////////////
  /////参数计算///////
  //////////////////
  // int i = 0;
  // double StaDelay;
  // while (i < 64)
  // { 
  //   if((stat[i].nPkt - LastnPkt[i])==0)
  //   {
  //     StaDelay = 0;
  //   }
  //   else
  //   {
  //     StaDelay = ((stat[i].singleStaDelay - singleStaLastDelay_large_period[i]) * 1.0 / (stat[i].nPkt - LastnPkt_large_period[i])).GetSeconds()*1000;
  //   }
  //   std::cout << now.GetSeconds() << "STA"<<i+1<<" Delay: " << StaDelay << " Ms" << std::endl;
  //   singleStaLastDelay_large_period[i] = stat[i].singleStaDelay; 
  //   LastnPkt_large_period[i] = stat[i].nPkt;
  //   std::cout << now.GetSeconds() << "STA"<<i+1<<" Throughput: " << (SingleStaRx[i] - SingleStaLastRx_large_period[i]) * (double)8 / (1e3 * timestep_large) << " Mbit/s" << std::endl;
  //   SingleStaLastRx_large_period[i] = SingleStaRx[i];
  //   i++;
  // }
 Simulator::Schedule(MilliSeconds(timestep_large), &CalculateThroughput_large_period);
}


void CalculateThroughput()  //每100ms统计一次吞吐量并输出
{
  Time now = Simulator::Now();                                     
  double cur = (totalRxMpduBytes - lastTotalRx) * (double)8 / (1e3 * timestep); 
  std::cout << now.GetSeconds() << "s: \t" << cur << " Mbit/s" << std::endl;
  ////////////////////////////
  /////信道空闲时间占空比计算/////
  ////////////////////////////
  t_idle=1-(double)total_duration.GetNanoSeconds()/(double)total_Txop.GetNanoSeconds();
  total_duration=zero_time;
  total_Txop=zero_time;
  std::cout << now.GetSeconds() << "s: \t占空比为" << t_idle << " " << std::endl;

  ///////////////////
  /////时延计算///////
  //////////////////
  int i = 0;
  while (i < 64)
  { 
    if((stat[i].nPkt - LastnPkt[i])==0)
    {
      SingleStaDelay[i] = 0;
    }
    else
    {
     SingleStaDelay[i] = ((stat[i].singleStaDelay - singleStaLastDelay[i]) * 1.0 / (stat[i].nPkt - LastnPkt[i])).GetSeconds()*1000;
    }
    singleStaLastDelay[i] = stat[i].singleStaDelay;
    LastnPkt[i] = stat[i].nPkt;
    i++;
  }
  
  ///////////////////
  /////吞吐量计算/////
  //////////////////

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
  //totalThroughput = totalRxMpduBytes * (double) 8 / ( simulationTime * 1000000.0);

  int j = 0;
  while(j<64)
  {
      SingleStacur[j] = (SingleStaRx[j] - SingleStaLastRx[j]) * (double)8 / (1e3 * timestep);
      SingleStaLastRx[j] = SingleStaRx[j];
      j++;
  }

  ///////////////////
  /////误码率计算/////
  //////////////////
  Calculate_BER();

  ///////////////////////
  //STA端调用参数交互函数///
  //////////////////////
  i = 0;
  while (i<32)
  {    
       Ptr<WifiRemoteStationManager> rlmanager = DynamicCast<WifiNetDevice>(acStaDevices.Get(i))->GetRemoteStationManager();
       rlmanager->ScheduleNextStateRead(i,0,total_SingleStacur[i]);   //进行间隔时间的仿真
       i++;   
  }
  i = 0;
  while (i<32)
  {    
       Ptr<WifiRemoteStationManager> rlmanager = DynamicCast<WifiNetDevice>(axStaDevices.Get(i))->GetRemoteStationManager();
       rlmanager->ScheduleNextStateRead(i,0,total_SingleStacur[i+32]);   //进行间隔时间的仿真
       i++;   
  }

  ///////////////////////
  //AP端调用参数交互函数///
  //////////////////////

  Ptr<WifiRemoteStationManager> rlmanager = DynamicCast<WifiNetDevice>(apDevice.Get(0))->GetRemoteStationManager();
  rlmanager->ScheduleNextStateRead(0,maxThroughput,cur);   //进行间隔时间的仿真
  
  // i = 0;
  // while(i<64)
  // {
  //   Ptr<NetDevice> dev = axWifiStaNodes.Get(i)->GetDevice(0);
  //   Ptr<WifiNetDevice> wifi_sta_dev = DynamicCast<WifiNetDevice>(dev);
  //   rlmanager->SetMaxAmpduSize(wifi_sta_dev->GetMac()->GetAddress(),Ampdusize[i+1] * 1536);
  //   i++;
  // }
  
  // ///////////////////////
  // //STA端调用动作置换函数///
  // //////////////////////
  sta_begin_action = true;
  APup_rewardFlag = false;
  i = 0;
  while (i<32)
  {    
       Ptr<WifiRemoteStationManager> rlmanager = DynamicCast<WifiNetDevice>(acStaDevices.Get(i))->GetRemoteStationManager();
       //rlmanager->action_length_sta();
       //rlmanager->action_cw_sta();
       rlmanager->ScheduleNextStateRead(i,0,total_SingleStacur[i]);   //进行间隔时间的仿真
       i++;   
  }
  i = 0;
  while (i<32)
  {    
       Ptr<WifiRemoteStationManager> rlmanager = DynamicCast<WifiNetDevice>(axStaDevices.Get(i))->GetRemoteStationManager();
       //rlmanager->action_length_sta();
       //rlmanager->action_cw_sta();
       rlmanager->ScheduleNextStateRead(i,0,total_SingleStacur[i+32]);   //进行间隔时间的仿真
       i++;   
  }
  sta_begin_action = false;

  Simulator::Schedule(MilliSeconds(timestep), &CalculateThroughput);
}

void setTrace()   //连接计算吞吐和时延的trace
{
  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/AckedMpdu", MakeCallback(&traceMacRxMpdu));
  Config::Connect("/NodeList/0/DeviceList/0/$ns3::WifiNetDevice/Mac/MacTx", MakeCallback(&ApMacTxTrace));
  Config::Connect("/NodeList/[1-64]/DeviceList/0/$ns3::WifiNetDevice/Mac/MacRx", MakeCallback(&StaMacRxTrace));
  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxPsduBegin", MakeCallback(&tracePhyTx));
  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/VI_Txop/TxopTrace", MakeCallback(&traceTxop));
  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/TxopTrace", MakeCallback(&traceTxop));

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


  double simulationTime{10};
  uint32_t payloadSize = 1440;
  bool tracing{true};


  // 802.11 ac
  bool acSgi{true};
  std::size_t acnStations{32};
  int acMcs{9};
  // 802.11 ax
  std::size_t axnStations{32};
  int axGi{800};
  bool useExtendedBlockAck{false};
  //std::string dlAckSeqType{"NO-OFDMA"};
  int axMcs{11};

  CommandLine cmd(__FILE__);
  cmd.AddValue("tracing", "true if need pcap files", tracing);
  cmd.AddValue("acGuardInterval", "acGuardInterval", acSgi);
  cmd.AddValue("axGuardInterval", "guardInterval", axGi);
  cmd.AddValue("simulationTime", "Simulation time in seconds", simulationTime);
  cmd.AddValue("useExtendedBlockAck", "Enable/disable use of extended BACK", useExtendedBlockAck);
  cmd.AddValue("dlAckType", "Ack sequence type for DL OFDMA (NO-OFDMA, ACK-SU-FORMAT, MU-BAR, AGGR-MU-BAR)",
               dlAckSeqType);
  cmd.AddValue("acMcs", "if set, limit testing to a specific MCS (0-9)", acMcs);
  cmd.AddValue("axMcs", "if set, limit testing to a specific MCS (0-11)", axMcs);
  cmd.AddValue("payloadSize", "The application payload size in bytes", payloadSize);
  cmd.AddValue("timestep","python timestep",timestep);
  cmd.AddValue("timestep_large","python timestep",timestep_large);//m_ns3ai_id
  cmd.AddValue("m_ns3ai_id","memory key",m_ns3ai_id);
  cmd.Parse(argc, argv);

  Config::SetDefault("ns3::TcpL4Protocol::SocketType", TypeIdValue(TcpNewReno::GetTypeId()));   //这个算法被我修改过，屏蔽了满启动和拥塞控制过程，竞争窗口会一直维持在初始值不变，除非发生了拥塞
  Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(payloadSize));
  Config::SetDefault("ns3::TcpSocket::InitialCwnd", UintegerValue(250));        //初始竞争窗口
  Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(2896000));
  Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(2896000));
  Config::SetDefault("ns3::TcpSocket::DelAckCount", UintegerValue(8));
  Config::SetDefault("ns3::TcpSocketBase::MinRto", TimeValue (Seconds (10.0)));  //TCP最小重传等待时间，我发现这个值小了导致部分STA的吞吐特别低


  /********ax*********/
  //NodeContainer wifiApNode;
  wifiApNode.Create(1);
  //NodeContainer acWifiStaNodes;
  acWifiStaNodes.Create(acnStations);
  //NodeContainer axWifiStaNodes;
  axWifiStaNodes.Create(axnStations);
  // NetDeviceContainer apDevice;
  // NetDeviceContainer axStaDevices;
  WifiMacHelper acMac,axMac;
  WifiHelper acWifi,axWifi,apWifi;

  acWifi.SetStandard(WIFI_STANDARD_80211ac);
  axWifi.SetStandard(WIFI_STANDARD_80211ax);
  apWifi.SetStandard(WIFI_STANDARD_80211ax);
      
  std::ostringstream oss1, oss2;
  oss1 << "VhtMcs" << acMcs;
  /*acWifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue(oss1.str()),
                                 "RtsCtsThreshold", StringValue("0")); */
  acWifi.SetRemoteStationManager("ns3::RLIdealWifiManager",
                                 "RtsCtsThreshold", StringValue("0"));

  oss2 << "HeMcs" << axMcs;
  axWifi.SetRemoteStationManager("ns3::RLIdealWifiManager",
                                 "RtsCtsThreshold", StringValue((dlAckSeqType == "NO-OFDMA")?"0":"7000000"));
  apWifi.SetRemoteStationManager("ns3::RLIdealWifiManager",
                                 "RtsCtsThreshold", StringValue((dlAckSeqType == "NO-OFDMA")?"0":"7000000"));

  acWifi.ConfigHtOptions("ShortGuardIntervalSupported", BooleanValue(acSgi));    //ac的保护间隔
  axWifi.ConfigHtOptions("ShortGuardIntervalSupported", BooleanValue(acSgi));
  axWifi.ConfigHeOptions("GuardInterval", TimeValue(NanoSeconds(axGi)), 
                         "MpduBufferSize", UintegerValue(useExtendedBlockAck ? 256 : 64));    //ax的保护间隔和MPDU Buffer大小
  apWifi.ConfigHtOptions("ShortGuardIntervalSupported", BooleanValue(acSgi));
  apWifi.ConfigHeOptions("GuardInterval", TimeValue(NanoSeconds(axGi)), 
                         "MpduBufferSize", UintegerValue(useExtendedBlockAck ? 256 : 64));    //ax的保护间隔和MPDU Buffer大小

  if (dlAckSeqType != "NO-OFDMA")
  {
    axMac.SetMultiUserScheduler("ns3::RrMultiUserScheduler",
                                "EnableUlOfdma", BooleanValue(true),
                                "EnableBsrp", BooleanValue(true),
                                "UseCentral26TonesRus", BooleanValue(false),
                                "NStations", UintegerValue(32));
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
    
  Ssid ssid = Ssid("scenario4");
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
  std::string channelStr("{42, 80, BAND_5GHZ, 0}");  //信道的参数
  phy.Set("ChannelSettings", StringValue(channelStr));
  phy.Set("FixedPhyBand", BooleanValue(true));
  phy.Set("RxNoiseFigure", DoubleValue(1));
  
  phy.Set("RxGain", DoubleValue(3));
  phy.Set("TxGain", DoubleValue(3));
  // phy.Set("CcaEdThreshold", DoubleValue (-72.0));

  phy.Set("Antennas", UintegerValue(4));
  phy.Set("MaxSupportedTxSpatialStreams", UintegerValue(4));
  phy.Set("MaxSupportedRxSpatialStreams", UintegerValue(2));
  phy.Set("TxPowerStart", DoubleValue(23));
  phy.Set("TxPowerEnd", DoubleValue(23));


  axMac.SetType("ns3::ApWifiMac",
                "EnableBeaconJitter", BooleanValue(false),
                "Ssid", SsidValue(ssid),
                "VI_MaxAmpduSize", UintegerValue(80 * 1536),
                "BE_MaxAmpduSize", UintegerValue(80 * 1536));

  apDevice = apWifi.Install(phy, axMac, wifiApNode);

  phy.Set("Antennas", UintegerValue(2));
  phy.Set("MaxSupportedTxSpatialStreams", UintegerValue(2));
  phy.Set("MaxSupportedRxSpatialStreams", UintegerValue(2));
  phy.Set("TxPowerStart", DoubleValue(20));
  phy.Set("TxPowerEnd", DoubleValue(20));
  acMac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssid),
                "VI_MaxAmpduSize", UintegerValue(80 * 1536),
                "BE_MaxAmpduSize", UintegerValue(80 * 1536));
  axMac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssid),
                "VI_MaxAmpduSize", UintegerValue(80 * 1536),
                "BE_MaxAmpduSize", UintegerValue(80 * 1536));
  acStaDevices = acWifi.Install(phy, acMac, acWifiStaNodes);
  axStaDevices = axWifi.Install(phy, axMac, axWifiStaNodes);

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
  //edca->TraceConnectWithoutContext("CwTrace", MakeCallback(&CwValueTracedCallback));
  queue = edca->GetWifiMacQueue();
  queue->TraceConnectWithoutContext("Drop", MakeCallback(&traceMacQueueDrop));
  queue->TraceConnectWithoutContext("Expired", MakeCallback(&traceMacQueueExpired));
  //queue->TraceConnectWithoutContext("PacketsInQueue", MakeCallback(&traceApMacQueueN));

  for(std::size_t i=0; i<acnStations; i++)      //这里是为了让每个STA能和AP成功关联
  {
    Ptr<NetDevice> staDev = acWifiStaNodes.Get(i)->GetDevice(0);
    Ptr<WifiNetDevice> sta_wifi_dev = DynamicCast<WifiNetDevice>(staDev);
    sta_wifi_dev->GetMac()->SetAttribute("WaitBeaconTimeout",TimeValue (MilliSeconds (120+(i*10))));  //make sure every sta can associate successfully
    ////////////STA 追踪 CW值////////////////
    sta_wifi_dev->GetMac()->GetAttribute("VI_Txop", ptr);
    edca = ptr.Get<QosTxop>();
    //edca->TraceConnectWithoutContext("CwTrace", MakeCallback(&CwValueTracedCallback));
  }

  for(std::size_t i=0; i<axnStations; i++)      //同上
  {
    Ptr<NetDevice> staDev = axWifiStaNodes.Get(i)->GetDevice(0);
    Ptr<WifiNetDevice> sta_wifi_dev = DynamicCast<WifiNetDevice>(staDev);
    sta_wifi_dev->GetMac()->SetAttribute("WaitBeaconTimeout",TimeValue (MilliSeconds (120+(i*10)+320)));  //make sure every sta can associate successfully
  
    ////////////STA 追踪 CW值////////////////
    sta_wifi_dev->GetMac()->GetAttribute("VI_Txop", ptr);
    edca = ptr.Get<QosTxop>();
    //edca->TraceConnectWithoutContext("CwTrace", MakeCallback(&CwValueTracedCallback));
  }

  /* RngSeedManager::SetSeed(1);
  RngSeedManager::SetRun(1);
  int64_t streamNumber = 100;
  streamNumber += acWifi.AssignStreams(apDevice, streamNumber);
  streamNumber += acWifi.AssignStreams(acStaDevices, streamNumber); */

  // mobility.
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                "X", StringValue("0.0"),
                                "Y", StringValue("0.0"),
                                "Rho", StringValue("ns3::UniformRandomVariable[Min=0|Max=0]"));
  mobility.Install(wifiApNode);
    
  mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                "X", StringValue("7.5"),
                                "Y", StringValue("7.5"),
                                "Rho", StringValue("ns3::UniformRandomVariable[Min=0.5|Max=7.5]"));
  //for (std::size_t i = 0; i < axnStations; i++)
  //{
  //    mobility.Install(axWifiStaNodes.Get(i));
  //}
  mobility.Install(acWifiStaNodes);
  mobility.Install(axWifiStaNodes);
    
  /* Internet stack*/
  InternetStackHelper stack;
  stack.Install(wifiApNode);
  stack.Install(acWifiStaNodes);
  stack.Install(axWifiStaNodes);

  Ipv4AddressHelper address;
  address.SetBase("192.168.1.0", "255.255.255.0");
  Ipv4InterfaceContainer apNodeInterface;
  Ipv4InterfaceContainer acStaNodeInterfaces;
  Ipv4InterfaceContainer axStaNodeInterfaces;

  apNodeInterface = address.Assign(apDevice);
  acStaNodeInterfaces = address.Assign(acStaDevices);
  axStaNodeInterfaces = address.Assign(axStaDevices);

  /* TrafficControlHelper apTch;
  apTch.Uninstall(apDevice);
  TrafficControlHelper staTch;
  staTch.Uninstall(axStaDevices); */

  /*  Setting applications */
  // TCP flow
  uint16_t port1 = 1603;
  uint16_t port2 = 1604;
    
  Address localAddress1(InetSocketAddress(apNodeInterface.GetAddress(0), port1));
  PacketSinkHelper packetSinkHelper1("ns3::TcpSocketFactory", localAddress1);
  ApplicationContainer serverApp1 = packetSinkHelper1.Install(wifiApNode.Get(0));
  serverApp1.Start(Seconds(0.0));
  serverApp1.Stop(Seconds(simulationTime + 10));
  
  for(std::size_t i=0; i<acnStations; i++)
  {
    Ptr<Node> appSource = NodeList::GetNode(i+1);
    Ptr<idSender> sender = CreateObject<idSender>();
    InetSocketAddress dest(apNodeInterface.GetAddress(0),port1);
    DataRate rate("156250kb/s");
    dest.SetTos(0xb8);  //0x70 AC_BE, 0x28 AC_BK, 0xb8 AC_VI, 0xc0 AC_VO
    sender->SetRemote(dest);    //设置目标地址
    sender->SetDataRate(rate);  //数率
    sender->SetTrafficType(0);             //0 if tcp, 1 if udp
    sender->SetInterval(Seconds(10));   //发第一个包和整体业务之间的时间间隔
    appSource->AddApplication(sender);
    sender->SetStartTime(Seconds(1.0+double(i*0.1)));   //应用启动的时间
    sender->SetPktSize(payloadSize);    //每个数据包的大小
  }

  Address localAddress2(InetSocketAddress(apNodeInterface.GetAddress(0), port2));
  PacketSinkHelper packetSinkHelper2("ns3::TcpSocketFactory", localAddress2);
  ApplicationContainer serverApp2 = packetSinkHelper2.Install(wifiApNode.Get(0));
  serverApp2.Start(Seconds(0.0));
  serverApp2.Stop(Seconds(simulationTime + 10));
  
  for(std::size_t i=0; i<axnStations; i++)      //这里面同上
  {
    Ptr<Node> appSource = NodeList::GetNode(i+33);
    Ptr<idSender> sender = CreateObject<idSender>();
    InetSocketAddress dest(apNodeInterface.GetAddress(0),port2);
    DataRate rate("156250kb/s");
    dest.SetTos(0xb8);   //0x70 AC_BE, 0x28 AC_BK, 0xb8 AC_VI, 0xc0 AC_VO
    sender->SetRemote(dest);
    sender->SetDataRate(rate);
    sender->SetTrafficType(0);             //0 if tcp, 1 if udp
    sender->SetInterval(Seconds(10));       //interval between first packet and the bulk of packets
    appSource->AddApplication(sender);
    sender->SetStartTime(Seconds(1.0+double(i*0.1)+3.2));
    sender->SetPktSize(payloadSize);
  }

  if (tracing)
  {
    phy.EnablePcap("scenario4-cw", apDevice.Get(0));
  }

  Simulator::Schedule(Seconds(10.0+timestep*0.001), &CalculateThroughput);  
  Simulator::Schedule(Seconds(10.0+timestep_large*0.001), &CalculateThroughput_large_period);    
  Simulator::Schedule(Seconds(10),&setTrace);  
  Simulator::Schedule(Seconds(10),&setEdcaTimer);               //10s后Mu Edca Parameters才开始生效，不然会影响10s前ARP协议的完成
  
  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop", MakeCallback(&tracePhyDrop));
  //Config::Connect("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Mac/VI_Txop", MakeCallback(&traceVICw));
  //Config::Connect("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Mac/Txop/$ns3::QosTxop", MakeCallback(&traceVICw));

  Simulator::Schedule(Seconds(0), &Ipv4GlobalRoutingHelper::PopulateRoutingTables);

  Simulator::Stop(Seconds(simulationTime + 10));
  Simulator::Run();
  // for(int i=0;i<64;i++)
  // {
  //   if(stat[i].nPkt != 0)
  //   {
  //     NS_LOG_UNCOND(" Delay of STA"<<i+1<<": \t"<<(stat[i].singleStaDelay / stat[i].nPkt).GetMilliSeconds());
  //   }
  //   else
  //   {
  //     NS_LOG_UNCOND("Delay of STA"<<i+1<<": \t"<<"0"<<" n : "<<stat[i].nPkt);
  //   }
  // }

  

  // std::cout << "Mac Delay: " << (macDelay / nmac).As(Time::MS) << std::endl;
  // std::cout << "over" << std::endl;

  double totalThroughput = totalRxMpduBytes * (double) 8 / ( simulationTime * 1000000.0);
  std::cout << "Throughput" << std::endl;
  std::cout << totalThroughput << " Mbit/s" << std::endl;

  for(int i=0;i<64;i++)
  {
    NS_LOG_UNCOND(" Throughput of STA "<<i+1<<":  "<<stat[i].singleStaThrouput * (double) 8 / (simulationTime * 1e6));
  }

  std::cout << "Delay Windows data: " << std::endl;
  for(int i=0;i<=graph_num;i++)
  {
    std::cout << Delay_graph[i] <<" , "<< std::endl;
  }

  std::cout << "Throughput Windows data: " << std::endl;
  for(int i=0;i<=graph_num;i++)
  {
    std::cout << throughput_graph[i] <<" , " << std::endl;
  }

  Simulator::Destroy();

  // for(int i=0;i<64;i++)
  // {
  //   if(stat[i].nPkt != 0)
  //   {
  //     NS_LOG_UNCOND(" Delay of STA"<<i+1<<": \t"<<(stat[i].singleStaDelay / stat[i].nPkt).GetMilliSeconds());
  //   }
  //   else
  //   {
  //     NS_LOG_UNCOND("Delay of STA"<<i+1<<": \t"<<"0"<<" n : "<<stat[i].nPkt);
  //   }
  // }
  // for(int i=0;i<64;i++)
  // {
  //   NS_LOG_UNCOND(" Throughput of STA "<<i+1<<":  "<<stat[i].singleStaThrouput * (double) 8 / (simulationTime * 1e6));
  // }
  // std::cout << "Mac Delay: " << (macDelay / nmac).As(Time::MS) << std::endl;
  // std::cout << "over" << std::endl;

  return 0;
}

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
#include "ns3/idApp.h"
#include "ns3/wifi-remote-station-manager.h"
#include "ns3/wifi-psdu.h"
#include "ns3/waveform-generator-helper.h"
#include "ns3/waveform-generator.h"
#include "ns3/non-communicating-net-device.h"
#include"math.h"
using namespace ns3;
using namespace std;

/// @brief ///AP???????????????


int nRxMpdu = 0;
double totalRxMpduBytes = 0;
double lastTotalRx = 0;
double lastTotalRx_large_period = 0;
double SingleStaRx[64];
double SingleStaLastRx[64];
extern double SingleStacur[64];
extern double SingleStaTht[64];
Ptr<SpectrumModel> SpectrumModelWifi5210MHz; //!< Spectrum model at 5210 MHz.
   
        class static_SpectrumModelWifi5210MHz_initializer
        {
        public:
        static_SpectrumModelWifi5210MHz_initializer()
        {
        BandInfo bandInfo;
        bandInfo.fc = 5210e6;
        bandInfo.fl = 5210e6 - 40e6;
        bandInfo.fh = 5210e6 + 40e6;

        Bands bands;
        bands.push_back(bandInfo);

        SpectrumModelWifi5210MHz = Create<SpectrumModel>(bands);
        }
        };
        static_SpectrumModelWifi5210MHz_initializer static_SpectrumModelWifi5210MHz_initializer_instance;
double g_signalDbmAvg; //!< Average signal power [dBm]
double g_noiseDbmAvg;  //!< Average noise power [dBm]
uint32_t g_samples;    //!< Number of samples
void
MonitorSniffRx(Ptr<const Packet> packet,
               uint16_t channelFreqMhz,
               WifiTxVector txVector,
               MpduInfo aMpdu,
               SignalNoiseDbm signalNoise,
               uint16_t staId)

{
    g_samples++;
    g_signalDbmAvg += ((signalNoise.signal - g_signalDbmAvg) / g_samples);
    g_noiseDbmAvg += ((signalNoise.noise - g_noiseDbmAvg) / g_samples);
    //NS_LOG_UNCOND("sample="<<Simulator::Now()<<signalNoise.signal<<   "   "<<signalNoise.noise <<  "  "<<staId<<"    "<<"   "<<aMpdu);
    //int snr;
    
}








int n = 0;
extern double throughputBytes;
double maxThroughput = 0;
static int CWnd = 0;
extern int sta_Ampdusize[2];
extern double snr[64];

uint8_t buffer[6];
int nSta;

//////////////////////////////////
////??????????????????????????????????????????///////
//////////////////////////////////
static int apMacTxPackets = 0;
static int apMacTxBytes = 0;
static int apMacRxBytes = 0;
static int ALL_staMacRxPackets = 0;
static int last_apMacTxPackets = 0;
static int last_ALL_staMacRxPackets = 0;

//???????????? AP???????????????STA????????????????????????
static int apMacRxPackets_sta[64];//???????????????
static int apMacTxPackets_sta[64];//???????????????
static int last_apMacRxPackets_sta[64];//???????????????
static int last_apMacTxPackets_sta[64];//???????????????


//STA????????????????????????
static int staMacTxPackets[64];//???????????????
static int staMacRxPackets[64];//???????????????
static int last_staMacTxPackets[64];//???????????????
static int last_staMacRxPackets[64];//???????????????

static double totalDataRx = 0;

//////////////////////////////////////////
///////??????????????????????????????//////////////////
//////////////////////////////////////////

static NetDeviceContainer apDevice, acStaDevices,axStaDevices{};
static NodeContainer wifiApNode;//??????
static NodeContainer acWifiStaNodes;//32???
static NodeContainer axWifiStaNodes;//32???

///


struct 
{
  double snr;
  double per;
  double idle;
  double cw_last;
  double length_last;
  double throughput;
}group1,group2;

///
//////////////////////////////////////////
///////extern?????? ?????????????????????/////////////
//////////////////////////////////////////

const int M = 5;
extern double error_radio;            //?????????h?????????
//extern double error_radio_M[65][M+1];      //AP???????????????????????????M??????????????????
extern double totalThroughput;
extern double sta_error_radio[64];//?????????h?????????
extern double ap_error_radio[64];//?????????h?????????  



extern double group_error_radio[2];
extern double group_error_radio_last[2];


extern double t_idle;
extern int nApMacQueue = 0;
extern bool flag;
extern std::map<ns3::Mac48Address /* recepient */, int /* AmpduSize */> AddrActMap;

///////////////////////////////////
////???????????????????????????????????????????????????///
///////////////////////////////////
double simulationTime{10};
int timestep=100    ;             //100ms
int timestep_large=200 ;         //1s
Time last_start_time{Seconds(10)};  //???????????????TXOP??????????????????????????????10s???????????????10s????????????
Time total_duration{Seconds(0)};   //???????????????????????????????????????????????????
Time last_duration{Seconds(0)};    //??????????????????duration
Time zero_time{Seconds(0)}; 
Time total_Txop{Seconds(0.1)};

//./ns3 run 'scratch/scenario2.cc --useExtendedBlockAck=true --dlAckType=AGGR-MU-BAR' 2>scenario2.log

NS_LOG_COMPONENT_DEFINE("scenario2");
//////////////////////////////////////////////////////
/******************TXOP trace*************************/
//////////////////////////////////////////////////////
void traceTxop(std::string context,Time start_time, Time duration,uint8_t linkID)    //???????????????
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

//////////////////////////////////////////////////////
/******************TCP trace*************************/
//////////////////////////////////////////////////////
void traceTcpCWND(std::string context, uint32_t oldValue, uint32_t newValue)    //TCP????????????????????????
{
  if (oldValue != newValue)
  {
    //NS_LOG_UNCOND(context << "\t" <<Simulator::Now().GetSeconds() << " s   "<<"CWnd changed from "<<oldValue<<" to "<<newValue);
  }
  CWnd = newValue;
}

void traceAdvWnd(uint32_t oldValue, uint32_t newValue)                          //?????????????????????????????????????????????????????????????????????
{
  // std::cout<<"AdvWnd changed from "<<oldValue<<" to "<<newValue<<std::endl;
}

void traceRWnd(uint32_t oldValue, uint32_t newValue)
{
  // std::cout<<Simulator::Now().GetSeconds()<<"s   "<<"RWnd changed from "<<oldValue<<" to "<<newValue<<std::endl;
}

void traceSocket()                                                              //???????????????????????????????????????trace?????????socket???trace???????????????????????????????????????????????????????????????????????????????????????????????????schedule
{
  Config::Connect("/NodeList/3/$ns3::TcpL4Protocol/SocketList/*/CongestionWindow", MakeCallback(&traceTcpCWND));
  //Config::ConnectWithoutContext("/NodeList/0/$ns3::TcpL4Protocol/SocketList/*/AdvWND", MakeCallback(&traceAdvWnd));
  //Config::ConnectWithoutContext("/NodeList/0/$ns3::TcpL4Protocol/SocketList/*/RWND", MakeCallback(&traceRWnd));
}

//////////////////////////////////////////////////////
/******************MAC trace*************************/
//////////////////////////////////////////////////////

//////////////
////????????????///
//////////////
void traceMacRxMpdu(std::string context, Ptr<const WifiMpdu> q)
{ 
  //Time now = Simulator::Now(); 
  //std::cout << now.GetSeconds() <<  " traceMacRxMpdu" << std::endl;
  // NS_LOG_UNCOND("At time " << Simulator::Now().GetSeconds() << " a " << q->GetSize() << " bytes mpdu was acked at Ap , packetSize = " << q->GetPacketSize());
  Ptr<const Packet> packet = q->GetPacket();
  const WifiMacHeader *macHdr = &q->GetHeader();
  if (macHdr->IsQosData() || macHdr->HasData())
  {
    nRxMpdu += 1;
    totalRxMpduBytes += q->GetPacketSize();

    macHdr->GetAddr1().CopyTo(buffer);
    nSta = int(buffer[5]);                              //???????????????mac????????????????????????????????????????????????STA
    if (nSta == 1) // reciever is AP ?????????????????????????????????????????????????????????
    { //????????????
      macHdr->GetAddr2().CopyTo(buffer);
      nSta = int(buffer[5]);
      SingleStaRx[nSta - 2] += q->GetPacketSize();      //????????????STA????????????ap??????????????????sta?????????????????????
      apMacRxPackets_sta[nSta - 2]++;                      //???????????????ap??????????????????sta???????????????????????????sta?????????ack?????????
    }
    else
    { //????????????
      SingleStaRx[nSta - 2] += q->GetPacketSize();      //index ???0?????? 
      staMacRxPackets[nSta - 2]++;                     //sta?????????????????????
      ALL_staMacRxPackets++;
    }
  }
}

void ApMacTxTrace(std::string context, Ptr<const Packet> packet)
{
  //NS_LOG_UNCOND("At time " << Simulator::Now().GetSeconds() << " Ap send a " << packet->GetSize() << " bytes packet");
  //std::cout << "Ap send a: " <<  packet->GetSize() << "  bytes packet" << std::endl;

  //apMacTxPackets += 1;
  //apMacTxBytes += packet->GetSize();
  if(packet->GetSize()>1000){
    TimestampTag timestamp;
    timestamp.SetTimestamp(Simulator::Now());
    packet->AddByteTag(timestamp);
  }
}


void traceBackOff(std::string context, const uint32_t value)    //trace?????????
{
  // NS_LOG_UNCOND(context<<" backOff: "<<value);
}

void traceMacQueueDrop(Ptr<const WifiMpdu> item)        //mac???????????????????????????
{
  //NS_LOG_UNCOND("at time: " << Simulator::Now().GetSeconds() << " Mac Dropped a pakcet, size = " << item->GetPacketSize() << " SeqN: " << item->GetHeader().GetSequenceNumber() << " from: " << item->GetHeader().GetAddr2() << " to: " << item->GetHeader().GetAddr1() << " type: " << item->GetHeader().GetTypeString());
}

void traceMacQueueExpired(Ptr<const WifiMpdu> item)     //mac???????????????????????????
{
  //NS_LOG_UNCOND("at time: " << Simulator::Now().GetSeconds() << " Mac a pakcet expired, size = " << item->GetPacketSize() << " SeqN: " << item->GetHeader().GetSequenceNumber() << " from: " << item->GetHeader().GetAddr2() << " to: " << item->GetHeader().GetAddr1() << " type: " << item->GetHeader().GetTypeString());
}

void traceApMacQueueN(uint32_t oldValue, uint32_t newValue)     //mac??????????????????
{
  nApMacQueue = newValue;
  // NS_LOG_UNCOND("AP Mac queue numbers changed from " << oldValue << " to " << newValue);
}

void traceRtsFailed(Mac48Address macaddr)
{
  //NS_LOG_UNCOND("RTS failed: "<<macaddr);
}
void traceDataFailed(Mac48Address macaddr)
{
  //NS_LOG_UNCOND("Data failed: "<<macaddr);
}
void traceRtsFinalFailed(Mac48Address macaddr)
{
  //NS_LOG_UNCOND("RTS finally failed: "<<macaddr);
}
void traceDataFinalFailed(Mac48Address macaddr)
{
  //NS_LOG_UNCOND("Data finally failed: "<<macaddr);
}
//////////////////////////////////////////////////////
/******************PHY trace*************************/
//////////////////////////////////////////////////////

//////////////
////????????????///
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
              hdr.GetAddr2().CopyTo(buffer); //????????????
              nSta = int(buffer[5]);
              if (nSta == 1)   //????????????AP???
              {
                hdr.GetAddr1().CopyTo(buffer);
                nSta = int(buffer[5]);  
                apMacTxPackets_sta[nSta - 2]++;
                apMacTxPackets ++;
              }
             //????????????
              else
              {
                staMacTxPackets[nSta - 2]++;                     //sta??????????????????      
              }
                
            }         
        }
    }
}

void tracePhyDrop(std::string context, Ptr<const Packet> packet, WifiPhyRxfailureReason reason) //???????????????
{
  Ptr<Packet> copy = packet->Copy();
  WifiMacHeader macheader;
  copy->PeekHeader(macheader);
  //NS_LOG_UNCOND("Time: " << Simulator::Now().GetSeconds() << context << " a packet has been dropped , size: " << copy->GetSize() << " TxAddr: " << macheader.GetAddr2() << " RxAddr: " << macheader.GetAddr1() << " type: " << macheader.GetTypeString() << " reason: " << reason);
}




void  traceMacTxDataFailed(Mac48Address value)

{
 NS_LOG_UNCOND(Simulator::Now().GetSeconds()<<value);
}

void  tracemcs(uint64_t oldValue, uint64_t newValue)


{
  NS_LOG_UNCOND(Simulator::Now().GetSeconds()<<"  "<<oldValue/1e6<<"      "<<newValue/1e6);
}



void traceRate(uint64_t oldValue, uint64_t newValue)
{
  //NS_LOG_UNCOND("Rate: "<<newValue);
}
//////////////////////////////////////////////////////
/****************Calculate throughput****************/
//////////////////////////////////////////////////////
//////////////
////????????????///
//////////////

void CalculateThroughput_large_period()
{
 Time now = Simulator::Now();       
 double Throughput = (totalRxMpduBytes - lastTotalRx_large_period) * (double)8 / (1e3 * timestep_large); /* Convert Application RX Packets to MBits. */
 NS_LOG_UNCOND(now.GetSeconds() << ":\t"<< Throughput );
 //std::cout << now.GetSeconds() << "s????????????????????????????????????: \t" << Throughput << " Mbit/s" << std::endl;

 lastTotalRx_large_period = totalRxMpduBytes;
 int j = 0; 
  while(j<64)
  {
      SingleStaTht[j] = (SingleStaRx[j] * (double)8) / ((now.GetMilliSeconds() - 10000) * 1e3);
      SingleStacur[j] = (SingleStaRx[j] - SingleStaLastRx[j]) * (double)8 / (1e3 * timestep_large);
      SingleStaLastRx[j] = SingleStaRx[j];
      //NS_LOG_UNCOND("STA"<<i+1<<": "<<SingleStacur[i]);
      j++;
  }
 Simulator::Schedule(MilliSeconds(timestep_large), &CalculateThroughput_large_period);
}

void CalculateThroughput()
{ 
  flag = true;
  Time now = Simulator::Now();                                     /* Return the simulator's virtual time. */
  double cur = (totalRxMpduBytes - lastTotalRx) * (double)8 / (1e3 * timestep); /* Convert Application RX Packets to MBits. */
  throughputBytes=cur;
  //std::cout << now.GetSeconds() << "s: \t" << cur << " Mbit/s" << std::endl;
  //NS_LOG_UNCOND(now.GetSeconds () << "s: \t" << cur << " Mbit/s");
  //NS_LOG_UNCOND(now.GetSeconds() << "s: \t" << cur);

  ////////////////////////////
  /////?????????????????????????????????/////
  ////////////////////////////
  t_idle=1-(double)total_duration.GetNanoSeconds()/(double)total_Txop.GetNanoSeconds();
  total_duration=zero_time;
  total_Txop=zero_time;
  //std::cout << now.GetSeconds() << "s: \t????????????" << t_idle << " " << std::endl;

  ///////////////////
  /////???????????????/////
  //////////////////
  lastTotalRx = totalRxMpduBytes;
  if (cur > maxThroughput)
  {
    maxThroughput = cur;
  }
  totalThroughput = totalRxMpduBytes * (double) 8 / ( simulationTime * 1000000.0);

  /* int j = 0;
  while(j<64)
  {
      SingleStaTht[j] = (SingleStaRx[j] * (double)8) / ((now.GetMilliSeconds() - 10000) * 1e3);
      SingleStacur[j] = (SingleStaRx[j] - SingleStaLastRx[j]) * (double)8 / (1e3 * timestep);
      SingleStaLastRx[j] = SingleStaRx[j];
      j++;
  } */
  

  ///////////////////
  /////???????????????/////
  //////////////////
  
  int aptx=apMacTxPackets-last_apMacTxPackets;
  int starx=ALL_staMacRxPackets-last_ALL_staMacRxPackets;
  error_radio=1-starx*1.0/aptx;//     ??????AP???????????????
  if(error_radio<0)
  {
    error_radio=0;
  }
  last_apMacTxPackets = apMacTxPackets;
  last_ALL_staMacRxPackets = ALL_staMacRxPackets;
  
  // std::cout<<"error_radio ??? "<<error_radio<<std::endl;
  // std::cout<<"aptx ??? "<<aptx<<std::endl;
  // std::cout<<"starx ??? "<<starx<<std::endl;
  
  int k = 0,t0=0,t1=0,t2=0,t3=0;
  //NS_LOG_UNCOND(Simulator::Now()<<" action: ");
  while(k<64)
  {
    sta_error_radio[k] = 1-(apMacRxPackets_sta[k]- last_apMacRxPackets_sta[k])*1.0/(staMacTxPackets[k]- last_staMacTxPackets[k]);
    ap_error_radio [k] = 1-(staMacRxPackets[k]- last_staMacRxPackets[k])*1.0/(apMacTxPackets_sta[k]- last_apMacTxPackets_sta[k]);
    
    if(sta_error_radio[k]<0)
    {
      sta_error_radio[k]=0;
    }
    if(staMacTxPackets[k]- last_staMacTxPackets[k]==0)   //??????????????????????????????1???
    {
      sta_error_radio[k]=1;
    }

    if (k<32)
    {
      t0+=(staMacRxPackets[k]- last_staMacRxPackets[k])+(apMacRxPackets_sta[k]-last_apMacRxPackets_sta[k]);
      t1+=(apMacTxPackets_sta[k]- last_apMacTxPackets_sta[k])+(staMacTxPackets[k]-last_staMacTxPackets[k]);
    }
    else
    {
      t2+=(staMacRxPackets[k]- last_staMacRxPackets[k])+(apMacRxPackets_sta[k]-last_apMacRxPackets_sta[k]);
      t3+=(apMacTxPackets_sta[k]- last_apMacTxPackets_sta[k])+(staMacTxPackets[k]-last_staMacTxPackets[k]);

    }



    // std::cerr << "sta_error_radio[k]???" << sta_error_radio[k]  <<  std::endl;
    // std::cerr << "ap rx ???" << apMacRxPackets_sta[k]- last_apMacRxPackets_sta[k]  <<  std::endl;
    // std::cerr << "sta tx ???" << staMacTxPackets[k]- last_staMacTxPackets[k]  <<  std::endl;

    // std::cout<<"staMacRxPackets "<< k <<"??? "<<staMacRxPackets[k]<<std::endl;
    // std::cout<<"last_staMacRxPackets "<< k <<"??? "<<last_staMacRxPackets[k]<<std::endl;
    // std::cout<<"apMacTxPackets_sta "<< k <<"??? "<<apMacTxPackets_sta[k]<<std::endl;
    // std::cout<<"last_apMacTxPackets_sta "<< k <<"??? "<<last_apMacTxPackets_sta[k]<<std::endl;
   //  std::cerr<<"ap_error_radio "<< k <<"??? "<<ap_error_radio[k]<<std::endl;
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
    k++;
  }

  ///////////////////////
  //AP???????????????????????????///
  //////////////////////



  ///////////////////////
  //STA???????????????????????????///
  //////////////////////
  //std::cerr<<"print"<<t0  <<t1  <<t2  <<t3 <<std::endl;

  group_error_radio[0]=(t1>0 ? (1-1.0*t0/t1) : 0);
  group_error_radio[1]=(t3>0 ? (1-1.0*t2/t3) : 0);
  group_error_radio[0]=std::max(0.0,group_error_radio[0]);
  group_error_radio[1]=std::max(0.0,group_error_radio[1]);
  
  Ptr<WifiRemoteStationManager> rlmanager = DynamicCast<WifiNetDevice>(acStaDevices.Get(1))->GetRemoteStationManager();
   
  //rlmanager->ScheduleNextStateRead(1,0,cur);

  Ptr<WifiRemoteStationManager> ApRlmanager = DynamicCast<WifiNetDevice>(apDevice.Get(0))->GetRemoteStationManager();
  int i=0;
  for(i=0;i<32;i++)
  {
    
    Ptr<WifiRemoteStationManager> rlmanager = DynamicCast<WifiNetDevice>(acStaDevices.Get(i))->GetRemoteStationManager();
    ApRlmanager->SetMaxAmpduSize(rlmanager->GetMac()->GetAddress(),sta_Ampdusize[0]);
  }
  
  for(i=0;i<32;i++)
  {
     Ptr<WifiRemoteStationManager> rlmanager = DynamicCast<WifiNetDevice>(axStaDevices.Get(i))->GetRemoteStationManager();

    ApRlmanager->SetMaxAmpduSize(rlmanager->GetMac()->GetAddress(),sta_Ampdusize[1]);
  }
  
  Simulator::Schedule(MilliSeconds(timestep), &CalculateThroughput);
}


void setInitialFrameLength()
{
  Ptr<WifiRemoteStationManager> ApRlmanager1 = DynamicCast<WifiNetDevice>(apDevice.Get(0))->GetRemoteStationManager();
  for(auto It=AddrActMap.begin(); It!=AddrActMap.end(); It++)
  {
    ApRlmanager1->SetMaxAmpduSize(It->first,It->second);
  }
}

void setTrace()
{
  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/AckedMpdu", MakeCallback(&traceMacRxMpdu));
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
 //LogComponentEnable("RLIdealWifiManager", LogLevel(LOG_LEVEL_DEBUG | LOG_PREFIX_TIME));
  //double simulationTime{10};
  double frequency{5};
  int channelWidth{80};
  uint32_t payloadSize = 1440;
  bool tracing{true};
  // 802.11 ac
  bool acSgi{true};
  //bool acSgi{false};
  std::size_t acnStations{32};
  int acMcs{8};
  // 802.11 ax
  std::size_t axnStations{32};
  int axGi{800};
  bool useExtendedBlockAck{true};
  std::string dlAckSeqType{"NO-OFDMA"};
  int axMcs{11};
  int epoch=0;
  std::string filename;

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
  cmd.AddValue("epoch","python epoch",epoch);
  cmd.AddValue("timestep","python timestep",timestep);
  cmd.AddValue("timestep_large","python timestep",timestep_large);
  cmd.AddValue("output_log_name","output_log_name",filename);
  cmd.Parse(argc, argv);
  
  Config::SetDefault("ns3::TcpL4Protocol::SocketType", TypeIdValue(TcpNewReno::GetTypeId()));   //?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(payloadSize));
  Config::SetDefault("ns3::TcpSocket::InitialCwnd", UintegerValue(250));        //??????????????????
  Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(2896000));
  Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(2896000));
  Config::SetDefault("ns3::TcpSocket::DelAckCount", UintegerValue(8));
  Config::SetDefault("ns3::TcpSocketBase::MinRto", TimeValue (Seconds (10.0)));  //TCP???????????????????????????????????????????????????????????????STA??????????????????

  /********ac*********/
 
  wifiApNode.Create(1);
  acWifiStaNodes.Create(acnStations);
  axWifiStaNodes.Create(axnStations);

  //NetDeviceContainer apDevice, acStaDevices;
  WifiMacHelper acMac;
  WifiHelper acWifi;

  //NetDeviceContainer axStaDevices;
  WifiMacHelper axMac;
  WifiHelper axWifi, apWifi;

  acWifi.SetStandard(WIFI_STANDARD_80211ac);
  axWifi.SetStandard(WIFI_STANDARD_80211ax);
  apWifi.SetStandard(WIFI_STANDARD_80211ax);

  std::ostringstream oss1, oss2;
  oss1 << "VhtMcs" << acMcs;
  acWifi.SetRemoteStationManager("ns3::RLIdealWifiManager",
                                  "RtsCtsThreshold", StringValue("7000000"));
 //  acWifi.SetRemoteStationManager("ns3::AarfWifiManager",
     //                             "RtsCtsThreshold", StringValue("7000000"));

  oss2 << "HeMcs" << axMcs;
  axWifi.SetRemoteStationManager("ns3::RLIdealWifiManager",
                                 "RtsCtsThreshold", StringValue((dlAckSeqType == "NO-OFDMA")?"7000000":"7000000"));
  apWifi.SetRemoteStationManager("ns3::RLIdealWifiManager",
                                 "RtsCtsThreshold", StringValue((dlAckSeqType == "NO-OFDMA")?"7000000":"7000000"));

  acWifi.ConfigHtOptions("ShortGuardIntervalSupported", BooleanValue(acSgi));    //ac???????????????
  axWifi.ConfigHtOptions("ShortGuardIntervalSupported", BooleanValue(acSgi));
  axWifi.ConfigHeOptions("GuardInterval", TimeValue(NanoSeconds(axGi)), 
                         "MpduBufferSize", UintegerValue(useExtendedBlockAck ? 256 : 64));    //ax??????????????????MPDU Buffer??????
  apWifi.ConfigHtOptions("ShortGuardIntervalSupported", BooleanValue(acSgi));
  apWifi.ConfigHeOptions("GuardInterval", TimeValue(NanoSeconds(axGi)), 
                         "MpduBufferSize", UintegerValue(useExtendedBlockAck ? 256 : 64));    //ax??????????????????MPDU Buffer??????

  if (dlAckSeqType != "NO-OFDMA")
  {
    axMac.SetMultiUserScheduler("ns3::RrMultiUserScheduler",
                                "EnableUlOfdma", BooleanValue(true),
                                "EnableBsrp", BooleanValue(true),
                                "UseCentral26TonesRus", BooleanValue(false),
                                "NStations", UintegerValue(32));
  }

  if (dlAckSeqType == "ACK-SU-FORMAT")                  //??????OFDMA??????ACK??????????????????????????????
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
  else if (dlAckSeqType != "NO-OFDMA")                  //check?????????????????????
  {
    NS_ABORT_MSG("Invalid DL ack sequence type (must be NO-OFDMA, ACK-SU-FORMAT, MU-BAR or AGGR-MU-BAR)");
  }

  Ssid ssid = Ssid("scenario2");
  /*
   * SingleModelSpectrumChannel cannot be used with 802.11ax because two
   * spectrum models are required: one with 78.125 kHz bands for HE PPDUs
   * and one with 312.5 kHz bands for, e.g., non-HT PPDUs (for more details,
   * see issue #408 (CLOSED))
   */
  Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();   //????????????multi?????????????????????????????????ns3??????

  Ptr<LogDistancePropagationLossModel> lossmodel = CreateObject<LogDistancePropagationLossModel>();     //??????????????????????????????
  spectrumChannel->AddPropagationLossModel(lossmodel);

  SpectrumWifiPhyHelper phy;
  phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
  phy.SetChannel(spectrumChannel);
  std::string channelStr("{42, 80, BAND_5GHZ, 0}");
  phy.Set("ChannelSettings", StringValue(channelStr));
  phy.Set("FixedPhyBand", BooleanValue(true));
  phy.Set("RxNoiseFigure", DoubleValue(1));     //????????????????????????ns3??????????????????????????????????????????
  phy.Set("RxGain", DoubleValue(3));
  phy.Set("TxGain", DoubleValue(3));
  // phy.Set("CcaEdThreshold", DoubleValue (-72.0));




  //********************************************senario_8***************************************************
   int interference=1;
   if( interference==1)
   {
        NodeContainer interferingNode;
        interferingNode.Create(1);
        MobilityHelper mobility;
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                "X", StringValue("7.5"),
                                "Y", StringValue("7.5"),
                                "Rho", StringValue("ns3::UniformRandomVariable[Min=100|Max=100]"));
        mobility.Install(interferingNode);
        Ptr<SpectrumValue> wgPsd =Create<SpectrumValue>(SpectrumModelWifi5210MHz);
        double power_interface=35;//??????????????????dbm
       
        double pathloss=10*3*log10(100)+46.67;//100??????????????????
        double power_dbm=power_interface-pathloss;//AP???????????????????????????dbm
        double w=pow(10,power_interface/10)/1000;//???????????????????????????W
        std::cout<<"????????????"<<power_dbm<<"dbm"<<std::endl;
        *wgPsd = w/ 80e6; // PSD spread across 20 MHz
        
        NS_LOG_INFO("wgPsd : " << *wgPsd
                               << " integrated power: " << Integral(*(GetPointer(wgPsd))));
        {
            WaveformGeneratorHelper waveformGeneratorHelper;
            waveformGeneratorHelper.SetChannel(spectrumChannel);
            waveformGeneratorHelper.SetTxPowerSpectralDensity(wgPsd);

            waveformGeneratorHelper.SetPhyAttribute("Period", TimeValue(Seconds(0.01)));//????????????
            waveformGeneratorHelper.SetPhyAttribute("DutyCycle", DoubleValue(0.1));//?????????
            NetDeviceContainer waveformGeneratorDevices =
                waveformGeneratorHelper.Install(interferingNode);
            Simulator::Schedule(Seconds(10),
                                &WaveformGenerator::Start,
                                waveformGeneratorDevices.Get(0)
                                    ->GetObject<NonCommunicatingNetDevice>()
                                    ->GetPhy()
                                    ->GetObject<WaveformGenerator>());
        }

        
   }

  axMac.SetType("ns3::ApWifiMac",
                "EnableBeaconJitter", BooleanValue(false),
                "Ssid", SsidValue(ssid),
                "VI_MaxAmpduSize", UintegerValue(6500631),
                "BE_MaxAmpduSize", UintegerValue(6500631));
   // AP setting //
  phy.Set("Antennas", UintegerValue(4));
  phy.Set("MaxSupportedTxSpatialStreams", UintegerValue(4));
  phy.Set("MaxSupportedRxSpatialStreams", UintegerValue(2));
  phy.Set("TxPowerStart", DoubleValue(23));
  phy.Set("TxPowerEnd", DoubleValue(23));
   phy.Set("CcaSensitivity", DoubleValue(-62));
  apDevice = apWifi.Install(phy, axMac, wifiApNode);//device???0 ?????????1
   phy.Set("CcaSensitivity", DoubleValue(-62));

  phy.Set("Antennas", UintegerValue(2));
  phy.Set("MaxSupportedTxSpatialStreams", UintegerValue(2));
  phy.Set("MaxSupportedRxSpatialStreams", UintegerValue(2));
  phy.Set("TxPowerStart", DoubleValue(20));
  phy.Set("TxPowerEnd", DoubleValue(20));
  acMac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssid),
                "BE_MaxAmpduSize", UintegerValue(6500631));
  axMac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssid),
                "BE_MaxAmpduSize", UintegerValue(6500631));
  acStaDevices = acWifi.Install(phy, acMac, acWifiStaNodes);//device???0-31   id???1-32
  axStaDevices = axWifi.Install(phy, axMac, axWifiStaNodes);//device???32-64  id???33-64
  

  //Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HtConfiguration/ShortGuardIntervalSupported", BooleanValue(acSgi)); //??????ac???????????????
  //Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/MpduBufferSize", UintegerValue(useExtendedBlockAck ? 256 : 64));    //ax?????????Block Ack
  //Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval", TimeValue(NanoSeconds(axGi)));      //ax???????????????
  
  Ptr<NetDevice> dev = wifiApNode.Get(0)->GetDevice(0); //????????????????????????txop limit???mac?????????????????????
  Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice>(dev);
  PointerValue ptr;
  Ptr<QosTxop> edca;
  Ptr<WifiMacQueue> queue;
  wifi_dev->GetMac()->GetAttribute("VI_Txop", ptr);
  edca = ptr.Get<QosTxop>();
  edca->SetTxopLimit(MicroSeconds(4096));       //txop limit?????????????????????limit????????????
  queue = edca->GetWifiMacQueue();
  //queue->SetMaxDelay(Seconds(10));     
  queue->TraceConnectWithoutContext("Drop", MakeCallback(&traceMacQueueDrop));
  queue->TraceConnectWithoutContext("Expired", MakeCallback(&traceMacQueueExpired));
  queue->TraceConnectWithoutContext("PacketsInQueue", MakeCallback(&traceApMacQueueN));

  for(std::size_t i=0; i<acnStations; i++)      //????????????????????????STA??????AP????????????
  {
    Ptr<NetDevice> staDev = acWifiStaNodes.Get(i)->GetDevice(0);
    Ptr<WifiNetDevice> sta_wifi_dev = DynamicCast<WifiNetDevice>(staDev);
    sta_wifi_dev->GetMac()->SetAttribute("WaitBeaconTimeout",TimeValue (MilliSeconds (120+(i*10))));  //make sure every sta can associate successfully
  }

  for(std::size_t i=0; i<axnStations; i++)      //??????
  {
    Ptr<NetDevice> staDev = axWifiStaNodes.Get(i)->GetDevice(0);
    Ptr<WifiNetDevice> sta_wifi_dev = DynamicCast<WifiNetDevice>(staDev);
    sta_wifi_dev->GetMac()->SetAttribute("WaitBeaconTimeout",TimeValue (MilliSeconds (120+(i*10)+320)));  //make sure every sta can associate successfully
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
                                "X", StringValue("7.5"),
                                "Y", StringValue("7.5"),
                                "Rho", StringValue("ns3::UniformRandomVariable[Min=0|Max=0]"));
  mobility.Install(wifiApNode);

  /* mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                "X", StringValue("7.5"),
                                "Y", StringValue("7.5"),
                                "Rho", StringValue("ns3::UniformRandomVariable[Min=1|Max=7.5]"));
  for (std::size_t i = 0; i < acnStations; i++)
  {
    mobility.Install(acWifiStaNodes.Get(i));
    mobility.Install(axWifiStaNodes.Get(i));
  } */

  mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                "X", StringValue("7.5"),
                                "Y", StringValue("7.5"),
                                "Rho", StringValue("ns3::UniformRandomVariable[Min=0.5|Max=7.5]"));
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

  /* Setting applications */
  ApplicationContainer serverApp;

  // TCP flow
  /* uint16_t port1 = 50000;
  uint16_t port2 = 60000; */
  uint16_t port1 = 1603;
  uint16_t port2 = 1604;
  for (std::size_t i = 0; i < acnStations; i++)
  {
    Address localAddress(InetSocketAddress(acStaNodeInterfaces.GetAddress(i), port1));
    PacketSinkHelper packetSinkHelper("ns3::TcpSocketFactory", localAddress);
    serverApp = packetSinkHelper.Install(acWifiStaNodes.Get(i));
    serverApp.Start(Seconds(0.0));
    serverApp.Stop(Seconds(simulationTime + 10));
  }

  for(std::size_t i=0; i<acnStations; i++)
  {
    Ptr<Node> appSource = NodeList::GetNode(0);
    Ptr<idSender> sender = CreateObject<idSender>();
    InetSocketAddress dest(acStaNodeInterfaces.GetAddress(i),port1);
    DataRate rate("156250kb/s");
    dest.SetTos(0x70);          //0x70 AC_BE, 0x28 AC_BK, 0xb8 AC_VI, 0xc0 AC_VO
    sender->SetRemote(dest);    //??????????????????
    sender->SetDataRate(rate);  //??????
    sender->SetTrafficType(0);             //0 if tcp, 1 if udp
    sender->SetInterval(Seconds(10));   //???????????????????????????????????????????????????
    appSource->AddApplication(sender);
    sender->SetStartTime(Seconds(1.0+double(i*0.1)));   //?????????????????????
    sender->SetPktSize(payloadSize);    //????????????????????????
  }

  for (std::size_t i = 0; i < axnStations; i++)
  {
    Address localAddress(InetSocketAddress(axStaNodeInterfaces.GetAddress(i), port2));
    PacketSinkHelper packetSinkHelper("ns3::TcpSocketFactory", localAddress);
    serverApp = packetSinkHelper.Install(axWifiStaNodes.Get(i));
    serverApp.Start(Seconds(0.0));
    serverApp.Stop(Seconds(simulationTime + 10));
  }

  for(std::size_t i=0; i<axnStations; i++)      //???????????????
  {
    Ptr<Node> appSource = NodeList::GetNode(0);
    Ptr<idSender> sender = CreateObject<idSender>();
    InetSocketAddress dest(axStaNodeInterfaces.GetAddress(i),port2);
    DataRate rate("156250kb/s");
    dest.SetTos(0x70);                     //0x70 AC_BE, 0x28 AC_BK, 0xb8 AC_VI, 0xc0 AC_VO
    sender->SetRemote(dest);
    sender->SetDataRate(rate);
    sender->SetTrafficType(0);             //0 if tcp, 1 if udp
    sender->SetInterval(Seconds(10));       //interval between first packet and the bulk of packets
    appSource->AddApplication(sender);
    sender->SetStartTime(Seconds(1.0+double(i*0.1)+3.2));
    sender->SetPktSize(payloadSize);
  }


   // Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/Phy/MonitorSnifferRx",
   //                                   MakeCallback(&MonitorSniffRx));
                                //  LogComponentEnable("InterferenceHelper",LogLevel(LOG_LEVEL_INFO|LOG_PREFIX_TIME));
                                      



  if (tracing)
  {
    std::cout<<"pcap"<<std::endl;
    phy.EnablePcap("scenario8_zlh", apDevice.Get(0));
    //phy.EnablePcap("scenario-2-" + std::to_string(frequency) + "-" + std::to_string(channelWidth), apDevice.Get(0));
    // phy.EnablePcapAll("scenario-2-" + std::to_string(frequency) + "-" + std::to_string(channelWidth));
  }

  Simulator::Schedule(Seconds(10.1+timestep*0.001), &CalculateThroughput);  
  Simulator::Schedule(Seconds(10.0+timestep_large*0.001), &CalculateThroughput_large_period);    
  
  Simulator::Schedule(Seconds(10.1+timestep*0.001),&setTrace);                   //10s?????????????????????????????????STA?????????
  Simulator::Schedule(Seconds(10),&setEdcaTimer);               //10s???Mu Edca Parameters?????????????????????????????????10s???ARP???????????????
  Simulator::Schedule(Seconds(10),&setInitialFrameLength);
  //Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop", MakeCallback(&tracePhyDrop));




  //Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$ns3::RLIdealWifiManager/Rate", MakeCallback(&tracemcs));
  //Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/MacTxDataFailed", MakeCallback(&traceMacTxDataFailed));
  //Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/AckedMpdu", MakeCallback(&traceMacRxMpdu));
  //Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/VI_Txop/BackoffTrace", MakeCallback(&traceBackOff));
  
  Config::Connect("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx", MakeCallback(&ApMacTxTrace));

  //Config::Connect("/NodeList/0/DeviceList/0/$ns3::WifiNetDevice/Mac/MacTx", MakeCallback(&StaMacTxTrace));
 
  // Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/MinCw",
  //             UintegerValue(31));
  // Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/MaxCw",
  //             UintegerValue(31));
  // Simulator::Schedule (Seconds (1.00000001), &traceSocket);     //socket???trace???????????????????????????????????????????????????

  Simulator::Schedule(Seconds(0), &Ipv4GlobalRoutingHelper::PopulateRoutingTables);
  Simulator::Stop(Seconds(simulationTime + 10.001));
  Simulator::Run();

  totalThroughput = totalRxMpduBytes * (double) 8 / ( simulationTime * 1000000.0);
  std::cout << "Throughput" << std::endl;
  std::cout << totalThroughput << " Mbit/s" << std::endl;

  for(int i = 0; i < 64; i++)
  {
    NS_LOG_UNCOND("STA"<<i+1<<": "<<SingleStaRx[i] * (double)8 / (simulationTime * 1e6));
  }
    std::cout << "over" << std::endl;
    
  Simulator::Destroy();
  
   totalThroughput = totalRxMpduBytes * (double) 8 / ( simulationTime * 1000000.0);
   std::cout << "Throughput" << std::endl;
   std::cout << totalThroughput << " Mbit/s" << std::endl;


  // for(int i = 0; i < 64; i++)
  // {
  //   NS_LOG_UNCOND("STA"<<i+1<<": "<<SingleStaRx[i] * (double)8 / (simulationTime * 1e6));
  // }
  // std::cout << "over" << std::endl;

  return 0;
}

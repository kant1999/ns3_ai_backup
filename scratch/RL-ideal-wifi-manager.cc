/*
 * Copyright (c) 2006 INRIA
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
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */

#include "RL-ideal-wifi-manager.h"

#include "math.h"

#include "ns3/core-module.h"
#include "ns3/log.h"
#include "ns3/ns3-ai-module.h"
#include "ns3/wifi-mac.h"
#include "ns3/wifi-phy.h"
#include "ns3/wifi-tx-vector.h"
#include "ns3/wifi-utils.h"

#include <algorithm>

// #include "windows.h"

namespace ns3
{

/**
 * \brief hold per-remote-station state for Ideal Wifi manager.
 *
 * This struct extends from WifiRemoteStation struct to hold additional
 * information required by the Ideal Wifi manager
 */
struct RLIdealWifiRemoteStation : public WifiRemoteStation
{
    double m_lastSnrObserved; //!< SNR of most recently reported packet sent to the remote station
    uint16_t m_lastChannelWidthObserved; //!< Channel width (in MHz) of most recently reported
                                         //!< packet sent to the remote station
    uint16_t m_lastNssObserved; //!<  Number of spatial streams of most recently reported packet
                                //!<  sent to the remote station
    double m_lastSnrCached;     //!< SNR most recently used to select a rate
    uint8_t m_lastNss;   //!< Number of spatial streams most recently used to the remote station
    WifiMode m_lastMode; //!< Mode most recently used to the remote station
    uint16_t
        m_lastChannelWidth; //!< Channel width (in MHz) most recently used to the remote station
};

/// To avoid using the cache before a valid value has been cached
static const double CACHE_INITIAL_VALUE = -100;

NS_OBJECT_ENSURE_REGISTERED(RLIdealWifiManager);

NS_LOG_COMPONENT_DEFINE("RLIdealWifiManager");

TypeId
RLIdealWifiManager::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::RLIdealWifiManager")
            .SetParent<WifiRemoteStationManager>()
            .SetGroupName("Wifi")
            .AddConstructor<RLIdealWifiManager>()
            .AddAttribute("BerThreshold",
                          "The maximum Bit Error Rate acceptable at any transmission mode",
                          DoubleValue(1e-6),
                          MakeDoubleAccessor(&RLIdealWifiManager::m_ber),
                          MakeDoubleChecker<double>())
            .AddTraceSource("Rate",
                            "Traced value for rate changes (b/s)",
                            MakeTraceSourceAccessor(&RLIdealWifiManager::m_currentRate),
                            "ns3::TracedValueCallback::Uint64");
    return tid;
}

RLIdealWifiManager::RLIdealWifiManager()
    : m_currentRate(0)
{
    if (this->id == 0) // AP创建参数交互接口
    {
        m_ns3ai_mod = new Ns3AIRL<AiRLLENGTHEnv, AiRLLENGTHAct>(m_ns3ai_id);
        m_ns3ai_mod->SetCond(2, 0);
    }

    id = device_id;
    device_id++;
    // std::cerr << Simulator::Now().GetMilliSeconds() << " RLIdealWifiManager ()发生，id为" << id
    // <<  std::endl;
    NS_LOG_FUNCTION(this);
}

RLIdealWifiManager::~RLIdealWifiManager()
{
    if (this->id == 0)
    {
        delete m_ns3ai_mod;
    }
    // std::cerr << "delete RLIdealWifiManager，id为" << id <<  std::endl;
    NS_LOG_FUNCTION(this);
}

void
RLIdealWifiManager::classify_ap() // 对STA进行分类时的函数
{
    int i = 1;
    while (i < 65)
    {
        //double snr = 10 * log10(10 * log10(sta_snr[i] / sta_recievepacketsnumber[i]));
        sta_type[i] = 0;
        i++;
    }
}

double
RLIdealWifiManager::compute_localreward(int sta_id)
{
    int m = 0;
    int i = 1;
    double max = 0;         // 所有用户中最大的吞吐量
    double total = 0;       // 所有用户的总吞吐量
    double average = 0;     // 所有用户的平均吞吐量
    double dqn_total = 0;   // 某个dqn网络的用户总吞吐量
    double dqn_average = 0; // 某个dqn网络的平均用户吞吐量
    double dqn_num = 0;     // 某个dqn网络的用户数目
    double reward = 0;
    while (m < 64)
    {
        total = total + SingleStacur[m];
        if (SingleStacur[m] > max)
        {
            max = SingleStacur[m];
        }
        m++;
    }
    while (i < 65)
    {
        if (sta_type[i] == sta_type[sta_id])
        {
            dqn_total = dqn_total + SingleStacur[i - 1];
            dqn_num = dqn_num + 1;
        }
        i++;
    }
    average = total / 64;
    dqn_average = dqn_total / dqn_num;
    reward = std::max(0.0,
                      SingleStacur[sta_id - 1] * (2 * dqn_average - SingleStacur[sta_id - 1]) /
                          (dqn_average * dqn_average));
    // reward = dqn_average * (2 * average - dqn_average)/(average * average);
    // std::cerr << "STA id为" << id << "本地reward为" << reward  <<  std::endl;
    // std::cerr << "STA id为" << id << "SingleStacur为" << SingleStacur[sta_id-1]  <<  std::endl;
    // std::cerr << "dqn_total为" << dqn_total << "dqn_num为" << dqn_num  <<  std::endl;
    // std::cerr << "average为" << average << "dqn_average为" << dqn_average  <<  std::endl;
    return reward;
}

double
RLIdealWifiManager::compute_globalreward()
{
    int m = 0;
    int N_total = 64;   // 用户的总数（包括AP吗？）
    double D_bound = 6; // 最低吞吐约束 6M
    double T_bound = 0; // 最低时间延迟约束
    double reward = 0;
    while (m < 64)
    {
       // reward = reward + floor(std::min(0.0, (SingleStaTht[m] - D_bound))) / (N_total * D_bound);
        m++;
    }
    if (reward != 0) // 若没有低于吞吐的STA，则全局奖励为0
    {
        //reward = reward - 1;
    }
    //throughputBytes_last=std::max(1.0,throughputBytes_last);
    //throughputBytes_last=std::max(1.0,throughputBytes_last);
  
   // throughputBytes_last=std::min(std::max(1.0,throughputBytes_last),99999.9);
    //throughputBytes=std::min(std::max(1.0,throughputBytes_last),99999.9);
    double reward2;
    double a=0.5;
    double EMA_Tht;
    if(initial_flag)
    {
        EMA_Tht = throughputBytes;
        initial_flag = false;
    }else
    {
        EMA_Tht=last_EMA_Tht+a*(throughputBytes-last_EMA_Tht);
    }
   // std::cerr << "t1="<<throughputBytes<<"t2="<<throughputBytes_last<<"reward2"<<std::endl;
    if (EMA_Tht < throughputBytes)
    {
        reward2 = throughputBytes / EMA_Tht;
    }

    else
    {
        reward2 = 0;
        
    }
    //reward2 = throughputBytes / 800;
    last_EMA_Tht=EMA_Tht;
    //reward2=throughputBytes/1000;
    
    return reward2;
}

void
RLIdealWifiManager::Zero()
{

}

void
RLIdealWifiManager::action_length_sta1(int a)
{
    if (a == 0) // 聚合帧大小不变
    {
        sta_Ampdusize[0] = last_Ampdusize[0];
        CrosslineFlag1 = false;
    }
    else if (a == 1) // 聚合帧大小增大
    {
        if (sta_Ampdusize[0] + 1536 * 10 > 98304)
        {
            CrosslineFlag1 = true;
        }
        else
        {
            sta_Ampdusize[0] += 1536 * 10;
            CrosslineFlag1 = false;
        }
    }
    else if (a == 2) // 减小聚合帧大小
    {
        if (sta_Ampdusize[0] <= 1536 * 10)
        {
            CrosslineFlag1 = true;
        }
        else
        {
            sta_Ampdusize[0] -= 1536 * 10;
            CrosslineFlag1 = false;
        }
    }


    // std::cerr << Simulator::Now().GetMilliSeconds() <<" MAC地址 "<< this->GetMac()->GetAddress()
    // << " MaxAmpduSize为" << sta_Ampdusize <<  std::endl;
    last_Ampdusize[0] = sta_Ampdusize[0];
}

void
RLIdealWifiManager::action_length_sta2(int a)
{
    if (a == 0) // 聚合帧大小不变
    {
        sta_Ampdusize[1] = last_Ampdusize[1];
        CrosslineFlag2 = false;
    }
    else if (a == 1) // 聚合帧大小增大
    {
        if (sta_Ampdusize[1] + 1536 * 10 > 122880)
        {
            CrosslineFlag2 = true;
        }
        else
        {
            sta_Ampdusize[1] += 1536 * 10;
            CrosslineFlag2 = false;
        }
    }
    else if (a == 2) // 减小聚合帧大小
    {
        if (sta_Ampdusize[1] <= 1536 * 10)
        {
            CrosslineFlag2 = true;
        }
        else
        {
            sta_Ampdusize[1] -= 1536 * 10;
            CrosslineFlag2 = false;
        }
    }


    // std::cerr << Simulator::Now().GetMilliSeconds() <<" MAC地址 "<< this->GetMac()->GetAddress()
    // << " MaxAmpduSize为" << sta_Ampdusize <<  std::endl;
    last_Ampdusize[1] = sta_Ampdusize[1];
}

void
RLIdealWifiManager::action_cw_sta1(int act)
{
    if (act == 0)
    {
        CW[0] = last_CW[0];
    }
    else if (act == 1)
    {
        CW[0] = (last_CW[0]+1) * 2 - 1;
    }
    else if (act == 2)
    {
        CW[0] = (last_CW[0]+1) / 2 - 1;
    }
    else
    {
        std::cout << "Unsupported agent type!" << std::endl;
        exit(0);
    }
    double min_cw = 15;
    double max_cw = 1023;
    if (CW[0] < min_cw || CW[0] > max_cw)
    {
        CrosslineFlag3 = true;
    }
    else
    {
        CrosslineFlag3 = false;
    }
    CW[0] = std::min(max_cw, std::max(CW[0], min_cw));
    // std::cout << "准备改变的CW值\t" << CW << " ;" << std::endl;
    if (CW[0])
    {
        //int i=0;
        //for(i=1;i<33;i++)
        //{
        //std::ostringstream oss, oss2, oss3;
        //oss << "/NodeList/" << i;
        //oss2 << oss.str() << "/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/MinCw";
        //Config::Set(oss2.str(), UintegerValue(CW[0]));
        //oss3 << oss.str() << "/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/MaxCw";
        //Config::Set(oss3.str(), UintegerValue(CW[0]));
        //}
        Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/MinCw", UintegerValue(CW[0]));
        Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/MaxCw", UintegerValue(CW[0]));
    }

    else
    {
        // NS_LOG_UNCOND("Default CW");
        Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/MinCw",
                    UintegerValue(15));
        Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/MaxCw",
                    UintegerValue(1023));
    }
    last_CW[0] = CW[0];
}

void
RLIdealWifiManager::action_cw_sta2(int act)
{
    if (act == 0)
    {
        CW[1] = last_CW[1];
    }
    else if (act == 1)
    {
        CW[1] = (last_CW[1]+1) * 2 - 1;
    }
    else if (act == 2)
    {
        CW[1] = (last_CW[1]+1) / 2 - 1;
    }
    else
    {
        std::cout << "Unsupported agent type!" << std::endl;
        exit(0);
    }
    double min_cw = 15;
    double max_cw = 1023;
    if (CW[1] < min_cw || CW[1] > max_cw)
    {
        CrosslineFlag2 = true;
    }
    else
    {
        CrosslineFlag2 = false;
    }
    CW[1] = std::min(max_cw, std::max(CW[1], min_cw));
    // std::cout << "准备改变的CW值\t" << CW << " ;" << std::endl;
    if (CW[1])
    {
        int i=0;
        for(i=33;i<65;i++)
        {
        std::ostringstream oss, oss2, oss3;
        oss << "/NodeList/" << i;
        oss2 << oss.str() << "/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/MinCw";
        Config::Set(oss2.str(), UintegerValue(CW[1]));
        oss3 << oss.str() << "/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/MaxCw";
        Config::Set(oss3.str(), UintegerValue(CW[1]));
        }
    }

    else
    {
        // NS_LOG_UNCOND("Default CW");
        Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/VI_Txop/MinCw",
                    UintegerValue(15));
        Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/VI_Txop/MaxCw",
                    UintegerValue(1023));
    }
    last_CW[1] = CW[1];
}

void
RLIdealWifiManager::ScheduleNextStateRead(uint16_t id, double maxThroughput, double cur)
{
    // error[id] = total_failpacketsnumber[id]/total_packetsnumber[id];//丢包率的计算
    if (this->id == 0) // AP
    {
        // error[this->id] = group_error_radio;
        // down_train(cur);
    }
    else // STA
    {
       // std::cerr << "group1_____error" << group_error_radio[0] << std::endl;
      //  std::cerr << "group2_____error" << group_error_radio[1] << std::endl;
        STAtrain(cur, this->id);
    }

    NS_LOG_FUNCTION(this);
}

void
RLIdealWifiManager::STAtrain(double cur, int sta_id)
{
    auto env = m_ns3ai_mod->EnvSetterCond(); ///< Acquire the Env memory for writing
    int i = 1;
    group_snr[0] = 0;
    group_snr[1] = 0;
    int bound=0;

    while (i < 65)
    {
        //std::cerr<<"danyonghutuntuliang"<<SingleStaTht[i-1]<<std::endl;
        if(SingleStaTht[i-1]>6)
        {
            bound++;
        }
        if(i<33)
        {
            group_snr[0]+=snr[i-1];
        }
        else
        {
            group_snr[1]+=snr[i-1];
        }
        i++;
    }
    group_snr[0] /= 32;
    group_snr[1] /= 32;
    env->snr[0]=RatioToDb(group_snr[0]);
    env->snr[1]=RatioToDb(group_snr[1]);
    

    //std::cerr << " STA id为" << id << " group1__snr为" << group_snr[0]/32 <<  std::endl;
    //std::cerr << " STA id为" << id << " group2__snr为" << group_snr[1]/32 <<  std::endl;
    // total_recievepacketsnumber[this->id] <<  std::endl;
    // // double snr;
    // if(sta_recievepacketsnumber[this->id]==0)
    // {
    //   snr = apsnr_M[this->id][M];  //若某个时间隙度未向某STA发包，则将上一个时间隙度的值赋给它；
    // }
    // else
    // {
    //   snr = 10*log10(sta_snr[this->id]/sta_recievepacketsnumber[this->id]);
    // }
    // std::cerr << " STA id为" << id << " snr为" << snr <<  std::endl;

    // 建立有M个历史长度的元组
    //  int i = 0;
    //  while(i<M)
    //  {
    //      sta_error_radio_M[i] = sta_error_radio_M[i+1];
    //      stasnr_M[i] = stasnr_M[i+1];
    //      sta_tidle_M[i] = sta_tidle_M[i+1];
    //      //std::cerr << Simulator::Now().GetMilliSeconds() << " snr[M]为" << stasnr_M[i] <<
    //      //std::endl;
    //      i++;
    //  }
    //  sta_error_radio_M[M] = sta_error_radio[sta_id-1];
    //  stasnr_M[M] = snr;
    //  sta_tidle_M[M] = t_idle;

    // 与内存池进行交互
    // int j = 0;

    // {
    //     env->snr[j] = stasnr_M[j];
    //     env->per[j] = sta_error_radio_M[j];
    //     env->t_idle[j] = sta_tidle_M[j];
    //     j++;
    // }
    // env->cw_last = last_CW;
    // env->length_last = last_Ampdusize / 1536;
    // env->type_id = sta_type[sta_id];
    // env->sta_id = sta_id;

    // // env->snr = snr;
    // // env->per = error[this->id];
    // env->bytes = nApMacQueue * 1.0 / 20000;
    // env->reward = SingleStacur[this->id]/800; //设置奖励为前一个时间间隙的吞吐量大小除以最大值
    if (CrosslineFlag1 || CrosslineFlag2 || CrosslineFlag3)
    {
        env->reward = -0.05;
    }
    else
    {
        env->reward = compute_globalreward();
       //env->reward = compute_globalreward();

    }
    env->per[0]=group_error_radio[0];
    env->per[1]=group_error_radio[1];
    //std::cerr<<"dabiaoshuliang"<<bound<<"          "<<throughputBytes<<std::endl;
    env->bound_num=bound;
    env->throughput = throughputBytes;
    if (t_idle >= 0 && t_idle <= 1)
    {
       env->t_idle = t_idle;
    }
    else
    {
       env->t_idle = 1;
    }
    env->cw_last[0]=(int)last_CW[0];
    env->cw_last[1]=(int)last_CW[1];
    env->length_last[0]=sta_Ampdusize[0]/1536;
    env->length_last[1]=sta_Ampdusize[1]/1536;
    //std::cerr<<"mpdu1=="<<sta_Ampdusize[0]/1536<<"mpdu2=="<<sta_Ampdusize[1]/1536<<std::endl;
    m_ns3ai_mod->SetCompleted(); ///< Release the memory and update conters

    /////////////////
    /////act读取/////
    ////////////////

    auto act = m_ns3ai_mod->ActionGetterCond(); ///< Acquire the Act memory for reading
    
    action_cw_sta1(act->cw[0]);
    //action_cw_sta2(act->cw[1]);
    action_length_sta1(act->length[0]);
    action_length_sta2(act->length[1]);
    
    /* if(act->cw[0]+act->cw[1]+act->length[0]+act->length[1]==0)
    {
        CrosslineFlag1=false;
        CrosslineFlag2=false;
    } */
    // NS_LOG_UNCOND(Simulator::Now()<<" action: "<< act->cw <<" "<< act->length);
    /////////////////
    /////参数归零/////
    ////////////////

    Zero();

    // 关闭内存池
    m_ns3ai_mod->GetCompleted(); ///< Release the memory, roll back memory version and update conters
}

void
RLIdealWifiManager::SetupPhy(const Ptr<WifiPhy> phy)
{
    NS_LOG_FUNCTION(this << phy);
    WifiRemoteStationManager::SetupPhy(phy);
}

uint16_t
RLIdealWifiManager::GetChannelWidthForNonHtMode(WifiMode mode) const
{
    NS_ASSERT(mode.GetModulationClass() != WIFI_MOD_CLASS_HT &&
              mode.GetModulationClass() != WIFI_MOD_CLASS_VHT &&
              mode.GetModulationClass() != WIFI_MOD_CLASS_HE);
    if (mode.GetModulationClass() == WIFI_MOD_CLASS_DSSS ||
        mode.GetModulationClass() == WIFI_MOD_CLASS_HR_DSSS)
    {
        return 22;
    }
    else
    {
        return 20;
    }
}

void
RLIdealWifiManager::DoInitialize()
{
    NS_LOG_FUNCTION(this);
    AddrActMap.insert({this->GetMac()->GetAddress(), (this->GetMac()->GetHeSupported())?ax_initial_framelength:ac_initial_framelength});
    //std::cout<<"shijian==============="<<Simulator::Now()<<std::endl;
    BuildSnrThresholds();
}

void
RLIdealWifiManager::BuildSnrThresholds()
{
    m_thresholds.clear();
    WifiMode mode;
    WifiTxVector txVector;
    uint8_t nss = 1;
    for (const auto& mode : GetPhy()->GetModeList())
    {
        txVector.SetChannelWidth(GetChannelWidthForNonHtMode(mode));
        txVector.SetNss(nss);
        txVector.SetMode(mode);
        NS_LOG_DEBUG("Adding mode = " << mode.GetUniqueName());
        AddSnrThreshold(txVector, GetPhy()->CalculateSnr(txVector, m_ber));
    }
    // Add all MCSes
    if (GetHtSupported())
    {
        for (const auto& mode : GetPhy()->GetMcsList())
        {
            for (uint16_t j = 20; j <= GetPhy()->GetChannelWidth(); j *= 2)
            {
                txVector.SetChannelWidth(j);
                if (mode.GetModulationClass() == WIFI_MOD_CLASS_HT)
                {
                    uint16_t guardInterval = GetShortGuardIntervalSupported() ? 400 : 800;
                    txVector.SetGuardInterval(guardInterval);
                    // derive NSS from the MCS index
                    nss = (mode.GetMcsValue() / 8) + 1;
                    NS_LOG_DEBUG("Adding mode = " << mode.GetUniqueName() << " channel width " << j
                                                  << " nss " << +nss << " GI " << guardInterval);
                    txVector.SetNss(nss);
                    txVector.SetMode(mode);
                    AddSnrThreshold(txVector, GetPhy()->CalculateSnr(txVector, m_ber));
                }
                else // VHT or HE
                {
                    uint16_t guardInterval;
                    if (mode.GetModulationClass() == WIFI_MOD_CLASS_VHT)
                    {
                        guardInterval = GetShortGuardIntervalSupported() ? 400 : 800;
                    }
                    else
                    {
                        guardInterval = GetGuardInterval();
                    }
                    txVector.SetGuardInterval(guardInterval);
                    for (uint8_t k = 1; k <= GetPhy()->GetMaxSupportedTxSpatialStreams(); k++)
                    {
                        if (mode.IsAllowed(j, k))
                        {
                            NS_LOG_DEBUG("Adding mode = " << mode.GetUniqueName()
                                                          << " channel width " << j << " nss " << +k
                                                          << " GI " << guardInterval);
                            txVector.SetNss(k);
                            txVector.SetMode(mode);
                            AddSnrThreshold(txVector, GetPhy()->CalculateSnr(txVector, m_ber));
                        }
                        else
                        {
                            NS_LOG_DEBUG("Mode = " << mode.GetUniqueName() << " disallowed");
                        }
                    }
                }
            }
        }
    }
}

double
RLIdealWifiManager::GetSnrThreshold(WifiTxVector txVector)
{
    NS_LOG_FUNCTION(this << txVector);
    auto it = std::find_if(m_thresholds.begin(),
                           m_thresholds.end(),
                           [&txVector](const std::pair<double, WifiTxVector>& p) -> bool {
                               return ((txVector.GetMode() == p.second.GetMode()) &&
                                       (txVector.GetNss() == p.second.GetNss()) &&
                                       (txVector.GetChannelWidth() == p.second.GetChannelWidth()));
                           });
    if (it == m_thresholds.end())
    {
        // This means capabilities have changed in runtime, hence rebuild SNR thresholds
        BuildSnrThresholds();
        it = std::find_if(m_thresholds.begin(),
                          m_thresholds.end(),
                          [&txVector](const std::pair<double, WifiTxVector>& p) -> bool {
                              return ((txVector.GetMode() == p.second.GetMode()) &&
                                      (txVector.GetNss() == p.second.GetNss()) &&
                                      (txVector.GetChannelWidth() == p.second.GetChannelWidth()));
                          });
        NS_ASSERT_MSG(it != m_thresholds.end(), "SNR threshold not found");
    }
    return it->first;
}

void
RLIdealWifiManager::AddSnrThreshold(WifiTxVector txVector, double snr)
{
    NS_LOG_FUNCTION(this << txVector.GetMode().GetUniqueName() << txVector.GetChannelWidth()
                         << snr);
    m_thresholds.emplace_back(snr, txVector);
}

WifiRemoteStation*
RLIdealWifiManager::DoCreateStation() const
{
    NS_LOG_FUNCTION(this);
    RLIdealWifiRemoteStation* station = new RLIdealWifiRemoteStation();
    Reset(station);
    return station;
}

void
RLIdealWifiManager::Reset(WifiRemoteStation* station) const
{
    NS_LOG_FUNCTION(this << station);
    RLIdealWifiRemoteStation* st = static_cast<RLIdealWifiRemoteStation*>(station);
    st->m_lastSnrObserved = 0.0;
    st->m_lastChannelWidthObserved = 0;
    st->m_lastNssObserved = 1;
    st->m_lastSnrCached = CACHE_INITIAL_VALUE;
    st->m_lastMode = GetDefaultMode();
    st->m_lastChannelWidth = 0;
    st->m_lastNss = 1;
}

void
RLIdealWifiManager::DoReportRxOk(WifiRemoteStation* station, double rxSnr, WifiMode txMode)
{
    NS_LOG_FUNCTION(this << station << rxSnr << txMode);
}

void
RLIdealWifiManager::DoReportRtsFailed(WifiRemoteStation* station)
{
    NS_LOG_FUNCTION(this << station);
}

void
RLIdealWifiManager::DoReportDataFailed(WifiRemoteStation* station)
{
    NS_LOG_FUNCTION(this << station);
}

void
RLIdealWifiManager::DoReportRtsOk(WifiRemoteStation* st,
                                  double ctsSnr,
                                  WifiMode ctsMode,
                                  double rtsSnr)
{
    NS_LOG_FUNCTION(this << st << ctsSnr << ctsMode.GetUniqueName() << rtsSnr);
    RLIdealWifiRemoteStation* station = static_cast<RLIdealWifiRemoteStation*>(st);
    station->m_lastSnrObserved = rtsSnr;
    station->m_lastChannelWidthObserved =
        GetPhy()->GetChannelWidth() >= 40 ? 20 : GetPhy()->GetChannelWidth();
    station->m_lastNssObserved = 1;
}

void
RLIdealWifiManager::DoReportDataOk(WifiRemoteStation* st,
                                   double ackSnr,
                                   WifiMode ackMode,
                                   double dataSnr,
                                   uint16_t dataChannelWidth,
                                   uint8_t dataNss)
{
    NS_LOG_FUNCTION(this << st << ackSnr << ackMode.GetUniqueName() << dataSnr << dataChannelWidth
                         << +dataNss);
    RLIdealWifiRemoteStation* station = static_cast<RLIdealWifiRemoteStation*>(st);
    if (dataSnr == 0)
    {
        NS_LOG_WARN("DataSnr reported to be zero; not saving this report.");
        return;
    }
    station->m_lastSnrObserved = dataSnr;
    station->m_lastChannelWidthObserved = dataChannelWidth;
    station->m_lastNssObserved = dataNss;
}

void
RLIdealWifiManager::DoReportAmpduTxStatus(WifiRemoteStation* st,
                                          uint16_t nSuccessfulMpdus,
                                          uint16_t nFailedMpdus,
                                          double rxSnr,
                                          double dataSnr,
                                          uint16_t dataChannelWidth,
                                          uint8_t dataNss)
{
    NS_LOG_FUNCTION(this << st << nSuccessfulMpdus << nFailedMpdus << rxSnr << dataSnr
                         << dataChannelWidth << +dataNss);
    RLIdealWifiRemoteStation* station = static_cast<RLIdealWifiRemoteStation*>(st);
    if (dataSnr == 0)
    {
        NS_LOG_WARN("DataSnr reported to be zero; not saving this report.");
        return;
    }
    station->m_lastSnrObserved = dataSnr;
    station->m_lastChannelWidthObserved = dataChannelWidth;
    station->m_lastNssObserved = dataNss;
}

void
RLIdealWifiManager::DoReportFinalRtsFailed(WifiRemoteStation* station)
{
    NS_LOG_FUNCTION(this << station);
    Reset(station);
}

void
RLIdealWifiManager::DoReportFinalDataFailed(WifiRemoteStation* station)
{
    NS_LOG_FUNCTION(this << station);
    Reset(station);
}

WifiTxVector
RLIdealWifiManager::DoGetDataTxVector(WifiRemoteStation* st, uint16_t allowedWidth)
{
    // if(this->id == 0)
    // {
    //     uint8_t buffer[6];
    //     st->m_state->m_address.CopyTo(buffer);
    //     int nSta = int(buffer[5]);
    //     this->GetMac()->SetAttribute("VI_MaxAmpduSize", UintegerValue(Ampdusize[nSta-1])); //
    //     122879 at most 80 MPDUs for an A-MPDU this->GetMac()->SetAttribute("VO_MaxAmpduSize",
    //     UintegerValue(Ampdusize[nSta-1])); // 122879 at most 80 MPDUs for an A-MPDU
    //     this->GetMac()->SetAttribute("BE_MaxAmpduSize", UintegerValue(Ampdusize[nSta-1])); //
    //     122879 at most 80 MPDUs for an A-MPDU this->GetMac()->SetAttribute("BK_MaxAmpduSize",
    //     UintegerValue(Ampdusize[nSta-1])); // 122879 at most 80 MPDUs for an A-MPDU
    // }
    // else
    // {
    //     this->GetMac()->SetAttribute("VI_MaxAmpduSize", UintegerValue(sta_Ampdusize)); // 122879
    //     at most 80 MPDUs for an A-MPDU this->GetMac()->SetAttribute("VO_MaxAmpduSize",
    //     UintegerValue(sta_Ampdusize)); // 122879 at most 80 MPDUs for an A-MPDU
    //     this->GetMac()->SetAttribute("BE_MaxAmpduSize", UintegerValue(sta_Ampdusize)); // 122879
    //     at most 80 MPDUs for an A-MPDU this->GetMac()->SetAttribute("BK_MaxAmpduSize",
    //     UintegerValue(sta_Ampdusize)); // 122879 at most 80 MPDUs for an A-MPDU

    // }
    NS_LOG_FUNCTION(this << st << allowedWidth);
    RLIdealWifiRemoteStation* station = static_cast<RLIdealWifiRemoteStation*>(st);
    // We search within the Supported rate set the mode with the
    // highest data rate for which the SNR threshold is smaller than m_lastSnr
    // to ensure correct packet delivery.
    WifiMode maxMode = GetDefaultModeForSta(st);
    WifiTxVector txVector;
    WifiMode mode;
    uint64_t bestRate = 0;
    uint8_t selectedNss = 1;
    uint16_t guardInterval;
    uint16_t channelWidth = std::min(GetChannelWidth(station), allowedWidth);
    txVector.SetChannelWidth(channelWidth);
    if ((station->m_lastSnrCached != CACHE_INITIAL_VALUE) &&
        (station->m_lastSnrObserved == station->m_lastSnrCached) &&
        (channelWidth == station->m_lastChannelWidth))
    {
        // SNR has not changed, so skip the search and use the last mode selected
        maxMode = station->m_lastMode;
        selectedNss = station->m_lastNss;
        NS_LOG_DEBUG("Using cached mode = " << maxMode.GetUniqueName() << " last snr observed "
                                            << station->m_lastSnrObserved << " cached "
                                            << station->m_lastSnrCached << " channel width "
                                            << station->m_lastChannelWidth << " nss "
                                            << +selectedNss);
    }
    else
    {
        if (GetHtSupported() && GetHtSupported(st))
        {
            for (uint8_t i = 0; i < GetNMcsSupported(station); i++)
            {
                mode = GetMcsSupported(station, i);
                txVector.SetMode(mode);
                if (mode.GetModulationClass() == WIFI_MOD_CLASS_HT)
                {
                    guardInterval = static_cast<uint16_t>(
                        std::max(GetShortGuardIntervalSupported(station) ? 400 : 800,
                                 GetShortGuardIntervalSupported() ? 400 : 800));
                    txVector.SetGuardInterval(guardInterval);
                    // If the node and peer are both VHT capable, only search VHT modes
                    if (GetVhtSupported() && GetVhtSupported(station))
                    {
                        continue;
                    }
                    // If the node and peer are both HE capable, only search HE modes
                    if (GetHeSupported() && GetHeSupported(station))
                    {
                        continue;
                    }
                    // Derive NSS from the MCS index. There is a different mode for each possible
                    // NSS value.
                    uint8_t nss = (mode.GetMcsValue() / 8) + 1;
                    txVector.SetNss(nss);
                    if (!txVector.IsValid() || nss > std::min(GetMaxNumberOfTransmitStreams(),
                                                              GetNumberOfSupportedStreams(st)))
                    {
                        NS_LOG_DEBUG("Skipping mode " << mode.GetUniqueName() << " nss " << +nss
                                                      << " width " << txVector.GetChannelWidth());
                        continue;
                    }
                    double threshold = GetSnrThreshold(txVector);
                    uint64_t dataRate = mode.GetDataRate(txVector.GetChannelWidth(),
                                                         txVector.GetGuardInterval(),
                                                         nss);
                    NS_LOG_DEBUG("Testing mode " << mode.GetUniqueName() << " data rate "
                                                 << dataRate << " threshold " << threshold
                                                 << " last snr observed "
                                                 << station->m_lastSnrObserved << " cached "
                                                 << station->m_lastSnrCached);
                    double snr = GetLastObservedSnr(station, channelWidth, nss);
                    if (dataRate > bestRate && threshold < snr)
                    {
                        NS_LOG_DEBUG("Candidate mode = " << mode.GetUniqueName() << " data rate "
                                                         << dataRate << " threshold " << threshold
                                                         << " channel width " << channelWidth
                                                         << " snr " << snr);
                        bestRate = dataRate;
                        maxMode = mode;
                        selectedNss = nss;
                    }
                }
                else if (mode.GetModulationClass() == WIFI_MOD_CLASS_VHT)
                {
                    guardInterval = static_cast<uint16_t>(
                        std::max(GetShortGuardIntervalSupported(station) ? 400 : 800,
                                 GetShortGuardIntervalSupported() ? 400 : 800));
                    txVector.SetGuardInterval(guardInterval);
                    // If the node and peer are both HE capable, only search HE modes
                    if (GetHeSupported() && GetHeSupported(station))
                    {
                        continue;
                    }
                    // If the node and peer are not both VHT capable, only search HT modes
                    if (!GetVhtSupported() || !GetVhtSupported(station))
                    {
                        continue;
                    }
                    for (uint8_t nss = 1; nss <= std::min(GetMaxNumberOfTransmitStreams(),
                                                          GetNumberOfSupportedStreams(station));
                         nss++)
                    {
                        txVector.SetNss(nss);
                        if (!txVector.IsValid())
                        {
                            NS_LOG_DEBUG("Skipping mode " << mode.GetUniqueName() << " nss " << +nss
                                                          << " width "
                                                          << txVector.GetChannelWidth());
                            continue;
                        }
                        double threshold = GetSnrThreshold(txVector);
                        uint64_t dataRate = mode.GetDataRate(txVector.GetChannelWidth(),
                                                             txVector.GetGuardInterval(),
                                                             nss);
                        NS_LOG_DEBUG("Testing mode = " << mode.GetUniqueName() << " data rate "
                                                       << dataRate << " threshold " << threshold
                                                       << " last snr observed "
                                                       << station->m_lastSnrObserved << " cached "
                                                       << station->m_lastSnrCached);
                        double snr = GetLastObservedSnr(station, channelWidth, nss);
                        if (dataRate > bestRate && threshold < snr)
                        {
                            NS_LOG_DEBUG("Candidate mode = "
                                         << mode.GetUniqueName() << " data rate " << dataRate
                                         << " channel width " << channelWidth << " snr " << snr);
                            bestRate = dataRate;
                            maxMode = mode;
                            selectedNss = nss;
                        }
                    }
                }
                else // HE
                {
                    guardInterval = std::max(GetGuardInterval(station), GetGuardInterval());
                    txVector.SetGuardInterval(guardInterval);
                    // If the node and peer are not both HE capable, only search (V)HT modes
                    if (!GetHeSupported() || !GetHeSupported(station))
                    {
                        continue;
                    }
                    for (uint8_t nss = 1; nss <= std::min(GetMaxNumberOfTransmitStreams(),
                                                          GetNumberOfSupportedStreams(station));
                         nss++)
                    {
                        txVector.SetNss(nss);
                        if (!txVector.IsValid())
                        {
                            NS_LOG_DEBUG("Skipping mode " << mode.GetUniqueName() << " nss " << +nss
                                                          << " width "
                                                          << +txVector.GetChannelWidth());
                            continue;
                        }
                        double threshold = GetSnrThreshold(txVector);
                        uint64_t dataRate = mode.GetDataRate(txVector.GetChannelWidth(),
                                                             txVector.GetGuardInterval(),
                                                             nss);
                        NS_LOG_DEBUG("Testing mode = " << mode.GetUniqueName() << " data rate "
                                                       << dataRate << " threshold " << threshold
                                                       << " last snr observed "
                                                       << station->m_lastSnrObserved << " cached "
                                                       << station->m_lastSnrCached);
                        double snr = GetLastObservedSnr(station, channelWidth, nss);
                        if (dataRate > bestRate && threshold < snr)
                        {
                            NS_LOG_DEBUG("Candidate mode = "
                                         << mode.GetUniqueName() << " data rate " << dataRate
                                         << " threshold " << threshold << " channel width "
                                         << channelWidth << " snr " << snr);
                            bestRate = dataRate;
                            maxMode = mode;
                            selectedNss = nss;
                        }
                    }
                }
            }
        }
        else
        {
            // Non-HT selection
            selectedNss = 1;
            for (uint8_t i = 0; i < GetNSupported(station); i++)
            {
                mode = GetSupported(station, i);
                txVector.SetMode(mode);
                txVector.SetNss(selectedNss);
                uint16_t channelWidth = GetChannelWidthForNonHtMode(mode);
                txVector.SetChannelWidth(channelWidth);
                double threshold = GetSnrThreshold(txVector);
                uint64_t dataRate = mode.GetDataRate(txVector.GetChannelWidth(),
                                                     txVector.GetGuardInterval(),
                                                     txVector.GetNss());
                NS_LOG_DEBUG("mode = " << mode.GetUniqueName() << " threshold " << threshold
                                       << " last snr observed " << station->m_lastSnrObserved);
                double snr = GetLastObservedSnr(station, channelWidth, 1);
                if (dataRate > bestRate && threshold < snr)
                {
                    NS_LOG_DEBUG("Candidate mode = " << mode.GetUniqueName() << " data rate "
                                                     << dataRate << " threshold " << threshold
                                                     << " snr " << snr);
                    bestRate = dataRate;
                    maxMode = mode;
                }
            }
        }
        NS_LOG_DEBUG("Updating cached values for station to " << maxMode.GetUniqueName() << " snr "
                                                              << station->m_lastSnrObserved);
        station->m_lastSnrCached = station->m_lastSnrObserved;
        station->m_lastMode = maxMode;
        station->m_lastNss = selectedNss;
    }
    NS_LOG_DEBUG("Found maxMode: " << maxMode << " channelWidth: " << channelWidth
                                   << " nss: " << +selectedNss);
    station->m_lastChannelWidth = channelWidth;
    if (maxMode.GetModulationClass() == WIFI_MOD_CLASS_HE)
    {
        guardInterval = std::max(GetGuardInterval(station), GetGuardInterval());
    }
    else if ((maxMode.GetModulationClass() == WIFI_MOD_CLASS_HT) ||
             (maxMode.GetModulationClass() == WIFI_MOD_CLASS_VHT))
    {
        guardInterval =
            static_cast<uint16_t>(std::max(GetShortGuardIntervalSupported(station) ? 400 : 800,
                                           GetShortGuardIntervalSupported() ? 400 : 800));
    }
    else
    {
        guardInterval = 800;
    }
    WifiTxVector bestTxVector{
        maxMode,
        GetDefaultTxPowerLevel(),
        GetPreambleForTransmission(maxMode.GetModulationClass(), GetShortPreambleEnabled()),
        guardInterval,
        GetNumberOfAntennas(),
        selectedNss,
        0,
        GetChannelWidthForTransmission(maxMode, channelWidth),
        GetAggregation(station)};
    uint64_t maxDataRate = maxMode.GetDataRate(bestTxVector);
    if (m_currentRate != maxDataRate)
    {
        NS_LOG_DEBUG("New datarate: " << maxDataRate);
        m_currentRate = maxDataRate;
    }

        if(this->id == 0)
    {
        uint8_t buffer1[6];
        station->m_state->m_address.CopyTo(buffer1);
        int nSta1 = int(buffer1[5]);
        snr[nSta1-2] = GetLastObservedSnr(station, GetChannelWidthForTransmission(maxMode, channelWidth), selectedNss);
    }

    return bestTxVector;
}

WifiTxVector
RLIdealWifiManager::DoGetRtsTxVector(WifiRemoteStation* st)
{
    NS_LOG_FUNCTION(this << st);
    RLIdealWifiRemoteStation* station = static_cast<RLIdealWifiRemoteStation*>(st);
    // We search within the Basic rate set the mode with the highest
    // SNR threshold possible which is smaller than m_lastSnr to
    // ensure correct packet delivery.
    double maxThreshold = 0.0;
    WifiTxVector txVector;
    WifiMode mode;
    uint8_t nss = 1;
    WifiMode maxMode = GetDefaultMode();
    // RTS is sent in a non-HT frame
    for (uint8_t i = 0; i < GetNBasicModes(); i++)
    {
        mode = GetBasicMode(i);
        txVector.SetMode(mode);
        txVector.SetNss(nss);
        txVector.SetChannelWidth(GetChannelWidthForNonHtMode(mode));
        double threshold = GetSnrThreshold(txVector);
        if (threshold > maxThreshold && threshold < station->m_lastSnrObserved)
        {
            maxThreshold = threshold;
            maxMode = mode;
        }
    }
    return WifiTxVector(
        maxMode,
        GetDefaultTxPowerLevel(),
        GetPreambleForTransmission(maxMode.GetModulationClass(), GetShortPreambleEnabled()),
        800,
        GetNumberOfAntennas(),
        nss,
        0,
        GetChannelWidthForNonHtMode(maxMode),
        GetAggregation(station));
}

double
RLIdealWifiManager::GetLastObservedSnr(RLIdealWifiRemoteStation* station,
                                       uint16_t channelWidth,
                                       uint8_t nss) const
{
    double snr = station->m_lastSnrObserved;
    if (channelWidth != station->m_lastChannelWidthObserved)
    {
        snr /= (static_cast<double>(channelWidth) / station->m_lastChannelWidthObserved);
    }
    if (nss != station->m_lastNssObserved)
    {
        snr /= (static_cast<double>(nss) / station->m_lastNssObserved);
    }
    NS_LOG_DEBUG("Last observed SNR is " << station->m_lastSnrObserved << " for channel width "
                                         << station->m_lastChannelWidthObserved << " and nss "
                                         << +station->m_lastNssObserved << "; computed SNR is "
                                         << snr << " for channel width " << channelWidth
                                         << " and nss " << +nss);
    return snr;
}

} // namespace ns3

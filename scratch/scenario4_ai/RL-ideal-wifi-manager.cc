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

#include "ns3/log.h"
#include "ns3/wifi-phy.h"
#include "ns3/wifi-tx-vector.h"
#include "ns3/wifi-utils.h"
#include "ns3/ns3-ai-module.h"
#include "ns3/core-module.h"
#include "ns3/wifi-mac.h"
#include <algorithm>
#include "math.h"
//#include "windows.h"

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
   id = device_id ;
   device_id++;
   if(this->id == 0)   //AP创建参数交互接口
    {
      m_ns3ai_mod = new Ns3AIRL<AiRLLENGTHEnv, AiRLLENGTHAct> (m_ns3ai_id);
      m_ns3ai_mod->SetCond (2, 0);
      //std::cerr <<  " RLIdealWifiManager ()发生，id为" << id <<  std::endl;
    }

   classify_ap();
   //std::cerr << Simulator::Now().GetMilliSeconds() << " RLIdealWifiManager ()发生，id为" << id <<  std::endl;
   
//    int i = 0;
//    while(i<65)
//   {
//    Ampdusize[i] = max_Ampdusiz;
//    i++;
//   }
//    i = 0;
//    while(i<Type)
//   {
//    Mpdulength[i] = max_Ampdusiz;
//    STAMpdulength[i] = max_Ampdusiz;
//    CWsize[i] = min_cw;
//    STACWsize[i] = min_cw;
//    i++;
//   }
   NS_LOG_FUNCTION(this);
}

RLIdealWifiManager::~RLIdealWifiManager()
{   
    if(this->id == 0)
    {
     delete m_ns3ai_mod;
    }
    //std::cerr << "delete RLIdealWifiManager，id为" << id <<  std::endl;
    NS_LOG_FUNCTION(this);
}


void
RLIdealWifiManager::classify_ap()   //对STA进行分类时的函数
{   
    //if (GetHeSupported())
    if (this->id>32||this->id==0) //HE
    {
     sta_type[this->id] = 1;
     sta_EDCA[this->id] = 0;    //All is BE
     //std::cerr << "HESupported，id为" << id <<  std::endl;
     Ampdusize[this->id] = max_axAmpdusiz;
     sta_Ampdusize = min_axAmpdusiz;
     Mpdulength[sta_type[this->id]] = min_axAmpdusiz;
     STAMpdulength[sta_type[this->id]] = min_axAmpdusiz;
    }
    else
    {
     sta_type[this->id] = 0;
     sta_EDCA[this->id] = 0;    //All is BE
     //std::cerr << "VHTSupported，id为" << id <<  std::endl;
     Ampdusize[this->id] = max_acAmpdusiz;
     sta_Ampdusize = min_acAmpdusiz;
     Mpdulength[sta_type[this->id]] = min_acAmpdusiz;
     STAMpdulength[sta_type[this->id]] = min_acAmpdusiz;
    }

     
     if(sta_EDCA[this->id]==0)  //BE
     {
       CWsize[sta_type[this->id]] = min_becw;
       STACWsize[sta_type[this->id]] = min_becw;
       CW = min_becw;
       last_CW = min_becw;
     }
     else                      //VI
     {
       CWsize[sta_type[this->id]] = min_vicw;
       STACWsize[sta_type[this->id]] = min_vicw;
       CW = min_vicw;
       last_CW = min_vicw;
     }

}

double
RLIdealWifiManager::compute_localreward(int EDCA)
{
    int m = 0;
    int i = 1;
    double BE_num = 0;
    double VI_num = 0;
    dqn_total_Tht = 0;
    dqn_total_Lat = 0;

    double max = 0;     //所有用户中最大的吞吐量
    double total = 0;   //所有用户的总吞吐量
    double average = 0; //所有用户的平均吞吐量
    double reward_Tht = 0;   //
    double reward_Lat = 0;   //
    while(m<64)
    {
        total = total + SingleStacur[m];
        if(SingleStacur[m]>max)
        {
        max = SingleStacur[m];
        }
        m++;
    }
    while(i<65)
    {   
        if(sta_EDCA[i] == 0)
        {
        dqn_total_Tht = dqn_total_Tht + SingleStacur[i-1];
        BE_num = BE_num + 1;
        }  

        if(sta_EDCA[i] == 1)
        {
        dqn_total_Lat = dqn_total_Lat + SingleStaDelay[i-1];  
        VI_num = VI_num + 1;
        }  
        
        i++;
    }
    average = total / 64;
    dqn_average_Tht = dqn_total_Tht / BE_num;
    dqn_average_Lat = dqn_total_Lat / VI_num;
    if(Last_slide_dqn_average_Lat==0)
    {
       Last_slide_dqn_average_Lat = dqn_average_Lat;
    }
    if(Last_slide_dqn_total_Tht==0)
    {
       Last_slide_dqn_total_Tht = dqn_total_Tht;
    }
    slide_dqn_average_Lat = Last_slide_dqn_average_Lat + 0.5 * (dqn_average_Lat - Last_slide_dqn_average_Lat );
    slide_dqn_total_Tht = Last_slide_dqn_total_Tht + 0.5 * (dqn_total_Tht - Last_slide_dqn_total_Tht );
    

    if(slide_dqn_average_Lat>dqn_average_Lat)
    {
       reward_Lat = slide_dqn_average_Lat * 1.0 / dqn_average_Lat;
    }
    else
    {
       reward_Lat = 0;
    }

    if(slide_dqn_total_Tht<dqn_total_Tht)
    {
       reward_Tht = dqn_total_Tht * 1.0 / slide_dqn_total_Tht;
    }
    else
    {
       reward_Tht = 0;
    }

    Last_dqn_average_Lat = dqn_average_Lat;
    Last_dqn_total_Tht = dqn_total_Tht;
    Last_slide_dqn_average_Lat = slide_dqn_average_Lat;
    Last_slide_dqn_total_Tht = slide_dqn_total_Tht;
    
    
    if(EDCA==0)
    { 
      std::cerr << "STA id为" << id << "本地reward_Tht为" << reward_Tht  <<  std::endl;
      std::cerr << "dqn_total_Tht为" << dqn_total_Tht  <<  std::endl;
      std::cerr << "slide_dqn_total_Tht为" << slide_dqn_total_Tht  <<  std::endl;
      return reward_Tht;
      //std::cerr << "STA id为" << id << "本地reward_Tht为" << reward_Tht  <<  std::endl;
    }
    std::cerr << "STA id为" << id << "本地reward_Lat为" << reward_Lat  <<  std::endl;
    std::cerr << "dqn_average_Lat为" << dqn_average_Lat  <<  std::endl;
    std::cerr << "slide_dqn_average_Lat为" << slide_dqn_average_Lat  <<  std::endl;
    return reward_Lat;
}

double
RLIdealWifiManager::compute_globalreward()
{
    int m = 0;
    int N_total = 64;       //用户的总数（包括AP吗？）
    double D_bound = 6;     //最低吞吐约束 6M
    double T_bound = 20;     //最低时间延迟约束
    double reward_Tht = 0;
    double reward_Lat = 0;
    while(m<64)
    {
        reward_Tht = reward_Tht + floor(std::min(0.0,(SingleStacur[m] - D_bound)))/(N_total * D_bound);
        reward_Lat = reward_Lat + floor(std::min(0.0,(T_bound - SingleStaDelay[m])))/(N_total * T_bound); 
        m++;
    }
    if(reward_Tht != 0)         // 若没有低于吞吐的STA，则全局奖励为0
    {
        reward_Tht = reward_Tht -1;
    }
    if(reward_Lat != 0)         // 若没有低于时延的STA，则全局奖励为0
    {
        reward_Lat = reward_Lat -1;
    }
    // std::cerr << "STA id为" << id << "全局reward_Tht为" << reward_Tht  <<  std::endl;
    // std::cerr << "STA id为" << id << "全局reward_Lat为" << reward_Lat  <<  std::endl;
    return reward_Lat;
}

void RLIdealWifiManager::up_train(double cur)
{
      STAtrain(cur,this->id);
}

void RLIdealWifiManager::down_train(double cur)
{
    
}

void RLIdealWifiManager::up_eval(double cur)
{
     STAtrain(cur,this->id);
}

void RLIdealWifiManager::down_eval(double cur)
{
}

void RLIdealWifiManager::Zero()
{
    total_packetsnumber[this->id] = 0;//成功发送的总的包数量
    total_successpacketsnumber[this->id] = 0;//成功接收的总的包数量
    total_failpacketsnumber[this->id] = 0;//失败的总的包数量
    total_recievepacketsnumber[this->id] = 0;//收到的所有的包的数量
    total_snr[this->id] = 0;  //成功接收的总的包累计的snr总值


}

void RLIdealWifiManager::action_length_ap(int a, int type_id)
{           
            if(a == 0)    // 减小聚合帧大小
            { 
                Mpdulength[type_id] = Mpdulength[type_id] - changelength;
            }
            else if (a == 1)
            {
                Mpdulength[type_id] = Mpdulength[type_id] + changelength;
            }
            // else  // 增大聚合帧大小
            // {
            //     Mpdulength[type_id] = Mpdulength[type_id] + 1;
            // }


            // if(Mpdulength[type_id] < min_Ampdusiz|| Mpdulength[type_id] >max_Ampdusiz)
            // {
            //     APdown_rewardFlag = true;
            // }
            // else
            // {
            //     APdown_rewardFlag = false;
            // }

            if(type_id==0)
            {
              Mpdulength[type_id] = std::min(max_acAmpdusiz, std::max(Mpdulength[type_id], min_acAmpdusiz));
            }
            else
            {
              Mpdulength[type_id] = std::min(max_axAmpdusiz, std::max(Mpdulength[type_id], min_axAmpdusiz));
            }
            int i = 1;
            while(i<=64)
            {   
                if(sta_type[i] == type_id)
                {
                    Ampdusize[i] = Mpdulength[type_id];
                }    
                //std::cerr << "STA id为" << id << "Ampdusize[i]为" << Ampdusize[i]  <<  std::endl;
                i++;
            }
            std::cerr << "Mpdulength[type_id]为" << Mpdulength[type_id]  <<  std::endl;     
            
    }
    
    //std::cerr << Simulator::Now().GetMilliSeconds() << " VI_MaxAmpduSize为" << Ampdusize[sta_id] <<  std::endl; 


void RLIdealWifiManager::action_length_sta()
{
    if(Mpdulength_act[sta_type[this->id]] == 0)    // 减小聚合帧大小
    { 
        sta_Ampdusize = sta_Ampdusize - changelength;
    }
    else //if (Mpdulength_act[sta_type[this->id]] == 1)
    {
        sta_Ampdusize = sta_Ampdusize + changelength;
    }
    
    if(GetHeSupported())
    {   
        if(sta_Ampdusize < min_axAmpdusiz|| sta_Ampdusize >max_axAmpdusiz)
        {
            APup_rewardFlag = true;
        }
        sta_Ampdusize = std::min(max_axAmpdusiz, std::max(sta_Ampdusize, min_axAmpdusiz));
    }
    else
    {   
        if(sta_Ampdusize < min_acAmpdusiz|| sta_Ampdusize >max_acAmpdusiz)
        {
            APup_rewardFlag = true;
        }
        sta_Ampdusize = std::min(max_acAmpdusiz, std::max(sta_Ampdusize, min_acAmpdusiz));
    }
    STAMpdulength[sta_type[this->id]] = sta_Ampdusize;
    //this->GetMac()->SetAttribute("VI_MaxAmpduSize", UintegerValue(sta_Ampdusize * 1536)); // 122879 at most 80 MPDUs for an A-MPDU
    //this->GetMac()->SetAttribute("VO_MaxAmpduSize", UintegerValue(sta_Ampdusize * 1536)); // 122879 at most 80 MPDUs for an A-MPDU
    //this->GetMac()->SetAttribute("BE_MaxAmpduSize", UintegerValue(sta_Ampdusize * 1536)); // 122879 at most 80 MPDUs for an A-MPDU
    //this->GetMac()->SetAttribute("BK_MaxAmpduSize", UintegerValue(sta_Ampdusize * 1536)); // 122879 at most 80 MPDUs for an A-MPDU
    Mac48Address apAddr;
    apAddr.CopyFrom(apMacAddr);
    this->SetMaxAmpduSize(apAddr,sta_Ampdusize);
    //std::cerr << Simulator::Now().GetMilliSeconds() << " BE_MaxAmpduSize为" << sta_Ampdusize <<  std::endl;
}

void RLIdealWifiManager::action_cw_ap(int act, int type_id)
{   
    
    if (act == 0)
    {
        CWsize[type_id] = (CWsize[type_id] + 1) / 2 - 1;
    }
    else if (act == 1)
    {
        CWsize[type_id] = CWsize[type_id] ;
    }
    else if (act == 2)
    {
        CWsize[type_id] = (CWsize[type_id] + 1) * 2 - 1 ;
    }
    else
    {
        std::cout << "Unsupported agent type!" << std::endl;
        exit(0);
    }
    // if(CWsize[type_id] < min_cw|| CWsize[type_id] >max_cw)
    // {
    //     APdown_rewardFlag = true;
    // }
    // else
    // {
    //     APdown_rewardFlag = false;
    // }
    if(sta_EDCA[this->id] == 0)   //BE业务
    {
      CWsize[type_id] = std::min(max_becw, std::max(CW, min_becw));
    }
    else
    {
      CWsize[type_id] = std::min(max_vicw, std::max(CW, min_vicw));
    }

    std::ostringstream oss,oss1;
    oss << "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/VI_Txop/MinCw";
    oss1 << "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/VI_Txop/MaxCw";
    Config::Set(oss.str(), UintegerValue(CWsize[type_id]));
    Config::Set(oss1.str(), UintegerValue(CWsize[type_id]));
    oss << "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/MinCw";
    oss1 << "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/MaxCw";
    Config::Set(oss.str(), UintegerValue(CWsize[type_id]));
    Config::Set(oss1.str(), UintegerValue(CWsize[type_id]));
    oss << "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BK_Txop/MinCw";
    oss1 << "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BK_Txop/MaxCw";
    Config::Set(oss.str(), UintegerValue(CWsize[type_id]));
    Config::Set(oss1.str(), UintegerValue(CWsize[type_id]));
    oss << "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/VO_Txop/MinCw";
    oss1 << "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/VO_Txop/MaxCw";
    Config::Set(oss.str(), UintegerValue(CWsize[type_id]));
    Config::Set(oss1.str(), UintegerValue(CWsize[type_id]));
    std::cerr << "CWsize[type_id]为" << CWsize[type_id]  <<  std::endl; 
   //////////
}

void RLIdealWifiManager::action_cw_sta()
{   
     if (CWsize_act[sta_type[this->id]] == 0)
    {
        CW = floor(last_CW/1.5) ;
        
    }
    else if (CWsize_act[sta_type[this->id]] == 1)
    {
        CW = floor(last_CW*1.5);
    }
    else
    {
        std::cout << "Unsupported agent type!" << std::endl;
        exit(0);
    }

    if(sta_EDCA[this->id] == 0)   //BE业务
    { 
        if(CW < min_becw|| CW >max_becw)
            {
                APup_rewardFlag = true;
            }
        CW = std::min(max_becw, std::max(CW, min_becw));
    }
    else
    {
        if(CW < min_vicw|| CW >max_vicw)
        {
            APup_rewardFlag = true;
        }
        CW = std::min(max_vicw, std::max(CW, min_vicw));
    }
    STACWsize[sta_type[this->id]] = CW;
   // std::cout << "准备改变的CW值\t" << CW << " ;" << std::endl;
    if(GetHeSupported())
    {   
        if(dlAckSeqType == "NO-OFDMA")
        {
            std::ostringstream oss, oss2, oss3;
            oss << "/NodeList/" << id;
            oss2 << oss.str() << "/DeviceList/*/$ns3::WifiNetDevice/Mac/VI_Txop/MinCw";
            Config::Set(oss2.str(), UintegerValue(CW));
            oss3 << oss.str() << "/DeviceList/*/$ns3::WifiNetDevice/Mac/VI_Txop/MaxCw";
            Config::Set(oss3.str(), UintegerValue(CW));
            oss2 << oss.str() << "/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/MinCw";
            Config::Set(oss2.str(), UintegerValue(CW));
            oss3 << oss.str() << "/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/MaxCw";
            Config::Set(oss3.str(), UintegerValue(CW));
        }
        else
        {
            std::ostringstream oss, oss2, oss3;
            oss << "/NodeList/" << id;
            oss2 << oss.str() << "/DeviceList/*/$ns3::WifiNetDevice//HeConfiguration/MuBeCwMin";
            Config::Set(oss2.str(), UintegerValue(CW));
            oss3 << oss.str() << "/DeviceList/*/$ns3::WifiNetDevice//HeConfiguration/MuBeCwMax";
            Config::Set(oss3.str(), UintegerValue(CW));
            oss2 << oss.str() << "/DeviceList/*/$ns3::WifiNetDevice//HeConfiguration/MuViCwMin";
            Config::Set(oss2.str(), UintegerValue(CW));
            oss3 << oss.str() << "/DeviceList/*/$ns3::WifiNetDevice//HeConfiguration/MuViCwMax";
            Config::Set(oss3.str(), UintegerValue(CW));
        }
        last_CW=CW;
    }
    else
    {
        std::ostringstream oss, oss2, oss3;
        oss << "/NodeList/" << id;
        oss2 << oss.str() << "/DeviceList/*/$ns3::WifiNetDevice/Mac/VI_Txop/MinCw";
        Config::Set(oss2.str(), UintegerValue(CW));
        oss3 << oss.str() << "/DeviceList/*/$ns3::WifiNetDevice/Mac/VI_Txop/MaxCw";
        Config::Set(oss3.str(), UintegerValue(CW));
        oss2 << oss.str() << "/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/MinCw";
        Config::Set(oss2.str(), UintegerValue(CW));
        oss3 << oss.str() << "/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/MaxCw";
        Config::Set(oss3.str(), UintegerValue(CW));
        last_CW=CW;
    }
    //std::cerr << Simulator::Now().GetMilliSeconds() << " CW为" << CW <<  std::endl; 
}

void
RLIdealWifiManager::ScheduleNextStateRead (uint16_t id,double maxThroughput,double cur)
{  
    if(this->id == 0) //AP
    {
      error[this->id] = error_radio;
      //APdowntrain(cur);
      APuptrain(cur);
    }
    else //STA
    {
      //classify_ap();
      STAtrain(cur,this->id);
    }
    
    NS_LOG_FUNCTION(this);
}

void
RLIdealWifiManager::STAtrain(double cur,int sta_id)
{   
    if(total_recievepacketsnumber[this->id]==0)
    {
        average_snr[this->id] = 0;
    }
    else
    {
        average_snr[this->id] = 10*log10(total_snr[this->id]/total_recievepacketsnumber[this->id]);
    }   
    error[this->id] = sta_error_radio[this->id-1];
    //std::cerr << " id为 " << id << " average_snr: " << average_snr[this->id] << std::endl; 
    //std::cerr << " id为 " << id << " error[this->id]: " << error[this->id] << std::endl; 
    if(sta_begin_action)
    {
        action_length_sta();
        action_cw_sta();
    }
    /////////////////
    /////参数归零/////
    ////////////////
    Zero();
    sta_snr[sta_id] = 0;
    sta_recievepacketsnumber[sta_id] = 0;
}

void
RLIdealWifiManager::APdowntrain(double cur)
{   
    Time now = Simulator::Now();      
    auto env =  m_ns3ai_mod->EnvSetterCond();     ///< Acquire the Env memory for writing
    double net_snr = 0;
    double error = 0;
    double number = 0;
    int i = 0;
    int j = 1;
    //std::cerr << " APdowntrain " <<  std::endl; 
    while(i<Type)
    {       
            while(j<=64)
            {
                if(sta_type[j]==i)
            {
                net_snr = net_snr + snr[j-1];
                error = error + net_error_radio[j-1];
                number++;
            }
                j++;
            }
                env->snr[i] = net_snr*1.0/number;
                env->per[i] = error*1.0/number;
                env->t_idle[i] = t_idle;
                env->cw_last[i] = CWsize[i];
                env->length_last[i] = Mpdulength[i];
                // std::cerr << " env->snr[i]: " << env->snr[i] << std::endl; 
                // std::cerr << " env->per[i]: " << env->per[i] << std::endl;
                // std::cerr << " t_idle[i]: " << env->t_idle[i] << std::endl;
                // std::cerr << " env->cw_last[i]: " << env->cw_last[i] << std::endl;
                // std::cerr << " env->length_last[i]: " << env->length_last[i] << std::endl;
                net_snr = 0;
                error = 0;
                number = 0;
                
            i++;
    }    
    ///////////////////
    /////奖励计算/////
    //////////////////
    if(APdown_rewardFlag == true)
    {
        env->reward = -1;
    }
    else
    { 
     //   env->reward = compute_localreward(1);                //VI业务的奖励计算
      env->reward = compute_localreward(0);                //BE业务的奖励计算
     }
    std::cerr << now.GetSeconds() << " env->reward: " << env->reward << std::endl;
    //////////////过滤判断///////
    if(Last_dqn_average_Lat==0)
    {
        env->filter_flag = false;
        std::cerr << " false!"  <<  std::endl;

    }
    else
    {
        env->filter_flag = true;
    }
    
    //std::cerr << Simulator::Now().GetMilliSeconds() << " snr[M]为" << env->snr[M] <<  std::endl;
    //std::cerr << Simulator::Now().GetMilliSeconds() << " per[M]为" << error_radio_M[M] <<  std::endl;
   
    env->throughput = Throughput_1s;
    env->delay = Delay_1s;
    env->cur = cur; 
    m_ns3ai_mod->SetCompleted();                 ///< Release the memory and update conters
    
    /////////////////
    /////act读取/////
    ////////////////

    auto act =  m_ns3ai_mod->ActionGetterCond();  ///< Acquire the Act memory for reading
    i = 0;
    std::cerr << " act->length[i]为" << act->length[i] <<  std::endl;
    std::cerr << " act->cw[i]为" << act->cw[i] <<  std::endl;
    APdown_rewardFlag = false;
    while(i<Type)
    { 
      action_length_ap(act->length[i],i);
      action_cw_ap(act->cw[i],i);
      i++;
    }

    /////////////////
    /////参数归零/////
    ////////////////

    Zero();

    //关闭内存池
    m_ns3ai_mod->GetCompleted();                 ///< Release the memory, roll back memory version and update conters
    
}

void
RLIdealWifiManager::APuptrain(double cur)
{   
    auto env =  m_ns3ai_mod->EnvSetterCond();     ///< Acquire the Env memory for writing
    double net_snr = 0;
    double error = 0;
    double number = 0;
    int i = 0;
    int j = 1;
    while(i<Type)
    {       
            j = 1;
            while(j<=64)
            {
                if(sta_type[j]==i)
            {
                net_snr = net_snr + snr[j-1];
                error = error + net_error_radio[j-1];
                number++;
            }
                j++;
            }   
                // std::cerr << " type: " << i << " number: " << number << std::endl; 
                env->snr[i] = net_snr*1.0/number;
                env->per[i] = error*1.0/number;
                env->t_idle[i] = t_idle;
                //env->cw_last[i] = STACWsize[i];
                env->cw_last[i] = STACWsize[i];             //分类类型为协议，但cw调节按照业务进行分组，即两个组cw调节都是一致的。0->be 1->vi
                env->length_last[i] = STAMpdulength[i];
                // std::cerr << " env->snr[i]: " << env->snr[i] << std::endl; 
                // std::cerr << " env->per[i]: " << env->per[i] << std::endl;
                // std::cerr << " t_idle[i]: " << env->t_idle[i] << std::endl;
                std::cerr << " env->cw_last[i]: " << env->cw_last[i] << std::endl;
                std::cerr << " env->length_last[i]: " << env->length_last[i] << std::endl;
                net_snr = 0;
                error = 0;
                number = 0;
                
            i++;
    }    
    ///////////////////
    /////奖励计算/////
    //////////////////
    if(APup_rewardFlag == true)
    {
        env->reward = -1;
    }
    else
    { 
    //    env->reward = compute_localreward(1);                //VI业务的奖励计算
          env->reward = compute_localreward(0);                //BE业务的奖励计算
    }
    //////////////过滤判断///////
    if(Last_dqn_average_Lat==0)
    {
        env->filter_flag = false;
        std::cerr << Simulator::Now().GetMilliSeconds() << " false!"  <<  std::endl;
    }
    else
    {
        env->filter_flag = true;
    }
    
    //std::cerr << Simulator::Now().GetMilliSeconds() << " snr[M]为" << env->snr[M] <<  std::endl;
    //std::cerr << Simulator::Now().GetMilliSeconds() << " per[M]为" << error_radio_M[M] <<  std::endl;
   
    env->throughput = Throughput_1s;
    env->delay = Delay_1s;
    env->cur = cur; 
    m_ns3ai_mod->SetCompleted();                 ///< Release the memory and update conters
    
    /////////////////
    /////act读取/////
    ////////////////

    auto act =  m_ns3ai_mod->ActionGetterCond();  ///< Acquire the Act memory for reading
    i = 0;
    while(i<Type)
    {     
        Mpdulength_act[i]  = act->length[i];
        CWsize_act[i] = act->cw[i];
        std::cerr << " act->length[i]为" << act->length[i] <<  std::endl;
        std::cerr << " act->cw[i]为" << act->cw[i] <<  std::endl;
        i++;
    }

    /////////////////
    /////参数归零/////
    ////////////////

    Zero();

    //关闭内存池
    m_ns3ai_mod->GetCompleted();                 ///< Release the memory, roll back memory version and update conters
    
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
    if(this->id == 0)
    {
        uint8_t buffer[6];
        station->m_state->m_address.CopyTo(buffer);
        int nSta = int(buffer[5]); 
        snr[nSta-2] = rxSnr;
    }

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
    total_successpacketsnumber[id] = total_successpacketsnumber[id] + nSuccessfulMpdus;   //各设备统计自身的总的数目
    total_failpacketsnumber[id] = total_failpacketsnumber[id] + nFailedMpdus;
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
    total_packetsnumber[this->id]++;   //自身发的总的包数目
    ////AP端设置聚合帧的方法？
    // if(this->id == 0)
    // {
    //     uint8_t buffer[6];
    //     st->m_state->m_address.CopyTo(buffer);
    //     int nSta = int(buffer[5]); 
    //     // this->GetMac()->SetAttribute("VI_MaxAmpduSize", UintegerValue(Ampdusize[nSta-1])); // 122879 at most 80 MPDUs for an A-MPDU
    //     // this->GetMac()->SetAttribute("VO_MaxAmpduSize", UintegerValue(Ampdusize[nSta-1])); // 122879 at most 80 MPDUs for an A-MPDU
    //     // this->GetMac()->SetAttribute("BE_MaxAmpduSize", UintegerValue(Ampdusize[nSta-1])); // 122879 at most 80 MPDUs for an A-MPDU
    //     // this->GetMac()->SetAttribute("BK_MaxAmpduSize", UintegerValue(Ampdusize[nSta-1])); // 122879 at most 80 MPDUs for an A-MPDU 
    //     this->SetMaxAmpduSize(this->GetAddress(st),Ampdusize[nSta-1]);
    // }
    // else
    // {
    //     this->GetMac()->SetAttribute("VI_MaxAmpduSize", UintegerValue(sta_Ampdusize)); // 122879 at most 80 MPDUs for an A-MPDU
    //     this->GetMac()->SetAttribute("VO_MaxAmpduSize", UintegerValue(sta_Ampdusize)); // 122879 at most 80 MPDUs for an A-MPDU
    //     this->GetMac()->SetAttribute("BE_MaxAmpduSize", UintegerValue(sta_Ampdusize)); // 122879 at most 80 MPDUs for an A-MPDU
    //     this->GetMac()->SetAttribute("BK_MaxAmpduSize", UintegerValue(sta_Ampdusize)); // 122879 at most 80 MPDUs for an A-MPDU 
    
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

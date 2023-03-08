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

#ifndef RL_IDEAL_WIFI_MANAGER_H
#define RL_IDEAL_WIFI_MANAGER_H



#include "ns3/traced-value.h"
#include "ns3/wifi-remote-station-manager.h"
#include "ns3/ns3-ai-module.h"

const int M = 5;
const int Type = 2;
extern uint16_t m_ns3ai_id;
extern std::string dlAckSeqType{"NO-OFDMA"};
extern bool APdown_rewardFlag{false};
extern bool APup_rewardFlag{false};

extern double error_radio = 0;       //AP总的丢包率
extern double t_idle = 0;


extern double totalThroughput = 0;
extern double Throughput_1s = 0;
extern double Delay_1s = 0;

extern bool flag = false;
extern bool sta_begin_action = false;
extern double SingleStacur[64]{};
extern double SingleStaDelay[64]{};
extern double sta_error_radio[64]{};//定义于h文件中 STA的总丢包率
extern double ap_error_radio[64]{}; //定义于h文件中
extern double net_error_radio[64]{};//统计了sta和ap都作为发送端的误码率参数
extern int Ampdusize[65]{};  // 80 聚合帧 由AP维护的发往不同STA的聚合帧长度设置
extern double snr[64]{};


//extern int nApMacQueue;


namespace ns3
{

struct AiRLLENGTHEnv
{
  double snr[Type];
  double per[Type];
  double t_idle[Type];
  int cw_last[Type];
  int length_last[Type];
  int type_id;
  int sta_id;
  double bytes;
  bool filter_flag;
  double reward;
  double throughput;
  double cur;
  double delay;
} Packed;

struct AiRLLENGTHAct
{
  int length[Type];
  int cw[Type];
} Packed;

    Ns3AIRL<AiRLLENGTHEnv, AiRLLENGTHAct> * m_ns3ai_mod;

    static double total_packetsnumber[65]{};//发送的总的包数量
    static double total_successpacketsnumber[65]{};//发送成功的总的包数量
    static double total_failpacketsnumber[65]{};//发送失败的总的包数量
    
    static double total_snr[65]{};  //成功接收的总的包累计的snr总值
    static double total_recievepacketsnumber[65]{};//接收的总的包数量
    static double average_snr[65]{};//snr的平均值
    
    static double apsnr_M[65][M+1]{};          //AP存储的snr（存储了M个历史长度）
    static double sta_snr[65]{};               //AP自身统计存储的不同STA的总snr值（存储了M个历史长度）
    static double sta_recievepacketsnumber[65]{};//接收的总的包数量

    static double error[65]{}; //每个STA/AP的丢包率
    static int sta_type[65];   //STA的归类(协议分组)
    static int sta_EDCA[65];   //STA的归类(业务分组)0:BE 1:VI

    static int Mpdulength[Type];      //用于AP端进行聚合帧调节的具体大小（1-10）
    static int Mpdulength_act[Type];  //用于STA端进行聚合帧调节的动作参数（0，1，2）
    static int STAMpdulength[Type];   //用于STA端进行聚合帧调节的具体大小（1-10）

    static int CWsize[Type];          //用于AP端进行CW调节的具体大小（15-1023）
    static int CWsize_act[Type];      //用于STA端进行 CW 调节的动作参数（0，1，2）
    static int STACWsize[Type];       //用于STA端进行CW调节的具体大小（15-1023）
    static uint16_t device_id = 0;    //STA数目

struct RLIdealWifiRemoteStation;

/**
 * \brief Ideal rate control algorithm
 * \ingroup wifi
 *
 * This class implements an 'ideal' rate control algorithm
 * similar to RBAR in spirit (see <i>A rate-adaptive MAC
 * protocol for multihop wireless networks</i> by G. Holland,
 * N. Vaidya, and P. Bahl.): every station keeps track of the
 * SNR of every packet received and sends back this SNR to the
 * original transmitter by an out-of-band mechanism. Each
 * transmitter keeps track of the last SNR sent back by a receiver
 * and uses it to pick a transmission mode based on a set
 * of SNR thresholds built from a target BER and transmission
 * mode-specific SNR/BER curves.
 */
class RLIdealWifiManager : public WifiRemoteStationManager
{
  public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();
    RLIdealWifiManager();
    ~RLIdealWifiManager() override;
    void ScheduleNextStateRead (uint16_t id,double maxThroughput,double cur) override;
    void APdowntrain (double cur);
    void APuptrain (double cur);
    void STAtrain (double cur, int sta_id);

    void SetupPhy(const Ptr<WifiPhy> phy) override;
    void Zero ();              //参数归零
    void action_length_ap(int a,int type_id);     //根据动作参数选择聚合帧长度
    void action_length_sta();     //根据动作参数选择聚合帧长度
    void action_cw_ap(int act,int type_id);     //根据动作参数选择cw
    void action_cw_sta();     //根据动作参数选择cw
    void classify_ap();            //对不同的id的STA进行归类
    double compute_localreward(int EDCA);     //计算局部奖励
    double compute_globalreward();    //计算全局奖励
    void up_train(double cur);      //上行传输训练
    void down_train(double cur);    //下行传输训练
    void up_eval(double cur);      //上行传输评估
    void down_eval(double cur);    //下行传输评估

  private:
    void DoInitialize() override;
    WifiRemoteStation* DoCreateStation() const override;
    void DoReportRxOk(WifiRemoteStation* station, double rxSnr, WifiMode txMode) override;
    void DoReportRtsFailed(WifiRemoteStation* station) override;
    void DoReportDataFailed(WifiRemoteStation* station) override;
    void DoReportRtsOk(WifiRemoteStation* station,
                       double ctsSnr,
                       WifiMode ctsMode,
                       double rtsSnr) override;
    void DoReportDataOk(WifiRemoteStation* station,
                        double ackSnr,
                        WifiMode ackMode,
                        double dataSnr,
                        uint16_t dataChannelWidth,
                        uint8_t dataNss) override;
    void DoReportAmpduTxStatus(WifiRemoteStation* station,
                               uint16_t nSuccessfulMpdus,
                               uint16_t nFailedMpdus,
                               double rxSnr,
                               double dataSnr,
                               uint16_t dataChannelWidth,
                               uint8_t dataNss) override;
    void DoReportFinalRtsFailed(WifiRemoteStation* station) override;
    void DoReportFinalDataFailed(WifiRemoteStation* station) override;
    WifiTxVector DoGetDataTxVector(WifiRemoteStation* station, uint16_t allowedWidth) override;
    WifiTxVector DoGetRtsTxVector(WifiRemoteStation* station) override;

    /**
     * Reset the station, invoked if the maximum amount of retries has failed.
     *
     * \param station the station for which statistics should be reset
     */
    void Reset(WifiRemoteStation* station) const;

    /**
     * Construct the vector of minimum SNRs needed to successfully transmit for
     * all possible combinations (rate, channel width, nss) based on PHY capabilities.
     * This is called at initialization and if PHY capabilities changed.
     */
    void BuildSnrThresholds();

    /**
     * Return the minimum SNR needed to successfully transmit
     * data with this WifiTxVector at the specified BER.
     *
     * \param txVector WifiTxVector (containing valid mode, width, and Nss)
     *
     * \return the minimum SNR for the given WifiTxVector in linear scale
     */
    double GetSnrThreshold(WifiTxVector txVector);
    /**
     * Adds a pair of WifiTxVector and the minimum SNR for that given vector
     * to the list.
     *
     * \param txVector the WifiTxVector storing mode, channel width, and Nss
     * \param snr the minimum SNR for the given txVector in linear scale
     */
    void AddSnrThreshold(WifiTxVector txVector, double snr);

    /**
     * Convenience function for selecting a channel width for non-HT mode
     * \param mode non-HT WifiMode
     * \return the channel width (MHz) for the selected mode
     */
    uint16_t GetChannelWidthForNonHtMode(WifiMode mode) const;

    /**
     * Convenience function to get the last observed SNR from a given station for a given channel
     * width and a given NSS. Since the previously received SNR information might be related to a
     * different channel width than the requested one, and/or a different NSS,  the function does
     * some computations to get the corresponding SNR.
     *
     * \param station the station being queried
     * \param channelWidth the channel width (in MHz)
     * \param nss the number of spatial streams
     * \return the SNR in linear scale
     */
    double GetLastObservedSnr(RLIdealWifiRemoteStation* station,
                              uint16_t channelWidth,
                              uint8_t nss) const;

    /**
     * A vector of <snr, WifiTxVector> pair holding the minimum SNR for the
     * WifiTxVector
     */
    typedef std::vector<std::pair<double, WifiTxVector>> Thresholds;

    double m_ber;            //!< The maximum Bit Error Rate acceptable at any transmission mode
    Thresholds m_thresholds; //!< List of WifiTxVector and the minimum SNR pair

    TracedValue<uint64_t> m_currentRate; //!< Trace rate changes

    
    bool m_started{false};
    uint16_t id;//sta和ap的标识，具体看设置速率算法的先后顺序 场景2:ap为0 sta为1-64
    int TypeID; //自己维持一个自己分组的类型
    uint16_t sumsta;//sta的总数
    //Ns3AIRL<AiRLLENGTHEnv, AiRLLENGTHAct> * m_ns3ai_mod;
    //int sta_Ampdusize = 122879;  // 80 聚合帧 STA自行维护的发往AP的聚合帧设置
    int sta_Ampdusize;       // 40 聚合帧 STA自行维护的发往AP的聚合帧设置
    int min_acAmpdusiz = 4;
    int max_acAmpdusiz = 64;
    int min_axAmpdusiz = 10;
    int max_axAmpdusiz = 80;
    int changelength = 5;   //每次length的调节大小
    int last_CW = 15;          // STA自身的上一次的CW值
    int CW = 15;               // STA自身的CW值
    int min_becw = 15;
    int max_becw = 1023;
    int min_vicw = 7;
    int max_vicw = 15;
    
    ////////历史存储//////////
    double error_radio_M[65][M+1]{};    //AP存储的丢包率（存储了M个历史长度）
    //double apsnr_M[65][M+1]{};          //AP存储的snr（存储了M个历史长度）
    double ap_tidle_M[65][M+1]{};       //AP存储的t_idle（存储了M个历史长度）
    double sta_error_radio_M[M+1]{};    //sta存储的丢包率（存储了M个历史长度）
    double stasnr_M[M+1]{};             //sta存储的snr（存储了M个历史长度）
    double sta_tidle_M[M+1]{};          //sta存储的t_idle（存储了M个历史长度）
    
    //////奖励计算////////
    ///////仅AP端计算//////
    double dqn_total_Tht = 0 ;   //某个dqn网络的用户总吞吐量
    double Last_dqn_total_Tht = 0 ;   
    double dqn_average_Tht = 0 ; //某个dqn网络的平均用户吞吐量
    double slide_dqn_total_Tht = 0 ;   //某个dqn网络的用户总吞吐量
    double Last_slide_dqn_total_Tht = 0 ;   

    double dqn_total_Lat = 0 ;   //某个dqn网络的总时延
    double dqn_average_Lat = 0 ; //某个dqn网络的平均时延
    double Last_dqn_average_Lat = 0 ; 
    double slide_dqn_average_Lat = 0 ; //某个dqn网络的平均时延
    double Last_slide_dqn_average_Lat = 0 ; 
   

};

} // namespace ns3

#endif /* RL_IDEAL_WIFI_MANAGER_H */

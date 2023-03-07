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

const int M = 2;
extern double error_radio = 0;       //AP总的丢包率
extern double t_idle = 0;


extern double totalThroughput = 0;
extern bool flag = false;
extern double SingleStacur[64]{};
extern double SingleStaTht[64]{};
extern double sta_error_radio[64]{};//定义于h文件中 STA的总丢包率
extern double ap_error_radio[64]{}; //定义于h文件中
extern double snr[64]{};
extern int ac_initial_framelength = 14*1536;
extern int ax_initial_framelength = 10*1536;
extern int sta_Ampdusize[2]{ac_initial_framelength,ax_initial_framelength};
extern int last_Ampdusize[2]{ac_initial_framelength,ax_initial_framelength};
extern double group_error_radio[2]{};
extern double group_error_radio_last[2]{};
extern double group_snr[2]{};
extern double group_per[2]{};
extern double group_snr_last[2]{};
extern double group_per_last[2]{};
extern double throughputBytes=1;
extern double last_EMA_Tht=1;
extern bool initial_flag = true;

extern std::map<ns3::Mac48Address /* recepient */, int /* AmpduSize */> AddrActMap({}); //接收地址和对应动作



extern int nApMacQueue;


namespace ns3
{

struct AiRLLENGTHEnv
{
  double snr[M];
  double per[M];
  double t_idle;
  int cw_last[M];
  int length_last[M];
  double reward;
  double throughput;
  int bound_num;
} Packed;

struct AiRLLENGTHAct
{
  int length[2];
  int cw[2];
} Packed;

Ns3AIRL<AiRLLENGTHEnv, AiRLLENGTHAct>* m_ns3ai_mod;

static double error[65]{};     // 每个STA/AP的丢包率
static int sta_type[65];       // STA的归类
static uint16_t device_id = 0; // STA数目

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
    void STAtrain (double cur, int sta_id);
    void SetupPhy(const Ptr<WifiPhy> phy) override;
    void Zero ();              //参数归零
    void action_length_sta1(int a);     //根据动作参数选择聚合帧长度
    void action_length_sta2(int a);     //根据动作参数选择聚合帧长度
    void action_cw_sta1(int a);     //根据动作参数选择cw
    void action_cw_sta2(int a);
    void classify_ap();   //根据 snr 对不同的id的STA进行归类
    double compute_localreward(int sta_id);     //计算局部奖励
    double compute_globalreward();    //计算全局奖励
    bool m_started{false};
    uint16_t m_ns3ai_id = 2321;
    uint16_t id;//sta和ap的标识，具体看设置速率算法的先后顺序 场景2:ap为0 sta为1-64
    uint16_t sumsta;//sta的总数
    //Ns3AIRL<AiRLLENGTHEnv, AiRLLENGTHAct> * m_ns3ai_mod;
    int length[65]; //AP保存的发往每一个STA的聚合帧长度的动作参数
  
    double last_CW [2]= {15,15};
    double CW[2] = {15,15};
    bool CrosslineFlag1 = false;
    bool CrosslineFlag2 = false;
    bool CrosslineFlag3 = false;

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
};

} // namespace ns3

#endif /* RL_IDEAL_WIFI_MANAGER_H */

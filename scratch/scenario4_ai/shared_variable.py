#python参数
M = 5        #历史长度
N = 4 * 2      #总的参数个数
Actnum = 1   #动作个数
Output = 8   #网络输出的结果个数
type_num = 2  #分组的个数
train = 2    #是否训练：1为训练 2：为评估
trainepoch = 4
simepoch = 1

#ns3端的参数
timestep=200                  #ns3与python交互 的时间隙度  单位毫秒
timestep_large = 1000         #ns3多久计算一次吞吐量       单位秒
simulationTime = 20          #一次ns3的仿真时间
RngRun = 1                    #随机种子
dlAckType = 'NO-OFDMA'        #调度方式，OFDMA->AGGR-MU-BAR 、NO-OFDMA
#共同的参数
memblock_key = 2333

#文件操作
import os
filename=os.path.abspath(__file__)
f=filename.split('/')[-2]
print(f)

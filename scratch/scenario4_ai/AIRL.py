from ctypes import *
from py_interface import *
import os
import torch
import argparse
import numpy as np
import torch.nn as nn
import matplotlib.pyplot as plt
from dqn_net import DQN_1
import shared_variable
import random

#np.transpose

class AiRLLENGTHEnv(Structure):
    _pack_ = 1
    _fields_ = [
        ('snr', c_double * shared_variable.type_num),
        ('per', c_double * shared_variable.type_num),
        ('t_idle', c_double * shared_variable.type_num),
        ('cw_last', c_int* shared_variable.type_num),
        ('length_last', c_int* shared_variable.type_num),
        ('type_id', c_int),
        ('sta_id', c_int),
        ('bytes', c_double),
        ('filter_flag',c_bool),
        ('reward', c_double),
        ('throughput', c_double),  # 用于传过来画图
        ('cur', c_double),         # 用于传过来画图 小间隙吞吐
        ('delay', c_double)
    ]


class AiRLLENGTHAct(Structure):
    _pack_ = 1
    _fields_ = [
        ('length', c_int * shared_variable.type_num),
        ('cw', c_int * shared_variable.type_num)
    ]

class AiRLContainer:
    use_ns3ai = True

    def __init__(self, uid: int = 2333) -> None:
        #self.rl = Ns3AIRL(uid, AiRLRateEnv, AiRLRateAct)
        # print('({})size: Env {} Act {}'.format(uid, sizeof(AiConstantRateEnv), sizeof(AiConstantRateAct)))
        self.num = int(shared_variable.N/shared_variable.type_num)
        self.c = np.zeros((shared_variable.type_num,self.num))
        # self.s = np.zeros(type_num * N)
        # self.a = np.zeros(type_num * Actnum)
        # self.s_ = np.zeros(type_num * N)
        self.s = [0]
        self.a = 0
        self.s_ = [0]
        self.a_ = 0
        self.r = 0

        self.agent_1 = DQN_1()  # 初始化agent1
        # self.agent_2 = DQN_2()  # 初始化agent2
        # self.agent_3 = DQN_3()  # 初始化agent3
        self.train = True
        self.cur = []
        self.throughput_1s = []
        self.delay_1s = []

        pass

    def do(self, env: AiRLLENGTHEnv, act: AiRLLENGTHAct) -> AiRLLENGTHAct:

            ######对观测向量ct进行处理########
            i = 0
            while i < shared_variable.type_num:
                if i == 0:
                    self.c[i] = [env.per[i],env.t_idle[i],env.cw_last[i]/1024,env.length_last[i]/64]  #归一化处理处
                if i == 1:
                    self.c[i] = [env.per[i], env.t_idle[i], env.cw_last[i] / 1024, env.length_last[i] / 80]  # 归一化处理处
                i = i+1
            #################################
            ####### 处理s、a、r、s_ ###########
            #################################
            i = 0
            j = 0
            self.s_ = []
            while i < shared_variable.type_num:
                j = 0
                while j < self.num:
                      self.s_.append(self.c[i,j])
                      j = j+1
                i = i+1
            self.a_ = self.agent_1.choose_action(self.s_)
            act.length[0] = self.a_ % 4 // 2
            act.cw[0] = self.a_ // 4
            act.length[1] = self.a_ % 4 % 2
            act.cw[1] = self.a_ // 4

            # act.length[0] = self.a_ // 9 // 3
            # act.cw[0] = self.a_ // 9 % 3
            # act.length[1] = self.a_ % 9 // 3
            # act.cw[1] = self.a_ % 9 % 3
            self.r  = env.reward
            self.throughput_1s.append(env.throughput)
            self.delay_1s.append(env.delay)
            #self.cur.append(env.cur)

            #########################################
            ####### 分类DQN进行经验向量存储 ###########
            #######################################
            # print('STA ID：', env.sta_id)
            # print('观测向量C：', self.c)
            # print('状态向量S：', self.s[env.sta_id, :])
            # print('奖励向量R：', self.r)
            # print('状态向量S_：', self.s_[env.sta_id, :])

            if ai.agent_1.flag:
               #if self.s_[0]!=0 and self.s[0]!=0 and env.filter_flag:   #进行经验池的过滤:
               if self.s!=[0] and env.filter_flag:  # 进行经验池的过滤:
                     #这里要调用一下store函数 存入轨迹哦！
                      print('正在存入轨迹！')
                      print(self.s)
                      print(self.a)
                      print(self.r)
                      print(self.s_)
                      print(env.filter_flag)
                      self.agent_1.store_transition(self.s, self.a, self.r, self.s_)#轨迹存入
            # if env.type_id == 2:
            #    if flag and env.snr[0]!=0 and env.snr[1]!=0 and env.snr[2]!=0 and env.snr[3]!=0 and env.snr[4]!=0 and env.snr[5]!=0:
            #          #这里要调用一下store函数 存入轨迹哦！
            #          self.agent_2.store_transition(self.s[env.sta_id, : ], self.a[env.sta_id, : ], self.r, self.s_[env.sta_id, : ])#轨迹存入
            # if env.type_id == 3:
            #    if flag and env.snr[0]!=0 and env.snr[1]!=0 and env.snr[2]!=0 and env.snr[3]!=0 and env.snr[4]!=0 and env.snr[5]!=0:
            #          #这里要调用一下store函数 存入轨迹哦！
            #          self.agent_3.store_transition(self.s[env.sta_id, : ], self.a[env.sta_id, : ], self.r, self.s_[env.sta_id, : ])#轨迹存入
            self.s = self.s_
            self.a = self.a_



               #这里要训练 调用learn函数哦！
            if self.agent_1.memory_counter > self.agent_1.memory_capacity and ai.agent_1.train:
                self.agent_1.learn()
                #print('agent 1 is learning')
            # if self.agent_2.memory_counter > self.agent_2.memory_capacity and self.train:
            #     self.agent_2.learn()
            #     print('agent 2 is learning')
            # if self.agent_3.memory_counter > self.agent_3.memory_capacity and self.train:
            #     self.agent_3.learn()
            #     print('agent 3 is learning')

            return act

if __name__ == '__main__':

 mempool_key = 1234                                          # memory pool key, arbitrary integer large than 1000
 mem_size = 4096                                             # memory pool size in bytes
 memblock_key = shared_variable.memblock_key  # memory block key, need to keep the same in the ns-3 script
 trainepoch = shared_variable.trainepoch                                            # train number
 simepoch = shared_variable.simepoch                                                    # simulate number
 loss_num_1 = 0                                                # 用于模型收敛判断
 throughput = np.zeros(trainepoch)
 Throughput_1s = []
 throughput_sim = np.zeros(simepoch)
 stop = False

 ns3Settings1 = {'tracing': 'true', 'useRts': 'false','useExtendedBlockAck': 'true',
               'simulationTime': 5 ,'dlAckType': 'NO-OFDMA','frequency': 5,
               'channelWidth': 160 ,'distance': 1 ,'mcs': 11 ,'RngRun': 1} #元组
#AGGR-MU-BAR
 ns3Settings2 = {'useExtendedBlockAck': 'true',
                 'simulationTime': shared_variable.simulationTime,
                 'dlAckType': shared_variable.dlAckType, 'RngRun': shared_variable.RngRun ,
                 'm_ns3ai_id':shared_variable.memblock_key,
                 'timestep':shared_variable.timestep,'timestep_large':shared_variable.timestep_large}  # 元组

 ai = AiRLContainer(memblock_key)
 cwlist = []
 cwlist.append([])
 cwlist.append([])
 cw_netact = []
 cw_netact.append([])
 cw_netact.append([])
 lenlist = []
 lenlist.append([])
 lenlist.append([])
 len_netact = []
 len_netact.append([])
 len_netact.append([])
# ai.agent_1.memory = np.loadtxt("experience.txt", delimiter=' ')
# ai.agent_1.memory_counter = 500
# ai.agent_1.loss = np.loadtxt("loss.txt", delimiter=' ')
# ai.throughput_1s = np.loadtxt("throughput.txt", delimiter=' ')
# lenlist = np.loadtxt("len.txt", delimiter=' ')
# cwlist = np.loadtxt("cw.txt", delimiter=' ')
# ai.agent_1.load_models(2)
 exp = Experiment(mempool_key, mem_size, 'scratch/'+shared_variable.f+'/scenario4_cw', '../../')      # Set up the ns-3 environment
 #exp = Experiment(mempool_key, mem_size, 'scratch/scenario1_mcs_base', '../../')      # Set up the ns-3 environment
try:
    if shared_variable.train == 1:
        # 进行模型训练
        for epoch in range(trainepoch):   #训练的epoch 此刻在训练！
            exp.reset()    # Reset the environment
            rl = Ns3AIRL(memblock_key, AiRLLENGTHEnv, AiRLLENGTHAct)  # Link the shared memory block with ns-3 script
            pro = exp.run(setting=ns3Settings2, show_output=True)  # Set and run the ns-3 script (sim.cc)
            ns3Settings2['RngRun'] = epoch + 1
            while not rl.isFinish():
                with rl as data:
                    if data == None:
                        print('python端没有数据：')
                        break
                    # AI algorithms here and put the data back to the action
                    data.act = ai.do(data.env, data.act)
                    throughput[epoch] = data.env.throughput
                    cwlist[0].append(data.env.cw_last[0])
                    lenlist[0].append(data.env.length_last[0])
                    len_netact[0].append(data.act.length[0])
                    cwlist[1].append(data.env.cw_last[1])
                    lenlist[1].append(data.env.length_last[1])
                    len_netact[1].append(data.act.length[1])
                    # 存起来 以免报错
                    np.savetxt("throughput.txt", ai.throughput_1s)
                    np.savetxt("len.txt", lenlist)
                    np.savetxt("cw.txt", cwlist)

                    # 进行收敛判断
                    # agent 1：
                    # if len(ai.agent_1.loss) > 2 and ai.agent_1.loss[len(ai.agent_1.loss)-1] - ai.agent_1.loss[len(ai.agent_1.loss)-2] < 0.1:
                    #     loss_num_1 = loss_num_1 + 1
                    # else :
                    #     loss_num_1 = 0                      #清空计算次数
                    # if  loss_num_1 > 500 and !stop:                   #当变化小于0.1的次数连续达到500次，则停止模型仿真
                    ##     ai.agent_1.save_models(epoch)       #存入模型参数
                    #     ai.agent_1.train = False            #不进行网络训练，即不调用learn函数
                    #     ai.agent_1.flag = False             #不进行轨迹存入
                    #     ai.agent_1.epsilon = 0              # 不进行贪心选取，全部由神经网络选择
                    #     stop = True

                        ##print(rl.isFinish())
            pro.wait()                                              # Wait the ns-3 to stop
            ai.agent_1.save_models(epoch+1)
            print(ai.agent_1.loss)
            # ai.agent_2.save_models(epoch)
            # ai.agent_3.save_models(epoch)


        #绘制吞吐量与epoch之间的关系图
        #x轴刻度标签
        # x轴范围（0, 1, ..., len(x_ticks)-1）
        x = np.arange(len(ai.throughput_1s))
        # 设置画布大小
        # plt.figure(figsize=(10, 6))
        # plt.subplot(221)
        # 画第1条折线
        plt.plot(x, ai.throughput_1s, color='#FF0000', label='throughput ', linewidth=3.0)
        # 给折线数据点加上数值，前两个参数是坐标，第三个是数值，ha和va分别是水平和垂直位置（数据点相对数值）。

        # for a, b in zip(x, throughput_1s):
        #  plt.text(a, b, '%d' % b, ha='center', va='bottom', fontsize=18)
        # 添加x轴和y轴标签
        plt.xlabel(u'epoch', fontsize=18)
        plt.ylabel(u'Throughput/Mbits', fontsize=18)
        # 标题
        plt.title(u'Throughput and epoch', fontsize=18)
        # 图例
        plt.legend(fontsize=18)
        # 显示图片
        plt.show()

        # 绘制时延与epoch之间的关系图
        # x轴刻度标签
        # x轴范围（0, 1, ..., len(x_ticks)-1）
        x = np.arange(len(ai.delay_1s))
        # 设置画布大小
        # plt.figure(figsize=(10, 6))
        # plt.subplot(221)
        # 画第1条折线
        plt.plot(x, ai.delay_1s, color='#FF0000', label='Delay ', linewidth=3.0)
        # 给折线数据点加上数值，前两个参数是坐标，第三个是数值，ha和va分别是水平和垂直位置（数据点相对数值）。

        # for a, b in zip(x, throughput_1s):
        #  plt.text(a, b, '%d' % b, ha='center', va='bottom', fontsize=18)
        # 添加x轴和y轴标签
        plt.xlabel(u'epoch', fontsize=18)
        plt.ylabel(u'Delay/ms', fontsize=18)
        # 标题
        plt.title(u'Delay and epoch', fontsize=18)
        # 图例
        plt.legend(fontsize=18)
        # 显示图片
        plt.show()

        ##############################
        ###### 绘制 loss 函数大小#######
        ##############################
        # x_ticks = ['1', '2', '3']
        # x轴范围（0, 1, ..., len(x_ticks)-1）
        x = np.arange(len(ai.agent_1.loss))
        # 设置画布大小
        plt.figure(figsize=(10, 6))
        # 画第1条折线
        plt.plot(x, ai.agent_1.loss, color='#FF0000', label='loss ', linewidth=3.0)
        # 给折线数据点加上数值，前两个参数是坐标，第三个是数值，ha和va分别是水平和垂直位置（数据点相对数值）。
        # for a, b in zip(x, ai.agent_1.loss):
        #     plt.text(a, b, '%d' % b, ha='center', va='bottom', fontsize=18)
        # 添加x轴和y轴标签
        plt.xlabel(u'交互轮次', fontsize=18)
        plt.ylabel(u'loss', fontsize=18)
        # 标题
        plt.title(u'loss与交互轮次之间的关系图', fontsize=18)
        # 图例
        plt.legend(fontsize=18)
        # 显示图片
        plt.show()


    if shared_variable.train == 2:
        # 训练完毕 进行模型仿真
            ai.agent_1.load_models('nof-1')  #读取模型参数
            ai.agent_1.train = False            #不进行网络训练，即不调用learn函数
            ai.agent_1.flag = False             #不进行轨迹存入
            ai.agent_1.epsilon = 0              #不进行贪心选取，全部由神经网络选择
            stop = True

            for epoch in range(simepoch):   #此刻没训练！
                exp.reset()    # Reset the environment
                rl = Ns3AIRL(memblock_key, AiRLLENGTHEnv, AiRLLENGTHAct)  # Link the shared memory block with ns-3 script
                pro = exp.run(setting=ns3Settings2, show_output=True)  # Set and run the ns-3 script (sim.cc)
                ns3Settings2['RngRun'] = epoch + 1
                while not rl.isFinish():
                    with rl as data:
                        if data == None:
                            print('python端没有数据：')
                            break
                        data.act = ai.do(data.env, data.act)
                        #throughput_sim[epoch] = data.env.throughput
                        cwlist[0].append(data.env.cw_last[0])
                        lenlist[0].append(data.env.length_last[0])
                        len_netact[0].append(data.act.length[0])
                        cwlist[1].append(data.env.cw_last[1])
                        lenlist[1].append(data.env.length_last[1])
                        len_netact[1].append(data.act.length[1])

            # x轴范围（0, 1, ..., len(x_ticks)-1）
            x = np.arange(len(ai.throughput_1s))
            # 设置画布大小
            # plt.figure(figsize=(10, 6))
            # plt.subplot(221)
            # 画第1条折线
            plt.plot(x, ai.throughput_1s, color='#FF0000', label='throughput ', linewidth=3.0)

            # 给折线数据点加上数值，前两个参数是坐标，第三个是数值，ha和va分别是水平和垂直位置（数据点相对数值）。
            # for a, b in zip(x, ai.throughput_1s):
            #  plt.text(a, b, '%d' % b, ha='center', va='bottom', fontsize=18)

            # 添加x轴和y轴标签
            plt.xlabel(u'epoch', fontsize=18)
            plt.ylabel(u'Throughput/Mbits', fontsize=18)
            # 标题
            plt.title(u'Throughput and epoch', fontsize=18)
            # 图例
            plt.legend(fontsize=18)
            # 显示图片
            plt.show()

            # 绘制时延与epoch之间的关系图
            # x轴刻度标签
            # x轴范围（0, 1, ..., len(x_ticks)-1）
            x = np.arange(len(ai.delay_1s))
            # 设置画布大小
            # plt.figure(figsize=(10, 6))
            # plt.subplot(221)
            # 画第1条折线
            plt.plot(x, ai.delay_1s, color='#FF0000', label='Delay ', linewidth=3.0)
            # 给折线数据点加上数值，前两个参数是坐标，第三个是数值，ha和va分别是水平和垂直位置（数据点相对数值）。
            # for a, b in zip(x, ai.delay_1s):
            #  plt.text(a, b, '%d' % b, ha='center', va='bottom', fontsize=18)
            # 添加x轴和y轴标签
            plt.xlabel(u'epoch', fontsize=18)
            plt.ylabel(u'Delay/ms', fontsize=18)
            # 标题
            plt.title(u'Delay and epoch', fontsize=18)
            # 图例
            plt.legend(fontsize=18)
            # 显示图片
            plt.show()

    ##############################
    ###### 绘制 动作选择 趋势#######
    ##############################
    x = np.arange(len(lenlist[0]))
    # 设置画布大小
    plt.figure(figsize=(10, 6))
    #plt.subplot(223)
    # 画第1条折线
    # for i in range(len(lenlist)):
    #     plt.plot(x, lenlist[i])
    plt.plot(x, lenlist[0], label='802.11ac')
    plt.plot(x, lenlist[1], label='802.11ax')
    # # 给折线数据点加上数值，前两个参数是坐标，第三个是数值，ha和va分别是水平和垂直位置（数据点相对数值）。
    # for a, b in zip(x, ai.agent_1.loss):
    #     plt.text(a, b, '%d' % b, ha='center', va='bottom', fontsize=5)
    # 添加x轴和y轴标签
    plt.xlabel(u'interaction steps', fontsize=18)
    plt.ylabel(u'Length', fontsize=18)
    # 标题
    plt.title(u'interaction steps and length', fontsize=18)
    # 图例
    plt.legend()
    # 显示图片
    plt.show()

    x = np.arange(len(len_netact[0]))
    # 设置画布大小
    plt.figure(figsize=(10, 6))
    # plt.subplot(223)
    # 画第1条折线
    # for i in range(len(len_netact)):
    #     plt.plot(x, len_netact[i])
    plt.plot(x, len_netact[0], label='802.11ac')
    plt.plot(x, len_netact[1], label='802.11ax')
    # # 给折线数据点加上数值，前两个参数是坐标，第三个是数值，ha和va分别是水平和垂直位置（数据点相对数值）。
    # for a, b in zip(x, ai.agent_1.loss):
    #     plt.text(a, b, '%d' % b, ha='center', va='bottom', fontsize=5)
    # 添加x轴和y轴标签
    plt.xlabel(u'interaction steps', fontsize=18)
    plt.ylabel(u'Length net action', fontsize=18)
    # 标题
    plt.title(u'interaction steps and length netact', fontsize=18)
    # 图例
    plt.legend()
    # 显示图片
    plt.show()

    x = np.arange(len(cwlist[0]))
    # 设置画布大小
    plt.figure(figsize=(10, 6))
    # plt.subplot(223)
    # 画第1条折线
    # for i in range(len(cwlist)):
    #     plt.plot(x, cwlist[i])
    plt.plot(x, cwlist[0], label='BE')
    # plt.plot(x, cwlist[0], label='802.11ac')
    # plt.plot(x, cwlist[1], label='802.11ax')
    # # 给折线数据点加上数值，前两个参数是坐标，第三个是数值，ha和va分别是水平和垂直位置（数据点相对数值）。
    # for a, b in zip(x, ai.agent_1.loss):
    #     plt.text(a, b, '%d' % b, ha='center', va='bottom', fontsize=5)
    # 添加x轴和y轴标签
    plt.xlabel(u'interaction steps', fontsize=18)
    plt.ylabel(u'CW', fontsize=18)
    # 标题
    plt.title(u'interaction steps and CW', fontsize=18)
    # 图例
    plt.legend()
    # 显示图片
    plt.show()

except Exception as e:
    print('Something wrong')
    print(e)
finally:
    del exp


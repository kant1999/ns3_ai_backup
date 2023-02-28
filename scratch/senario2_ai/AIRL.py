from ctypes import *
from py_interface import *
import time
import os
import torch
import argparse
import numpy as np
import torch.nn as nn
import matplotlib.pyplot as plt
import random
import pdb
import math
import csv
import dqn_net
from dqn_net import DQN_1
import shared_variable
#np.transpose

class AiRLLENGTHEnv(Structure):
    _pack_ = 1
    _fields_ = [
        ('snr', c_double * shared_variable.M),
        ('per', c_double * shared_variable.M),
        ('t_idle', c_double),
        ('cw_last', c_int*shared_variable.M),
        ('length_last',c_int*shared_variable.M),
        ('reward', c_double),
        ('throughput', c_double),      #
        ('bound_num',c_int)         # 
    ]
class AiRLLENGTHAct(Structure):
    _pack_ = 1
    _fields_ = [
        ('length', c_int*shared_variable.M),
        ('cw', c_int*shared_variable.M)
    ]     
class AiRLContainer:
    use_ns3ai = True
    def __init__(self, uid: int = shared_variable.memblock_key) -> None:
        #self.rl = Ns3AIRL(uid, AiRLRateEnv, AiRLRateAct)
        # print('({})size: Env {} Act {}'.format(uid, sizeof(AiConstantRateEnv), sizeof(AiConstantRateAct)))
        self.c = np.zeros(shared_variable.N)
        self.c1 = np.zeros(shared_variable.N)
        self.s = np.zeros(shared_variable.N)
        self.a = 0
        self.s_ = np.zeros(shared_variable.N)
        self.a_ = 0
        self.r = 0
        self.agent_1 = DQN_1()  # 初始化agent1
        self.train = True
        self.cur = []
        self.lossCount = []
        self.totalLearnStep = 0
        pass

    def do(self, env: AiRLLENGTHEnv, act: AiRLLENGTHAct, flag: bool) -> AiRLLENGTHAct:
            #pdb.set_trace()

            ######对观测向量ct进行处理########
        
            #pdb.set_trace()
            i = 0
            
            self.c[0:shared_variable.N] = [env.per[0],env.per[1],env.t_idle,1.0*(env.cw_last[0])/1023,env.length_last[0]/64,env.length_last[1]/80]
            self.c1[0:shared_variable.N] = [env.per[0],env.per[1],env.t_idle,env.cw_last[0],env.length_last[0],env.length_last[1]]
            #self.c[0:N] = [env.cw_last[0],env.cw_last[1],env.length_last[0],env.length_last[1]]
            #print(self.c)
            #if (env.reward==-5):
               # pdb.set_trace()
      
            ################################
            ####### 处理s、a、r、s_ ###########
            #################################

            #self.s_[env.sta_id, : ] = self.c.flatten()
           # self.s_[env.sta_id, :] = [env.snr[M],env.per[M],env.t_idle[M],env.cw_last,env.length_last]
            #if env.type_id == 0:
            self.s_=self.c
        
            p = self.agent_1.choose_action(self.s_)
            if(1):
                if p==0:
                    # act.cw[0]=0
                    # act.length[0]=0
                    # act.cw[1]=0
                    # act.length[1]=0   
                    act.cw[0]=1
                    act.length[0]=0
                    act.cw[1]=0
                    act.length[1]=0
                if p==1:
                    act.cw[0]=2
                    act.length[0]=0
                    act.cw[1]=0
                    act.length[1]=0  
                if p==2:
                    act.cw[0]=0
                    act.length[0]=1
                    act.cw[1]=0
                    act.length[1]=0  
                if p==3:
                    act.cw[0]=0
                    act.length[0]=2
                    act.cw[1]=0
                    act.length[1]=0  
                if p==4:
                    act.cw[0]=0
                    act.length[0]=0
                    act.cw[1]=0
                    act.length[1]=1            
                if p==5:
                    act.cw[0]=0
                    act.length[0]=0
                    act.cw[1]=0
                    act.length[1]=2
                    # act.cw[0]=0
                    # act.length[0]=0
                    # act.cw[1]=1
                    # act.length[1]=0
                # if p==6:
                #     act.cw[0]=0
                #     act.length[0]=0
                #     act.cw[1]=0
                #     act.length[1]=0
                # if p==7:
                #     act.cw[0]=0
                #     act.length[0]=0
                #     act.cw[1]=0
                #     act.length[1]=1
                # if p==8:
                #     p=0
                #     act.cw[0]=0
                #     act.length[0]=0
                #     act.cw[1]=0
                #     act.length[1]=2
                    

            self.a_ = p
            self.r  = env.reward
            #self.cur.append(env.cur)
            

            #########################################
            ####### 分类DQN进行经验向量存储 ###########
            #######################################
            #print('STA ID：', env.sta_id)
            #print('观测向量C：', self.c)
           # print('状态向量S：', self.s)
            print('奖励向量R：', self.r)
            print('状态向量s_：', self.c1)
            print('动作：',self.a,self.a_)
            print('throughput=',env.throughput)
            #print('状态向量S_：', self.s_)
            
            
            global t_count
            t_count=t_count+1
            t_over=(t_sim*(1000/shared_variable.timestep)*shared_variable.trainepoch/t_count-1)*(time.time()-t_start)
            
            print("预计训练结束北京时间",time.localtime(time.time()+t_over+16*3600),"预计还有多少秒结束",t_over)
            #print('收敛所用学习步数：', self.totalLearnStep)


            if  flag:
               if  not all(item == 0 for item in self.s_) and not all(item == 0 for item in self.s) :   #进行经验池的过滤:
                     #这里要调用一下store函数 存入轨迹哦！
                      # print('正在存入轨迹！')
                      # print(self.s[env.sta_id, : ])
                      # print(self.a[env.sta_id, :])
                      # print(self.r)
                      # print(self.s_[env.sta_id, :])
                      self.agent_1.store_transition(self.s, self.a, self.r, self.s_)#轨迹存入
            # if env.type_id == 2:
            #    if flag and env.snr[0]!=0 and env.snr[1]!=0 and env.snr[2]!=0 and env.snr[3]!=0 and env.snr[4]!=0 and env.snr[5]!=0:
            #          #这里要调用一下store函数 存入轨迹哦！
            #          self.agent_2.store_transition(self.s[env.sta_id, : ], self.a[env.sta_id, : ], self.r, self.s_[env.sta_id, : ])#轨迹存入
            # if env.type_id == 3:
            #    if flag and env.snr[0]!=0 and env.snr[1]!=0 and env.snr[2]!=0 and env.snr[3]!=0 and env.snr[4]!=0 and env.snr[5]!=0:
            #          #这里要调用一下store函数 存入轨迹哦！
            #          self.agent_3.store_transition(self.s[env.sta_id, : ], self.a[env.sta_id, : ], self.r, self.s_[env.sta_id, : ])#轨迹存入
            
            self.a = self.a_
            
            self.s=self.s_.copy()

            
          
               
            #这里要训练 调用learn函数哦！
            if self.agent_1.memory_counter >500 and self.train:
                self.agent_1.learn()
                #print('agent 1 is learning')
            # if self.agent_2.memory_counter > self.agent_2.memory_capacity and self.train:
            #     self.agent_2.learn()
            #     print('agent 2 is learning')
            # if self.agent_3.memory_counter > self.agent_3.memory_capacity and self.train:
            #     self.agent_3.learn()
            #     print('agent 3 is learning')


            if len(self.agent_1.loss) > 0:
                self.lossCount = self.agent_1.loss[(len(self.agent_1.loss)-10):len(self.agent_1.loss):1]
            if len(self.lossCount)!=0:
                pass
                #print('lossCount: ',self.lossCount,' sum: ',sum(self.lossCount)/len(self.lossCount))
            if self.train and len(self.lossCount)==10 and (sum(self.lossCount)/len(self.lossCount)).item() < 0:
                print("train over")
                self.train = False  # 不进行网络训练，即不调用learn函数
                flag = False  # 不进行轨迹存入
                self.agent_1.epsilon = 0  # 不进行贪心选取，全部由神经网络选择
                self.totalLearnStep = self.agent_1.learn_step
            return act

if __name__ == '__main__':

 mempool_key = 1234                                          # memory pool key, arbitrary integer large than 1000
 mem_size = 4096                                             # memory pool size in bytes
 memblock_key = shared_variable.memblock_key                                         # memory block key, need to keep the same in the ns-3 script
 flag = True
 trainepoch = shared_variable.trainepoch                                             # train number
 simepoch = 1                                                # simulate number
 throughput = np.zeros(trainepoch)
 throughput_1s=np.zeros(0)
 throughput_sim = np.zeros(simepoch)
 t_start=time.time()
 t_dif=0
 t_over=0
 t_count=0
 t_sim=shared_variable.t_sim
 timestep=shared_variable.timestep
 filename=shared_variable.f+'_scenario2_ai'
 ns3Settings1 = {'tracing': 'true', 'useRts': 'false','useExtendedBlockAck': 'true',
               'simulationTime': 5 ,'dlAckType': 'NO-OFDMA','frequency': 5,
               'channelWidth': 160 ,'distance': 1 ,'mcs': 11 ,'RngRun': 1} #元组

 ns3Settings2 = {'output_log_name':filename,'useExtendedBlockAck': 'true',
                 'simulationTime': t_sim, 'dlAckType': 'NO-OFDMA', 'RngRun': 7,'epoch':0,'timestep':timestep}  # zz

 ai = AiRLContainer(memblock_key)
 cwlist = []
 cwlist.append([])
 cwlist.append([])
 lenlist=[]
 lenlist.append([])
 lenlist.append([])
 print('1')
 #c.agent_ax.load_models(2)

 print(filename)
 exp = Experiment(mempool_key, mem_size, 'scratch/'+shared_variable.f+'/scenario2_ai', '../../')      # Set up the ns-3 environment
 print('2')
 train=shared_variable.train
 #exp = Experiment(mempool_key, mem_size, 'scratch/scenario1_mcs_base', '../../')      # Set up the ns-3 environment
try:
    # 进行模型训练
    if train==1:
        for epoch in range(trainepoch):   #训练的epoch 此刻在训练！
            exp.reset()    # Reset the environment
            rl = Ns3AIRL(memblock_key, AiRLLENGTHEnv, AiRLLENGTHAct)  # Link the shared memory block with ns-3 script
            print('3')
            ns3Settings2 = {'output_log_name':filename,'useExtendedBlockAck': 'true',
                 'simulationTime': t_sim, 'dlAckType': 'NO-OFDMA', 'RngRun': 5,'epoch':epoch,'timestep':timestep,'timestep_large':shared_variable.timestep_large}  # zz
            pro = exp.run(setting=ns3Settings2, show_output=True)  # Set and run the ns-3 script (sim.cc)
            #ns3Settings2['RngRun'] = epoch + 1
            while not rl.isFinish():
                with rl as data:
                    if data == None:
                        print('python端没有数据：')
                        break
                    # AI algorithms here and put the data back to the action
                    data.act = ai.do(data.env, data.act, flag)
                    throughput[epoch] = data.env.throughput
                    throughput_1s=np.append(throughput_1s,data.env.throughput)
                    #pdb.set_trace()
                    cwlist[0].append(data.env.cw_last[0])
                    cwlist[1].append(data.env.cw_last[1])
                    lenlist[0].append(data.env.length_last[0])
                    lenlist[1].append(data.env.length_last[1])
                    print('epoch==',epoch,'lunci==',ai.agent_1.memory_counter)

            ##print(rl.isFinish())
            pro.wait()                                              # Wait the ns-3 to stop
            ai.agent_1.save_models(epoch+4)
            #ai.agent_1.memory = np.zeros((200, 16))
            #ai.agent_1.memory_counter=0
            #ai.agent_1.eps_dec=(5e-3)*(epoch+1)
            #ai.agent_1.epsilon = 1

            # ai.agent_2.save_models(epoch)
            # ai.agent_3.save_models(epoch)


        #绘制吞吐量与epoch之间的关系图
        #x轴刻度标签

        # x轴范围（0, 1, ..., len(x_ticks)-1）
        #pdb.set_trace()
        x = np.arange(len(throughput_1s))
        # 设置画布大小
        #plt.figure(figsize=(10, 6))
        plt.subplot(221)
        # 画第1条折线
        plt.plot(x, throughput_1s, color='#FF0000', label='throughput ', linewidth=3.0)
        #给折线数据点加上数值，前两个参数是坐标，第三个是数值，ha和va分别是水平和垂直位置（数据点相对数值）。

        #for a, b in zip(x, throughput_1s):
            #  plt.text(a, b, '%d' % b, ha='center', va='bottom', fontsize=18)
        #添加x轴和y轴标签
        plt.xlabel(u'epoch', fontsize=18)
        plt.ylabel(u'Throughput/Mbits', fontsize=18)
        # 标题
        plt.title(u'Throughput and epoch', fontsize=18)
        # 图例
        plt.legend(fontsize=18)
        # 显示图片
        
    

    if train==2:
        # 训练完毕 进行模型仿真
        ai.agent_1.load_models(7)  #读取模型参数
        ai.train = False            #不进行网络训练，即不调用learn函数
        flag = False               #不进行轨迹存入
        ai.agent_1.epsilon = 0     #不进行贪心选取，全部由神经网络选择
        
        for epoch in range(1):   #此刻没训练！
            exp.reset()    # Reset the environment
            rl = Ns3AIRL(memblock_key, AiRLLENGTHEnv, AiRLLENGTHAct)  # Link the shared memory block with ns-3 script
            ns3Settings2 = {'output_log_name': filename,'useExtendedBlockAck': 'true',
                 'simulationTime': t_sim, 'dlAckType': 'NO-OFDMA', 'RngRun': 5,'epoch':5,'timestep':timestep,'timestep_large':shared_variable.timestep_large}  # zz
            pro = exp.run(setting=ns3Settings2, show_output=True)  # Set and run the ns-3 script (sim.cc)
            #ns3Settings2['RngRun'] = epoch + 1
          
            while not rl.isFinish():
                with rl as data:
                    if data == None:
                        print('python端没有数据：')
                        break
                    data.act = ai.do(data.env, data.act ,flag)
        
                    throughput[epoch] = data.env.throughput
                    throughput_1s=np.append(throughput_1s,data.env.throughput)
                    cwlist[0].append(data.env.cw_last[0])
                    cwlist[1].append(data.env.cw_last[1])
                    lenlist[0].append(data.env.length_last[0])
                    lenlist[1].append(data.env.length_last[1])
        
        
        # 绘制吞吐量与epoch之间的关系图
        # x轴刻度标签
        # x轴范围（0, 1, ..., len(x_ticks)-1）
        x = np.arange(len(throughput_1s))
        # 设置画布大小
        plt.figure(figsize=(10, 6))
        # 画第1条折线
        plt.subplot(221)
        plt.plot(x, throughput_1s, color='#FF0000', label='throughput ', linewidth=3.0)
        # 给折线数据点加上数值，前两个参数是坐标，第三个是数值，ha和va分别是水平和垂直位置（数据点相对数值）。
        #for a, b in zip(x, throughput_sim):
        # plt.text(a, b, '%d' % b, ha='center', va='bottom', fontsize=18)
        # 添加x轴和y轴标签
        plt.xlabel(u'epoch', fontsize=18)
        plt.ylabel(u'Throughput/Mbits', fontsize=18)
        # 标题
        plt.title(u'Throughput and epoch', fontsize=18)
        # 图例
        plt.legend(fontsize=18)
        # 显示图片
        

    ##############################
    ###### 绘制 loss 函数大小#######
    ##############################
    #x_ticks = ['1', '2', '3']
    # x轴范围（0, 1, ..., len(x_ticks)-1）
    x = np.arange(len(ai.agent_1.loss))
    # 设置画布大小
    #plt.figure(figsize=(10, 6))
    plt.subplot(222)
    # 画第1条折线
    plt.plot(x, ai.agent_1.loss, color='#FF0000', label='loss ', linewidth=3.0)
    # 给折线数据点加上数值，前两个参数是坐标，第三个是数值，ha和va分别是水平和垂直位置（数据点相对数值）。
   # for a, b in zip(x, ai.agent_1.loss):
    #    plt.text(a, b, '%d' % b, ha='center', va='bottom', fontsize=5)
    # 添加x轴和y轴标签
    plt.xlabel(u'epoch', fontsize=18)
    plt.ylabel(u'loss', fontsize=18)
    # 标题
    plt.title(u'loss and epoch', fontsize=18)
    # 图例
    plt.legend(fontsize=18)
    # 显示图片
    
    


    ##############################
    ###### 绘制 action 函数大小#######
    ##############################
    
    
    x = np.arange(len(cwlist[0]))
    # 设置画布大小
    #plt.figure(figsize=(10, 6))
    plt.subplot(223)
    # 画第1条折线
    # for i in range(len(cwlist)):
    #     plt.plot(x, cwlist[i])
    # # 给折线数据点加上数值，前两个参数是坐标，第三个是数值，ha和va分别是水平和垂直位置（数据点相对数值）。
    # for a, b in zip(x, ai.agent_1.loss):
    #     plt.text(a, b, '%d' % b, ha='center', va='bottom', fontsize=5)
    # 添加x轴和y轴标签
    plt.xlabel(u'interaction steps', fontsize=18)
    plt.ylabel(u'CW', fontsize=18)
    # 标题
    plt.title(u'interaction steps and CW', fontsize=18)
    # 图例
    plt.plot(x,cwlist[0],label='802.11ac')
    # plt.plot(x,cwlist[1],label='802.11ax')
    plt.legend()
    # 显示图片
    
    

    x = np.arange(len(lenlist[0]))
    # 设置画布大小
    #plt.figure(figsize=(10, 6))
    # 画第1条折线
    plt.subplot(224)
    

    for i in range(len(lenlist)):
        plt.plot(x, lenlist[i])
    # # 给折线数据点加上数值，前两个参数是坐标，第三个是数值，ha和va分别是水平和垂直位置（数据点相对数值）。
    # for a, b in zip(x, ai.agent_1.loss):
    #     plt.text(a, b, '%d' % b, ha='center', va='bottom', fontsize=5)
    # 添加x轴和y轴标签
    plt.xlabel(u'interaction steps', fontsize=18)
    plt.ylabel(u'Length', fontsize=18)
    # 标题
    plt.title(u'interaction steps and Length', fontsize=18)
    # 图例
    plt.plot(x,lenlist[0],label='802.11ac')
    plt.plot(x,lenlist[1],label='802.11ax')
    plt.legend()
    # 显示图片
    
    plt.show()
   

    

except Exception as e:
    print('Something wrong')
    print(e)
finally:
    del exp

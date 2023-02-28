#python参数
train=1  #train=1时训练，等于2时验证
M = 2    #分组个数1-8
N = 6    #参数个数 group number * 2 + 2
NUM=6    #动作个数
#memory_capacity=500
trainepoch = 4               #训练多少epoch 
t_sim=100                     #仿真一个epoch的时间          单位秒
#ns3端的参数
timestep=200                 #ns3与python交互 的时间隙度  单位毫秒
timestep_large=1000          #ns3多久计算一次吞吐量       单位ms
#共同的参数
memblock_key = 2321  

#文件操作
import os
filename=os.path.abspath(__file__)
f=filename.split('/')[-2]
print(f)
import torch
import argparse
import numpy as np
import torch.nn as nn
import argparse
import math
import pdb
import os
import shared_variable


def create_directory(path: str, sub_dirs: list):
    for sub_dir in sub_dirs:
        if os.path.exists(path + sub_dir):
            print(path + sub_dir + ' is already exist!')
        else:
            os.makedirs(path + sub_dir, exist_ok=True)
            print(path + sub_dir + ' create successfully!')

parser = argparse.ArgumentParser()
parser.add_argument('--ckpt_dir', type=str, default='./checkpoints/DQN/')
args = parser.parse_args()
create_directory(args.ckpt_dir, sub_dirs=['Q_eval_1', 'Q_target_1','Q_eval_2', 'Q_target_2','Q_eval_3', 'Q_target_3'])




class net_ax(nn.Module):
    def __init__(self):
    #     super(net_ax, self).__init__()
    #     self.fc1=nn.Linear(shared_variable.N, 20)
    #     self.fc2=nn.Linear(20, shared_variable.NUM)
        

    # def forward(self, x):
    #     x = nn.functional.relu(self.fc1(x))  # 隐藏层使用ReLU激活函数
    #     #pdb.set_trace()
    #     return self.fc2(x)
    #     #return self.layers(x)
        super(net_ax, self).__init__()
        self.layers = nn.Sequential(
            nn.Linear(shared_variable.N, 20),
            nn.ReLU(),
            nn.Linear(20, 20),
            nn.ReLU(),
            nn.Linear(20, shared_variable.NUM),
        )

    def forward(self, x):
        return self.layers(x)

    def save_checkpoint(self, checkpoint_file):
        torch.save(self.state_dict(), checkpoint_file, _use_new_zipfile_serialization=False)

    def load_checkpoint(self, checkpoint_file):
        self.load_state_dict(torch.load(checkpoint_file))



class DQN_1(object):
    def __init__(self):
        self.eval_net = net_ax()
        self.target_net = net_ax()
        self.learn_step = 0
        self.batchsize = 32
        self.observer_shape = shared_variable.N     #观察的状态参数有几个变量
        self.target_replace = 100   #进行网络更新的步骤数
        self.memory_counter = 0
        self.memory_capacity = 500
        self.epsilon = 1
        self.eps_min = 0.1
        self.eps_dec = 1e-3
        self.memory = np.zeros((self.memory_capacity, self.observer_shape*2+2))    # s, a, r, s'
        self.optimizer = torch.optim.Adam(self.eval_net.parameters(), lr=0.001)
        self.loss_func = nn.MSELoss()
        self.checkpoint_dir = args.ckpt_dir
        self.loss = []
        print('_____________________________________________________________________init_______________________________________________________')

    def choose_action(self, x):
        x = torch.Tensor(x)
        if np.random.uniform() > self.epsilon :  #** self.memory_counter:    # choose best
            action = self.eval_net.forward(x)
            #pdb.set_trace()
            action = torch.argmax(action, 0).item()
        else:    # explore
            action = np.random.randint(0, shared_variable.NUM)
        return action

    def store_transition(self, s, a, r, s_):
        index = self.memory_counter % self.memory_capacity
        #pdb.set_trace()
        
        self.memory[index, :] = np.hstack((s, [a, r], s_))
        f=open('train_data.txt','a+')
        for i in self.memory[index, :]:
            f.write(str(i)+' ')
        f.write('\n')
        f.close() 
        # m = self.memory[index, :]
       
        # print(m.shape)
        self.memory_counter += 1
        #print(self.memory_counter)

    def learn(self):
        #pdb.set_trace()
        self.learn_step += 1
        if self.learn_step % self.target_replace == 0:
            self.target_net.load_state_dict(self.eval_net.state_dict())
        # print('learn!')
        sample_list = np.random.choice(self.memory_capacity, self.batchsize)

        # choose a mini batch
        sample = self.memory[sample_list, :]

        s = torch.Tensor(sample[:,:self.observer_shape])
        a = torch.LongTensor(
            sample[:,self.observer_shape:self.observer_shape+1])
        r = torch.Tensor(
            sample[:,self.observer_shape+1:self.observer_shape+2])
        s_ = torch.Tensor(sample[:,self.observer_shape+2:])
        # 计算q_target,loss，反向传递
        q_eval = self.eval_net(s).gather(1, a) #注意gather函数！
        q_next = self.target_net(s_).detach()  #注意detach()函数！
        max_a=self.eval_net(s_).max(1)[1].view(-1,1)
        max_next_q=self.target_net(s_).gather(1,max_a)
        q_target = r + 0.8 *max_next_q
        #q_target = r + 0.8 * q_next.max(1)[0].view(self.batchsize, 1)
        loss = self.loss_func(q_eval, q_target)
        #pdb.set_trace()
        self.optimizer.zero_grad()
        self.loss.append(loss.detach()) # 记录loss的值
        #print("self.loss==",self.loss,"q_eval=",q_eval,"q_target=",q_target)
        print("lossssssssssssssssssssssssssss",loss.item())
        if (math.isnan(loss)):
            pdb.set_trace()
            
        loss.backward()
        self.optimizer.step()
        print('epsilon=',self.epsilon )

        if self.epsilon > self.eps_min:
           self.epsilon = self.epsilon - self.eps_dec    # 更新贪心探索的参数值
        # f=open('train_data2.txt','a+')
        
        # f.write(loss+' '+)
        # f.write('\n')
        # f.close 


    def save_models(self, episode):
       self.eval_net.save_checkpoint(self.checkpoint_dir + 'Q_eval_1/DQN_q_eval_{}.pth'.format(episode))
       print('Saving Q_eval network successfully!')
       self.target_net.save_checkpoint(self.checkpoint_dir + 'Q_target_1/DQN_Q_target_{}.pth'.format(episode))
       print('Saving Q_target network successfully!')

    def load_models(self, episode):
       self.eval_net.load_checkpoint(self.checkpoint_dir + 'Q_eval_1/DQN_q_eval_{}.pth'.format(episode))
       print('Loading Q_eval network successfully!')
       self.target_net.load_checkpoint(self.checkpoint_dir + 'Q_target_1/DQN_Q_target_{}.pth'.format(episode))
       print('Loading Q_target network successfully!')
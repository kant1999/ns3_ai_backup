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
        super(net_ax, self).__init__()
        self.layers = nn.Sequential(
            nn.Linear(shared_variable.N, 20),
            nn.ReLU(),
            nn.Linear(20, 20),
            nn.ReLU(),
            nn.Linear(20, shared_variable.Output),
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
        self.target_replace = 100    #进行网络更新的步骤数
        self.memory_counter = 0
        self.memory_capacity = 500
        self.epsilon = 1
        self.eps_min = 0.01
        self.eps_dec = 5e-4
        self.memory = np.zeros((self.memory_capacity, self.observer_shape*2+ shared_variable.Actnum + 1))    # s, a, r, s'
        self.memory_data = np.zeros((2000, self.observer_shape * 2 + shared_variable.Actnum + 1))  # s, a, r, s'
        self.optimizer = torch.optim.Adam(
            self.eval_net.parameters(), lr=0.0001)
        self.loss_func = nn.MSELoss()
        self.checkpoint_dir = args.ckpt_dir
        self.loss = []
        self.flag = True               #是否进行轨迹存入
        self.train = True              #是否进行网络训练，即是否调用learn函数


    def choose_action(self, x):
        x = torch.Tensor(x)
        if np.random.uniform() > self.epsilon :  #** self.memory_counter:    # choose best
            action = self.eval_net.forward(x)
            action = torch.argmax(action, 0).item()
        else:    # explore
            action = np.random.randint(0, shared_variable.Output)
        return action

    def store_transition(self, s, a, r, s_):
        index = self.memory_counter % self.memory_capacity
        index_2000 = self.memory_counter % 2000
        self.memory[index, :] = np.hstack((s, [a, r], s_))
        self.memory_data[index_2000, :] = np.hstack((s, [a, r], s_))
        np.savetxt("experience.txt", self.memory)
        np.savetxt("experience_2000.txt", self.memory_data)
        np.savetxt("loss.txt", self.loss)
        # m = self.memory[index, :]
        # print(self.memory[index, :])
        # print(m.shape)
        self.memory_counter += 1
        print(self.memory_counter)

    def learn(self):
        self.learn_step += 1
        if self.learn_step % self.target_replace == 0:
            self.target_net.load_state_dict(self.eval_net.state_dict())
        # print('learn!')
        sample_list = np.random.choice(self.memory_capacity, self.batchsize)

        # choose a mini batch
        sample = self.memory[sample_list, :]

        s = torch.Tensor(sample[:, :self.observer_shape])
        a = torch.LongTensor(
            sample[:, self.observer_shape:self.observer_shape+shared_variable.Actnum])
        r = torch.Tensor(
            sample[:, self.observer_shape+shared_variable.Actnum:self.observer_shape+shared_variable.Actnum+1])
        s_ = torch.Tensor(sample[:, self.observer_shape+shared_variable.Actnum+1:])
        # 计算q_target,loss，反向传递
        q_eval = self.eval_net(s).gather(1, a) #注意gather函数！
        q_next = self.target_net(s_).detach()  #注意detach()函数！
        max_a = self.eval_net(s_).max(1)[1].view(-1,1)
        max_next_q = self.target_net(s_).gather(1,max_a)
        #q_target = r + 0.8 * q_next.max(1, True)[0].data
        #q_target = r + 0.8 * q_next.max(1)[0].view(self.batchsize, 1)
        q_target = r + 0.8 * max_next_q
        loss = self.loss_func(q_eval, q_target)
        self.optimizer.zero_grad()
        self.loss.append(loss.detach()) # 记录loss的值
        #print(self.loss)
        loss.backward()
        self.optimizer.step()

        if self.epsilon > self.eps_min:
           self.epsilon = self.epsilon - self.eps_dec    # 更新贪心探索的参数值


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

class DQN_2(object):
    def __init__(self):
        self.eval_net = net_ax()
        self.target_net = net_ax()
        self.learn_step = 0
        self.batchsize = 32
        self.observer_shape = 2 #观察的状态参数有几个变量
        self.target_replace = 100 #进行网络更新的步骤数
        self.memory_counter = 0
        self.memory_capacity = 500
        self.epsilon = 1
        self.eps_min = 0.01
        self.eps_dec = 5e-4
        self.memory = np.zeros((self.memory_capacity, self.observer_shape*2+2))    # s, a, r, s'
        self.optimizer = torch.optim.Adam(
            self.eval_net.parameters(), lr=0.0001)
        self.loss_func = nn.MSELoss()
        self.checkpoint_dir = args.ckpt_dir
        self.loss = []

    def choose_action(self, x):
        x = torch.Tensor(x)
        if np.random.uniform() > self.epsilon :  #** self.memory_counter:    # choose best
            action = self.eval_net.forward(x)
            action = torch.argmax(action, 0).item()
        else:    # explore
            action = np.random.randint(0, 3)
        return action

    def store_transition(self, s, a, r, s_):
        index = self.memory_counter % self.memory_capacity
        self.memory[index, :] = np.hstack((s, [a, r], s_))
        self.memory_counter += 1

    def learn(self):
        self.learn_step += 1
        if self.learn_step % self.target_replace == 0:
            self.target_net.load_state_dict(self.eval_net.state_dict())

        sample_list = np.random.choice(self.memory_capacity, self.batchsize)
        # choose a mini batch
        sample = self.memory[sample_list, :]
        s = torch.Tensor(sample[:, :self.observer_shape])
        a = torch.LongTensor(
            sample[:, self.observer_shape:self.observer_shape+1])
        r = torch.Tensor(
            sample[:, self.observer_shape+1:self.observer_shape+2])
        s_ = torch.Tensor(sample[:, self.observer_shape+2:])
        # 计算q_target,loss，反向传递
        q_eval = self.eval_net(s).gather(1, a) #注意gather函数！
        q_next = self.target_net(s_).detach()  #注意detach()函数！
        #q_target = r + 0.8 * q_next.max(1, True)[0].data
        q_target = r + 0.8 * q_next.max(1)[0].view(self.batchsize, 1)
        loss = self.loss_func(q_eval, q_target)
        self.optimizer.zero_grad()
        self.loss.append(loss.detach()) # 记录loss的值
        print(self.loss)
        loss.backward()
        self.optimizer.step()

        if self.epsilon > self.eps_min:
           self.epsilon = self.epsilon - self.eps_dec    # 更新贪心探索的参数值


    def save_models(self, episode):
       self.eval_net.save_checkpoint(self.checkpoint_dir + 'Q_eval_2/DQN_q_eval_{}.pth'.format(episode))
       print('Saving Q_eval network successfully!')
       self.target_net.save_checkpoint(self.checkpoint_dir + 'Q_target_2/DQN_Q_target_{}.pth'.format(episode))
       print('Saving Q_target network successfully!')

    def load_models(self, episode):
       self.eval_net.load_checkpoint(self.checkpoint_dir + 'Q_eval_2/DQN_q_eval_{}.pth'.format(episode))
       print('Loading Q_eval network successfully!')
       self.target_net.load_checkpoint(self.checkpoint_dir + 'Q_target_2/DQN_Q_target_{}.pth'.format(episode))
       print('Loading Q_target network successfully!')

class DQN_3(object):
    def __init__(self):
        self.eval_net = net_ax()
        self.target_net = net_ax()
        self.learn_step = 0
        self.batchsize = 32
        self.observer_shape = 2 #观察的状态参数有几个变量
        self.target_replace = 100 #进行网络更新的步骤数
        self.memory_counter = 0
        self.memory_capacity = 500
        self.epsilon = 1
        self.eps_min = 0.01
        self.eps_dec = 5e-4
        self.memory = np.zeros((self.memory_capacity, self.observer_shape*2+2))    # s, a, r, s'
        self.optimizer = torch.optim.Adam(
            self.eval_net.parameters(), lr=0.0001)
        self.loss_func = nn.MSELoss()
        self.checkpoint_dir = args.ckpt_dir
        self.loss = []
        self.flag = True
        self.train = True
        # ai.train = False            #不进行网络训练，即不调用learn函数
        # flag = False               #不进行轨迹存入
        # ai.agent_1.epsilon = 0     #不进行贪心选取，全部由神经网络选择

    def choose_action(self, x):
        x = torch.Tensor(x)
        if np.random.uniform() > self.epsilon :  #** self.memory_counter:    # choose best
            action = self.eval_net.forward(x)
            action = torch.argmax(action, 0).item()
        else:     # explore
            action = np.random.randint(0, 3)
        return action

    def store_transition(self, s, a, r, s_):
        index = self.memory_counter % self.memory_capacity
        self.memory[index, :] = np.hstack((s, [a, r], s_))
        self.memory_counter += 1

    def learn(self):
        self.learn_step += 1
        if self.learn_step % self.target_replace == 0:
            self.target_net.load_state_dict(self.eval_net.state_dict())

        sample_list = np.random.choice(self.memory_capacity, self.batchsize)
        # choose a mini batch
        sample = self.memory[sample_list, :]
        s = torch.Tensor(sample[:, :self.observer_shape])
        a = torch.LongTensor(
            sample[:, self.observer_shape:self.observer_shape+1])
        r = torch.Tensor(
            sample[:, self.observer_shape+1:self.observer_shape+2])
        s_ = torch.Tensor(sample[:, self.observer_shape+2:])
        # 计算q_target,loss，反向传递
        q_eval = self.eval_net(s).gather(1, a) #注意gather函数！
        q_next = self.target_net(s_).detach()  #注意detach()函数！
        #q_target = r + 0.8 * q_next.max(1, True)[0].data
        q_target = r + 0.8 * q_next.max(1)[0].view(self.batchsize, 1)
        loss = self.loss_func(q_eval, q_target)
        self.optimizer.zero_grad()
        self.loss.append(loss.detach()) # 记录loss的值
        print(self.loss)
        loss.backward()
        self.optimizer.step()

        if self.epsilon > self.eps_min:
           self.epsilon = self.epsilon - self.eps_dec    # 更新贪心探索的参数值


    def save_models(self, episode):
       self.eval_net.save_checkpoint(self.checkpoint_dir + 'Q_eval_3/DQN_q_eval_{}.pth'.format(episode))
       print('Saving Q_eval network successfully!')
       self.target_net.save_checkpoint(self.checkpoint_dir + 'Q_target_3/DQN_Q_target_{}.pth'.format(episode))
       print('Saving Q_target network successfully!')

    def load_models(self, episode):
       self.eval_net.load_checkpoint(self.checkpoint_dir + 'Q_eval_3/DQN_q_eval_{}.pth'.format(episode))
       print('Loading Q_eval network successfully!')
       self.target_net.load_checkpoint(self.checkpoint_dir + 'Q_target_3/DQN_Q_target_{}.pth'.format(episode))
       print('Loading Q_target network successfully!')

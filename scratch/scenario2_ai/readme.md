直接python3 AIRL.py运行。共享的参数在shared_variable里面

需要修改参数直接在shared_variable里面改。

注意：shared_variable.py的memblock_key必须和.h文件里128行的m_ns3ai_id保持一致。并且不能和其他文件的相冲突。修改该值可以并行跑仿真

默认参数
不开RTS/CTS
经验池为500
NS3_AI与python交互时间200ms
eposide=4
不使用SNR作为状态
target replace=100
神经网络个数20：20
越界惩罚-0.05

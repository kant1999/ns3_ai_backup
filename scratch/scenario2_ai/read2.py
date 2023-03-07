#f=open('train_data.txt','r')
import matplotlib.pyplot as plt
b=[]
with open('/home/zhr/workspace/ns-allinone-3.37/ns-3.37/scratch/CW7/train_data.txt') as f:
    for line in f:
        a=float(line.split(' ')[8])
        print(a)
        b.append(a)
plt.plot(b)
plt.show()
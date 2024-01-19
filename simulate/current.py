import numpy as np
import matplotlib.pyplot as plt

current_alpha=[]
current_beta=[]
current_d=[]
current_q=[]
one_sqr3 = 1/np.sqrt(3)
sqr3 =np.sqrt(3)
pi = np.pi
x = []
A = 0.2
Ia =[] 
Ib =[]
Ic =[]
total=[]
# Ia = A*np.sin(x)
# Ib = A*np.sin(x+19.108)
# Ib = A*np.sin(x-19.108)

def Clarke_transfrom(x):
    current_alpha.append(1.0*2/3*(Ia[x] - Ib[b]/2 -Ic[b]/2))
    current_beta.append(1.0*one_sqr3*(Ib[b] - Ic[b]))



def Park_transfrom(x):
    current_d.append(current_alpha*np.cos(x) + current_beta*np.sin(x))
    current_q.append(current_beta*np.cos(x) - current_alpha*np.sin(x))


for a in range(0,300):
    x.append(a/10)
for b in range(0,300):

    # 正转
    # Ia.append(A*np.sin(x[b]))
    # Ib.append(A*np.sin(x[b]-2.099))
    # Ic.append(A*np.sin(x[b]+2.099))
    # total.append(Ia[b]+Ib[b]+Ic[b])
    # current_alpha.append(1.0*2/3*(Ia[b] - Ib[b]/2 -Ic[b]/2))
    # current_beta.append(1.0*one_sqr3*(Ib[b] - Ic[b]))
    # current_d.append(current_alpha[b]*np.cos(x[b]) + current_beta[b]*np.sin(x[b]))
    # current_q.append(current_beta[b]*np.cos(x[b]) - current_alpha[b]*np.sin(x[b]))

    # 反转
    Ia.append(20*np.sin(x[b]))
    Ic.append(20*np.sin(x[b]-2.0934))
    Ib.append(10*np.sin(x[b]+2.0934))
    total.append(Ia[b]+Ib[b]+Ic[b])
    current_alpha.append(1.0*2/3*(Ia[b] - Ib[b]/2 -Ic[b]/2))
    current_beta.append(1.0*one_sqr3*(Ib[b] - Ic[b]))
    current_d.append(current_beta[b]*np.cos(x[b]) + current_alpha[b]*np.sin(x[b]))
    current_q.append(current_alpha[b]*np.cos(x[b]) - current_beta[b]*np.sin(x[b]))

#创建画板1
fig = plt.figure(1) #如果不传入参数默认画板1
#第2步创建画纸，并选择画纸1
ax1=plt.subplot(3,1,1)   
#在画纸1上绘图
plt.plot(x,Ia,label='Ia')
plt.plot(x,Ib,label='Ib')
plt.plot(x,Ic,label='Ic')
plt.legend()

#选择画纸2
ax2=plt.subplot(3,1,2)
#在画纸2上绘图
plt.plot(x,current_alpha,label='Ia')
plt.plot(x,current_beta,label='Ib')
plt.legend()

ax3=plt.subplot(3,1,3)
plt.plot(x,current_q,label='Iq')
plt.plot(x,current_d,label='Id')
plt.legend()
#显示图像
plt.show()


# plt.plot(x,current_q)
# plt.plot(x,current_d)
# plt.show()
print(total)


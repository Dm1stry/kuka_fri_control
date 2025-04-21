import numpy as np
import matplotlib.pyplot as plt

f = np.loadtxt("build/commanded_position.txt", delimiter="\t")
f = np.loadtxt("build/actual_position.txt", delimiter="\t")
# f = np.loadtxt("build/delta_position.txt", delimiter="\t")

print(f*180/np.pi)

q = f
dq = np.array([[0, 0, 0, 0, 0, 0, 0]])

for i in range(len(q)-1):
    dq = np.vstack((dq, (q[i+1]-q[i])/0.005))

plt.figure(0)
plt.plot(q)
plt.legend(["1","2","3","4","5","6","7" ])

plt.figure(1)
plt.plot(dq)
plt.legend(["1","2","3","4","5","6","7" ])
plt.show()
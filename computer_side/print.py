import numpy as np
import matplotlib.pyplot as plt

data = np.genfromtxt("build/Logs.txt", delimiter=",")

print(data)

t_target_q = data[data[:,0]==0, 1]
target_q = data[data[:,0]==0,2:]

t_current_q = data[data[:,0]==1, 1]
current_q = data[data[:,0]==1,2:]

t_target_torque = data[data[:,0]==2, 1]
target_torque = data[data[:,0]==2,2:]

t_current_torque = data[data[:,0]==3, 1]
current_torque = data[data[:,0]==3,2:]

derivative_current_q = np.gradient(current_q)
derivative_target_q = np.gradient(target_q)

print(target_q.shape)

plt.figure(0)
plt.plot(t_target_q, target_q)
plt.plot(t_current_q, current_q)
plt.legend(["target 0", "target 1", "target 2", "target 3", "target 4", "target 5", "target 6",
            "current 0", "current 1", "current 2", "current 3", "current 4", "current 5", "current 6"])

plt.figure(1)
plt.plot(t_target_torque[:], target_torque[:, 6])
# plt.plot(t_current_torque[:], current_torque)
plt.legend(["target 0", "target 1", "target 2", "target 3", "target 4", "target 5", "target 6",
            "current 0", "current 1", "current 2", "current 3", "current 4", "current 5", "current 6"])

# plt.figure(3)
# plt.plot(t_target_q[:], derivative_target_q[0][:,0])
# plt.plot(t_current_q[:], derivative_current_q[0][:,0])
# plt.legend(["t_0", "t_1", "t_2", "c_0", "c_1", "c_2"])


plt.show()



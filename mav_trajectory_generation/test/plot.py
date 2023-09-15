import numpy as np
import matplotlib.pyplot as plt


traj = np.loadtxt("../build/trajectory.csv", delimiter=",")
print(traj)
plt.figure()

ax = plt.axes(projection="3d")

pos = traj[:,:3]
vel = traj[:,3:6]
acc = traj[:,6:9]

ax.plot(pos[:,0], pos[:,2],pos[:,1], "bo")
ax.plot(pos[:,0], pos[:,2],pos[:,1], "r-")
ax.quiver(pos[:,0], pos[:,2], pos[:,1], vel[:,0], vel[:,2], vel[:,1], color="gray", arrow_length_ratio=0.1)
ax.quiver(pos[:,0], pos[:,2], pos[:,1], acc[:,0], acc[:,2], acc[:,1], color="black", arrow_length_ratio=0.1)

plt.show()
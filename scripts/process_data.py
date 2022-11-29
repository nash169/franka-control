#!/usr/bin/env python
# encoding: utf-8

import numpy as np
import sys
import os
import matplotlib.pyplot as plt

dataset = sys.argv[1]

data = np.loadtxt(os.path.join('outputs', '{}.csv'.format(dataset + "_2")))
attractor = data[-1, 14:20]
data[:, 14:20] -= attractor
for i in range(3, int(sys.argv[2]) + 1):
    traj = np.loadtxt(os.path.join(
        'outputs', '{}.csv'.format(dataset + "_"+str(i))))
    traj[:, 14:20] -= traj[-1, 14:20]
    data = np.append(data, traj, axis=0)

# data = data[200:-600, :]
data[:, 14:20] += attractor
np.savetxt('outputs/robot_demo.csv', data[:, 14:26])

joint_pos = data[:, :7]
joint_vel = data[:, 7:14]
task_pose = data[:, 14:20]
task_vel = data[:, 20:26]
torques = data[:, 26:]

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
step = 10
ax.scatter(task_pose[::step, 0], task_pose[::step, 1], task_pose[::step, 2])
ax.quiver(task_pose[::step, 0], task_pose[::step, 1], task_pose[::step, 2],
          task_vel[::10, 0], task_vel[::step, 1], task_vel[::step, 2], length=0.01, normalize=True, color='r')

plt.show()

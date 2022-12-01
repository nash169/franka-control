#!/usr/bin/env python
# encoding: utf-8

import numpy as np
import sys
import os
import matplotlib.pyplot as plt

dataset = sys.argv[1]

# data = np.loadtxt(os.path.join('outputs', '{}.csv'.format(dataset + "_2")))
# attractor = data[-1, 14:20]
# data[:, 14:20] -= attractor
# for i in range(3, int(sys.argv[2]) + 1):
#     traj =
#     traj[:, 14:20] -= traj[-1, 14:20]
#     data = np.append(data, traj, axis=0)

dim = 3
dt = 0.01
data = np.empty([1, 9])
cut = 100
for i in range(1,  int(sys.argv[2]) + 1):
    traj = np.loadtxt(os.path.join(
        'outputs', '{}.csv'.format(dataset + "_"+str(i))))[cut:, :]
    pos = traj[:, 14:17] - traj[-1, 14:17]
    vel = traj[:, 20:23]
    acc = np.divide(vel[1:, :] - vel[:-1, :], dt)
    acc = np.append(acc, np.zeros([1, dim]), axis=0)
    data = np.append(data, np.concatenate((pos, vel, acc), axis=1), axis=0)
data = data[1:, :]

task_pose = data[:, :3]
task_vel = data[:, 3:6]
task_acc = data[:, 6:]

np.savetxt('outputs/robot_demo.csv', data)

# data = data[200:-600, :]
# data[:, 14:20] += attractor


# joint_pos = data[:, :7]
# joint_vel = data[:, 7:14]
# task_pose = data[:, 14:20]
# task_vel = data[:, 20:26]
# torques = data[:, 26:]

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
step = 10
ax.scatter(task_pose[::step, 0], task_pose[::step, 1], task_pose[::step, 2])
ax.quiver(task_pose[::step, 0], task_pose[::step, 1], task_pose[::step, 2],
          task_vel[::10, 0], task_vel[::step, 1], task_vel[::step, 2], length=0.01, normalize=True, color='r')
ax.quiver(task_pose[::step, 0], task_pose[::step, 1], task_pose[::step, 2],
          task_acc[::10, 0], task_acc[::step, 1], task_acc[::step, 2], length=0.01, normalize=True, color='g')

plt.show()

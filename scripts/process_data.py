#!/usr/bin/env python
# encoding: utf-8

import numpy as np
import pickle
import os
import matplotlib.pyplot as plt


num_records = 7
record_list = []

for i in range(1,  num_records + 1):
    traj = np.loadtxt(os.path.join(
        'outputs', '{}.csv'.format("record_"+str(i))))

    demo_dict = {
        'joint_position': traj[:, :7],
        'joint_velocity': traj[:, 7:14],
        'ee_pose': traj[:, 14:20],
        'ee_velocity': traj[:, 20:26],
        'torque': traj[:, 26:],
        'dt': 0.01
    }

    record_list.append(demo_dict)

# save
with open('outputs/robotic_demo.pkl', 'wb') as fp:
    pickle.dump(record_list, fp)

# load
with open('outputs/robotic_demo.pkl', 'rb') as fp:
    data = pickle.load(fp)

# plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
for traj in data:
    ax.scatter(traj['ee_pose'][:, 0], traj['ee_pose']
               [:, 1], traj['ee_pose'][:, 2])
plt.show()

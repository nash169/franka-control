#!/usr/bin/env python
# encoding: utf-8

import numpy as np
import pickle
import os
import matplotlib.pyplot as plt


num_records = 1
record_list = []

for i in range(1,  num_records + 1):
    traj = np.loadtxt(os.path.join(
        'outputs', '{}.csv'.format("record_"+str(i))))

    dict = {
        'joint_position': traj[:, :7],
        'joint_velocity': traj[:, 7:14],
        'ee_pose': traj[:, 14:20],
        'ee_velocity': traj[:, 20:26],
        'torque': traj[:, 26:],
        'dt': 0.01
    }

    record_list.append(dict)


with open('robotic_demo.pkl', 'wb') as fp:
    pickle.dump(traj, fp)

#!/usr/bin/env python
# encoding: utf-8

import numpy as np
from zmq_stream.replier import Replier

rep = Replier()
rep.configure("0.0.0.0", "5511")

xR = np.array([0.683783, 0.308249, 0.185577])


def dynamics(x):
    return -10.0*(x-xR)


while True:
    x = rep.reply(dynamics, np.float64, 3)

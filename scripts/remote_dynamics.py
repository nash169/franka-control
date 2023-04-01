#!/usr/bin/env python
# encoding: utf-8

import numpy as np
from zmq_stream.replier import Replier

rep = Replier()
rep.configure("0.0.0.0", "5511")

xR = np.array([0.683783, 0.308249, 0.185577])

while True:
    x = rep.receive(np.float64, 3)
    xdot = -10.0*(x-xR)
    rep.send(xdot)
    print(xdot)

#!/usr/bin/env python

from __future__ import division
from collections import defaultdict

import morsesim
import time

class RoboticsError(Exception):
    pass

class BaseRobot(object):
    def __init__(self, components, name='dummy', host='localhost'):
        self.components = components

        self.actuators = defaultdict(list)
        self.sensors = defaultdict(list)
        self.name = name
        self.host = host

        self._create_devices(components)



class NeuralRobot(BaseRobot):
    def __init__(self, components, network, name='dummy', host='localhost'):
        self.network = network
        super(NeuralRobot, self).__init__(components, name, host)

#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" 
坐标系旋转与平移
"""

import numpy as np

class GeneralConverter:
    def __init__(self, R, t):
        self.R = R        
        self.t = t
    
    def world_to_robot(self, point):
        point = point - self.t
        point = np.dot(self.R.transpose(),point.reshape(2,1)).reshape(2)
        return point
    
    def robot_to_world(self, point):
        point = np.dot(self.R,point.reshape(2,1))
        point = point.reshape(2) + self.t
        return point

    def vel_world_to_robot(self, vel):
        vel = np.dot(self.R.transpose(),vel.reshape(2,1)).reshape(2)
        return vel





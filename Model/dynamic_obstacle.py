#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
动态障碍物
"""

import numpy as np
from vehicle_box import VehicleBox


class DynamicObstacle():
    def __init__(self, ob_pos, ob_velocity, ob_dist):
        self.ob_pos_list = []
        self.ob_velocity = ob_velocity
        self.ob_dist = ob_dist
        self.ob_pos_list.append(ob_pos)
        self.eval_time = 3
        self.eval_time_interval = 0.5
        self.get_trajectory()

    def get_trajectory(self):
        t = self.eval_time_interval
        while t <= self.eval_time:
            pre_ob_pos = self.ob_pos_list[-1]
            next_pos = pre_ob_pos + self.ob_velocity*self.eval_time_interval  
            self.ob_pos_list.append(next_pos)
            t += self.eval_time_interval

    def cal_min_dist(self,point):
        t = 0
        ind = 0
        tmp_dist = []
        while t <= self.eval_time:
            ob_box = VehicleBox(self.ob_pos_list[ind],self.ob_velocity,self.ob_dist)
            tmp_dist.append(ob_box.cal_dist(point))
            t += self.eval_time_interval
            ind += 1
        return min(tmp_dist)


        
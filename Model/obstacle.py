#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
障碍物
"""

from matplotlib.pyplot import cla
import numpy as np
from Utils.tool import cal_dist

class VehicleBox:
    def __init__(self, pos, vel, radius):
        self.pos = pos
        self.vel = vel
        self.radius = radius

    def cal_dist(self, point):
        return cal_dist(self.pos, point)


class ObstacleTrajectory:
    def __init__(self, ob_dist):
        self.ob_pos_list = []
        self.ob_dist = ob_dist
        self.dt = 0.5


class DynamicObstacle:
    def __init__(self, ob_pos, ob_vel, ob_dist):
        self.ob_pos_list = []
        self.ob_vel = ob_vel
        self.ob_dist = ob_dist
        self.ob_pos_list.append(ob_pos)
        self.eval_time = 5
        self.dt = 0.5
        self.get_trajectory()

    def get_trajectory(self):
        # t = ind*dt
        t = self.dt
        while t <= self.eval_time:
            pre_ob_pos = self.ob_pos_list[-1]
            next_pos = pre_ob_pos + self.ob_vel*self.dt  
            self.ob_pos_list.append(next_pos)
            t += self.dt

    def cal_min_dist(self,point):
        t = 0
        ind = 0
        tmp_dist = []
        while t <= self.eval_time:
            ob_box = VehicleBox(self.ob_pos_list[ind],self.ob_vel,self.ob_dist)
            tmp_dist.append(ob_box.cal_dist(point))
            t += self.dt
            ind += 1
        return min(tmp_dist)

    def cal_dist(self,point,t):
        ind = int(1.0*t/self.dt)
        if t <= self.eval_time:
            ob_box = VehicleBox(self.ob_pos_list[ind],self.ob_vel,self.ob_dist)
            return ob_box.cal_dist(point)
        else:
            print("t is out of prediction range.")
            return 0.0

    def info(self):
        print("pos: ",)
        print(self.ob_pos_list[0])
        print("vel: ",)
        print(self.ob_vel)



        
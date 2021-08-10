#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" 
SL地图 
"""

import numpy as np
import math
import matplotlib.pyplot as plt
from Model.obstacle import VehicleBox


class STMap():
    def __init__(self, s_end, t_end=5):
        self.ds = 2
        self.dt = 0.5
        self.n_s = int(math.ceil(1.0*s_end/self.ds))+1
        self.n_t = int(math.ceil(1.0*t_end/self.dt))+1
        self.s_end = self.n_s * self.ds
        self.t_end = self.n_t * self.dt
        self.map = [[PointState(i*self.dt, j*self.ds) for j in range(self.n_s)] for i in range(self.n_t)]
        self.ob_mat = []

    def add_obstacle(self, path, static_ob, dynamic_ob):
        for i in range(self.n_t):
            t = i * self.dt
            ob_state = ObState(t)
            for ob in dynamic_ob:
                s_down = -1
                ob_box = VehicleBox(ob.ob_pos_list[i],0.0,ob.ob_dist)
                for path_point in path:
                    dist = ob_box.cal_dist(path_point)
                    if dist < ob_box.radius:   # 当前位置会碰到
                        if s_down == -1:
                            s_down = path_point[0]
                            s_up = s_down
                        else:
                            s_up = path_point[0]
                if s_down != -1:
                    ob_state.s_down.append(s_down)
                    ob_state.s_up.append(s_up)
            self.ob_mat.append(ob_state)

    def show(self):
        for i in range(self.n_t):
            ob = self.ob_mat[i]
            for j in range(self.n_s):
                t = self.map[i][j].t
                s = self.map[i][j].s
                ob_flag = False
                for k in range(len(ob.s_down)):
                    if s >= ob.s_down[k] and s <= ob.s_up[k]:
                        ob_flag = True
                        break
                if ob_flag:
                    plt.scatter(t, s, c='red')
                else:
                    plt.scatter(t, s, c='green')
                
                

class ObState():
    def __init__(self, t):
        self.t = t
        self.s_down = []
        self.s_up = []

class PointState():
    def __init__(self, t, s):
        self.t = t
        self.s = s
        self.cost = 0.0
        self.pre_ind = -1

        
    
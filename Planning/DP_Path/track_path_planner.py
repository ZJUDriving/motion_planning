#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
道路中心线跟随路径规划器
"""


import numpy as np
import matplotlib.pyplot as plt

from Utils.tool import DRAW_SL_FIG
from Model.curve import Curve


class TrackPathPlanner():
    def __init__(self, sl_map):
        self.sl_map = sl_map
        self.d_s = 0.5              # 插值间隔

    def plan(self):
        if DRAW_SL_FIG: # 绘制Frenet系采样地图
            self.sl_map.draw_sl_fig()
        
        # path_found = self.find_path()
        path_found = True
        curve_path = []
        if path_found:
            path_s = []
            path_l = []
            path_s.append(self.sl_map.ego_point[0])
            path_l.append(self.sl_map.ego_point[1])
            for s in range(self.sl_map.n_s):
                tmp_l = self.sl_map.mid_l
                path_s.append(self.sl_map.s_map[s][tmp_l])
                path_l.append(self.sl_map.l_map[s][tmp_l])
                if DRAW_SL_FIG:    # 选取的路径点
                    plt.scatter(path_s[-1],path_l[-1],c='red')

            # 路径五次样条插值
            path_s = np.array(tuple(path_s))
            path_l = np.array(tuple(path_l))
            curve_path = Curve(path_s,self.d_s,path_l,0.0)
        return path_found, curve_path


        


#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" 
机器人坐标系地图 
"""

import numpy as np
import matplotlib.pyplot as plt

from Utils.tool import to_point, save_fig, check_static, DRAW_ROBOT_FIG
from Model.obstacle import *
from Model.general_converter import GeneralConverter

class RobotMap():
    def __init__(self, R, t):
        self.WRC = GeneralConverter(R,t) 
        # 机器人坐标系到世界坐标系的转移矩阵，TODO：真实情况下矩阵应该根据位置实时更新
        self.ori_point = np.array([0.0,0.0])
        self.ignore_dist = 20       # 视野范围之外的障碍物直接不计入RobotMap
        self.static_ob = []
        self.dynamic_ob = []
        self.robot = []

    """ 添加车道参考线 """
    def add_robot(self, rob_pos, rob_vel, rob_dist):
        rob_pos = self.WRC.world_to_robot(rob_pos)
        rob_vel = self.WRC.vel_world_to_robot(rob_vel)
        self.robot = VehicleBox(rob_pos, rob_vel, rob_dist)

    """ 添加车道参考线 """
    def add_ref_line(self, ref_line, l_width, n_l, n_s, cal_theta_ind=0):
        self.n_l = n_l
        self.n_s = n_s
        self.l_width = l_width
        self.rx_list = []
        self.ry_list = []
        for world_point in ref_line:
            point = self.WRC.world_to_robot(world_point)
            self.rx_list.append(point[0])
            self.ry_list.append(point[1])
        self.rx_list.insert(0,0.0)   # TODO:直接用当前位置作为Frenet系原点？
        self.ry_list.insert(0,0.0)
        self.line_vec = 0.0

    """ 添加单个障碍物 """
    def add_obstacle(self, ob_pos, ob_vel, ob_dist):
        ob_pos = self.WRC.world_to_robot(ob_pos)
        if ob_pos[0] < -1:  # TODO：判断是否在车后方，x<-1，即在车后方，比较暴力
            return False
        if check_static(ob_vel):  # 静态障碍物      
            ob_to_ori = cal_dist(self.ori_point,ob_pos)
            if ob_to_ori < self.ignore_dist:
                print("Static obstacle dist: %.4f" % ob_to_ori)
                self.static_ob.append(StaticObstacle(ob_pos,ob_vel,ob_dist))
                return True
            else:
                return False
        else: # 动态障碍物
            ob_vel = self.WRC.vel_world_to_robot(ob_vel)
            dy_ob = DynamicObstacle(ob_pos,ob_vel,ob_dist)
            ob_to_ori = dy_ob.cal_min_dist(self.ori_point)
            if ob_to_ori < self.ignore_dist:
                print("Dynamic obstacle min_dist: %.4f" % ob_to_ori)
                self.dynamic_ob.append(dy_ob)
                return True
            else:
                return False

    """ 根据已有参考线的起点进行延长至得到机器人投影点（暂时没用） """
    def extend_line(self, rx_list, ry_list, cal_theta_ind):
        va = rx_list[cal_theta_ind+1]-rx_list[cal_theta_ind]
        vb = ry_list[cal_theta_ind+1]-ry_list[cal_theta_ind]
        x0 = rx_list[0]
        y0 = ry_list[0]
        t = -(va*x0+vb*y0)*1.0/(va**2+vb**2)
        x = x0 + va*t
        y = y0 + vb*t
        new_point = np.array([x,y])     # 根据机器人当前位置添加的延长线上的点
        line_vec = vb / va              # 延长线斜率，作为五次多项式插值的中间点斜率
        return new_point, line_vec

    """ Robot转World """
    def path_convert(self, point_list):
        world_point_list = []
        for point in point_list:
            if DRAW_ROBOT_FIG:
                plt.scatter(point[0], point[1], c='red')
            node = self.WRC.robot_to_world(to_point(point[0], point[1]))
            world_point_list.append(node)
        return world_point_list

    def save_robot_fig(self):
        pass
        if DRAW_ROBOT_FIG:
            # plt.show()
            save_fig()



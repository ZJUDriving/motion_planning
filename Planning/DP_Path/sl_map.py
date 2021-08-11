#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" 
SL地图 
"""

import numpy as np
import math
import matplotlib.pyplot as plt

from Utils.tool import to_point, DRAW_ROBOT_FIG
from Model.cartesian_frenet_conversion import CartesianFrenetConverter
from Model.obstacle import *

class SLMap():
    def __init__(self, robot_map):
        self.RMP = robot_map    # 机器人坐标系
        self.n_l = robot_map.n_l
        self.n_s = robot_map.n_s
        self.mid_l = int((self.n_l-1)/2)
        self.l_width = robot_map.l_width
        self.ob_list = []       # 障碍物中心位置列表（包含动态障碍物的每一帧）
        self.ob_dist = 0.0      # TODO:障碍物半径目前直接取最大的   
        self.d_s = 8            # 参考线采样间隔
        self.dy_ob_traj = []    # 每个动态障碍物的轨迹记录
        self.create_frenet_map()
        self.add_obstacle()
        
        
    """ 创建Frenet栅格地图 """
    def create_frenet_map(self):
        # 插值得到参考线轨迹，并建立Frenet坐标系
        rx_list = self.RMP.rx_list
        ry_list = self.RMP.ry_list
        line_vec = self.RMP.line_vec    # 起点速度
        self.converter = CartesianFrenetConverter(0.0,0.0,np.array(rx_list),np.array(ry_list),line_vec)
        so,lo = self.converter.cartesian_to_frenet(0.0,0.0)
        self.ego_point = np.array([so,lo])
        self.s_map = []
        self.l_map = []
        if DRAW_ROBOT_FIG:
            plt.figure()
            self.converter.show()
        # 对参考线进行等间距采样
        end_s = self.converter.get_s(rx_list[-1])
        ss = self.d_s
        s_list = []     # 参考线上的采样点
        while True:
            s_list.append(ss)
            ss += self.d_s
            if ss+self.d_s/2 >= end_s:
                s_list.append(end_s)
                break
        # 对Frenet坐标系进行栅格采样
        self.n_s = len(s_list)
        for s in s_list:
            s_line = s*np.ones((1,self.n_l))
            l_line = np.linspace(0.0-self.l_width/2, 0.0+self.l_width/2, num=self.n_l).reshape(1,self.n_l)
            self.s_map.append(s_line)
            self.l_map.append(l_line)
            if DRAW_ROBOT_FIG:
                rx,ry = self.converter.frenet_to_cartesian(s,0.0)
                plt.scatter(rx,ry,c='yellow')
        self.s_map = np.concatenate(tuple(self.s_map),axis=0).reshape(self.n_s,self.n_l)
        self.l_map = np.concatenate(tuple(self.l_map),axis=0).reshape(self.n_s,self.n_l)
        # print(self.s_map)
        # print(self.l_map)
        

    def add_obstacle(self):
        # TODO:静态障碍物未添加轨迹
        for st_ob in self.RMP.static_ob:
            ob_pos = st_ob.ob_pos
            s,l = self.converter.cartesian_to_frenet(ob_pos[0],ob_pos[1])
            ob_point = np.array([s,l])
            self.ob_list.append(ob_point)
            self.ob_dist = max(self.ob_dist, st_ob.ob_dist)
            if DRAW_ROBOT_FIG:  # 绘制障碍物在机器人坐标系下的动态图
                self.draw_circle(ob_pos[0],ob_pos[1],self.ob_dist,'green')

        for dy_ob in self.RMP.dynamic_ob:
            ob_trajectory = ObstacleTrajectory(dy_ob.ob_dist)
            for ob_pos in dy_ob.ob_pos_list:
                s,l = self.converter.cartesian_to_frenet(ob_pos[0],ob_pos[1])
                ob_point = np.array([s,l])
                ob_trajectory.ob_pos_list.append(ob_point)
                if self.check_in_line(ob_point, dy_ob.ob_dist):  # 对应SL图，只添加会影响规划结果的障碍物帧
                    self.ob_list.append(ob_point)
                    self.ob_dist = max(self.ob_dist, dy_ob.ob_dist)
                    if DRAW_ROBOT_FIG:  # 绘制障碍物在机器人坐标系下的动态图
                        self.draw_circle(ob_pos[0],ob_pos[1],self.ob_dist,'green')
            self.dy_ob_traj.append(ob_trajectory)

    def check_in_line(self, point, radius):
        return abs(point[1]) - radius < self.l_width

    """ Frenet转Robot """
    def path_convert(self, point_list):
        robot_point_list = []
        for point in point_list:
            rx, ry = self.converter.frenet_to_cartesian(point[0],point[1])
            robot_point_list.append(to_point(rx, ry))
        return robot_point_list

    def draw_sl_fig(self):
        plt.figure()
        s_list = self.s_map.reshape(1,self.n_s*self.n_l)
        l_list = self.l_map.reshape(1,self.n_s*self.n_l)
        plt.scatter(s_list, l_list, c='blue')
        plt.scatter(self.ego_point[0], self.ego_point[1], c='yellow')
        for ob_point in self.ob_list:
            if abs(ob_point[1]) - self.ob_dist < self.l_width:     # 是否绘制障碍物
                self.draw_circle(ob_point[0],ob_point[1],self.ob_dist,'green')
        
    def draw_circle(self,x,y,R,color='green'):
        th = np.arange(0,2*math.pi,math.pi/20)
        circle_x = x + R*np.cos(th)
        circle_y = y + R*np.sin(th)
        plt.scatter(x,y,c=color)
        plt.scatter(circle_x,circle_y,c=color)


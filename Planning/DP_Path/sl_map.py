#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" 
SL地图 
"""

import numpy as np
import math
import matplotlib.pyplot as plt

from Utils.tool import to_point, save_fig, check_static, DRAW_ROBOT_FIG
from Model.cartesian_frenet_conversion import CartesianFrenetConverter
from Model.obstacle import *
from Model.world_robot_conversion import WorldRobotConverter

class SLMap():
    def __init__(self, R, t):
        self.WRC = WorldRobotConverter(R,t) 
        # 机器人坐标系到世界坐标系的转移矩阵，TODO：真实情况下矩阵应该根据位置实时更新
        self.final_point = []
        self.ob_point = []
        self.ob_list = []
        self.dy_ob_frame = []
        self.ignore_dist = 20
        self.ob_dist = 0.0
        self.st_l = 0       # Frenet系下规划第一列下标     
        self.d_s = 8
        self.dynamic_ob = []
        
    """ 添加车道参考线，同时创建Frenet栅格地图 """
    def add_ref_line(self, ref_line, l_width, n_l, n_s, cal_theta_ind=0):
        self.n_l = n_l
        self.n_s = n_s
        self.mid_l = int((self.n_l-1)/2)
        self.l_width = l_width
        rx_list = []
        ry_list = []
        for world_point in ref_line:
            point = self.WRC.world_to_robot(world_point)
            rx_list.append(point[0])
            ry_list.append(point[1])
        # new_point, line_vec = self.extend_line(rx_list,ry_list,cal_theta_ind)
        # rx_list.insert(0,new_point[0])
        # ry_list.insert(0,new_point[1])
        rx_list.insert(0,0.0)   # TODO:直接用当前位置作为Frenet系原点？
        ry_list.insert(0,0.0)
        line_vec = 0.0
        # print(rx_list)
        # plt.figure()
        # 创建Frenet栅格地图
        self.create_frenet_map(rx_list, ry_list, line_vec)

    def create_frenet_map(self, rx_list, ry_list, line_vec):
        self.converter = CartesianFrenetConverter(0.0,0.0,np.array(rx_list),np.array(ry_list),line_vec)
        so,lo = self.converter.cartesian_to_frenet(0.0,0.0)
        self.ego_point = np.array([so,lo])
        self.s_map = []
        self.l_map = []
        if DRAW_ROBOT_FIG:
            plt.figure()
            self.converter.show()
            # plt.scatter(0.0,0.0,c='yellow')
            # for i in range(len(rx_list)):
            #     plt.scatter(rx_list[i],ry_list[i],c='red')
        end_s = self.converter.get_s(rx_list[-1])
        ss = self.d_s
        s_list = []
        while True:
            s_list.append(ss)
            ss += self.d_s
            if ss+self.d_s/2 >= end_s:
                s_list.append(end_s)
                break
        self.n_s = len(s_list)
        for s in s_list:
            s_line = s*np.ones((1,self.n_l))
            l_line = np.linspace(0.0-self.l_width/2, 0.0+self.l_width/2, num=self.n_l).reshape(1,self.n_l)
            self.s_map.append(s_line)
            self.l_map.append(l_line)
            # plt.scatter(rx,ry_list[i],c='green')
            if DRAW_ROBOT_FIG:
                rx,ry = self.converter.frenet_to_cartesian(s,0.0)
                plt.scatter(rx,ry,c='yellow')
        """
        for i in range(1,len(rx_list)):
            rx = rx_list[i]
            s = self.converter.get_s(rx)
            s_line = s*np.ones((1,self.n_l))
            l_line = np.linspace(0.0-self.l_width/2, 0.0+self.l_width/2, num=self.n_l).reshape(1,self.n_l)
            self.s_map.append(s_line)
            self.l_map.append(l_line)
            # plt.scatter(rx,ry_list[i],c='green')
            if DRAW_ROBOT_FIG:
                plt.scatter(rx,ry_list[i],c='green')
        """
        self.s_map = np.concatenate(tuple(self.s_map),axis=0).reshape(self.n_s,self.n_l)
        self.l_map = np.concatenate(tuple(self.l_map),axis=0).reshape(self.n_s,self.n_l)
        # print(self.s_map)
        # print(self.l_map)
        

    def add_obstacle(self, ob_pos, ob_dist,ob_vel):
        ob_pos = self.WRC.world_to_robot(ob_pos)
        if ob_pos[0] < -1:  # x<-1，即在车后方
            return False
        if check_static(ob_vel):        
            ob_to_ori = cal_dist(self.ego_point,ob_pos)
            if ob_to_ori < self.ignore_dist:
                print("Static obstacle dist: %.4f" % ob_to_ori)
                s,l = self.converter.cartesian_to_frenet(ob_pos[0],ob_pos[1])
                ob_point = np.array([s,l])
                self.ob_list.append(ob_point)
                self.ob_dist = max(self.ob_dist, ob_dist)
                return True
            else:
                return False
        else: # 动态障碍物
            ob_vel = self.WRC.vel_world_to_robot(ob_vel)
            dy_ob = DynamicObstacle(ob_pos,ob_vel,ob_dist)
            ob_to_ori = dy_ob.cal_min_dist(self.ego_point)
            if ob_to_ori < self.ignore_dist:
                print("Dynamic obstacle min_dist: %.4f" % ob_to_ori)
                # dy_ob.info()
                ob_trajectory = ObstacleTrajectory(ob_dist)
                for ob_pos in dy_ob.ob_pos_list:
                    s,l = self.converter.cartesian_to_frenet(ob_pos[0],ob_pos[1])
                    ob_point = np.array([s,l])
                    ob_trajectory.ob_pos_list.append(ob_point)
                    if abs(ob_point[1]) - self.ob_dist < self.l_width:  # 对应SL图，只添加会影响规划结果的障碍物帧
                        self.ob_list.append(ob_point)
                        self.ob_dist = max(self.ob_dist, ob_dist)
                        if DRAW_ROBOT_FIG:  # 绘制障碍物在机器人坐标系下的动态图
                            self.draw_circle(ob_pos[0],ob_pos[1],self.ob_dist,'green')
                self.dynamic_ob.append(ob_trajectory)
                return True
            else:
                return False

    """ 根据已有参考线的起点进行延长至得到机器人投影点 """
    def extend_line(self, rx_list, ry_list, cal_theta_ind):
        va = rx_list[cal_theta_ind+1]-rx_list[cal_theta_ind]
        vb = ry_list[cal_theta_ind+1]-ry_list[cal_theta_ind]
        x0 = rx_list[0]
        y0 = ry_list[0]
        # print(va,vb)
        # print(x0,y0)
        t = -(va*x0+vb*y0)*1.0/(va**2+vb**2)
        # print(t)
        x = x0 + va*t
        y = y0 + vb*t
        # print(x,y)    
        new_point = np.array([x,y])     # 根据机器人当前位置添加的延长线上的点
        line_vec = vb / va              # 延长线斜率，作为五次多项式插值的中间点斜率
        return new_point, line_vec

    def frenet_to_world(self, point_list):
        world_point_list = []
        for point in point_list:
            rx, ry = self.converter.frenet_to_cartesian(point[0],point[1])
            if DRAW_ROBOT_FIG:
                plt.scatter(rx,ry,c='red')
            node = self.WRC.robot_to_world(to_point(rx, ry))
            world_point_list.append(node)
        return world_point_list

    def draw_sl_fig(self):
        plt.figure()
        # [1:self.n_s+1][:]
        s_list = self.s_map.reshape(1,self.n_s*self.n_l)
        l_list = self.l_map.reshape(1,self.n_s*self.n_l)
        plt.scatter(s_list,l_list,c='blue')
        plt.scatter(self.ego_point[0],self.ego_point[1],c='yellow')
        # plt.scatter(self.final_point[0],self.final_point[1],c='yellow')
        
        for ob_point in self.ob_list:
            # plt.scatter(self.start_point[0],self.start_point[1],c='red')
            if abs(ob_point[1]) - self.ob_dist < self.l_width:     # 是否绘制障碍物
                self.draw_circle(ob_point[0],ob_point[1],self.ob_dist,'green')
                # th = np.arange(0,2*math.pi,math.pi/20)
                # circle_x = ob_point[0] + self.ob_dist*np.cos(th)
                # circle_y = ob_point[1] + self.ob_dist*np.sin(th)
                # plt.scatter(ob_point[0],ob_point[1],c='green')
                # plt.scatter(circle_x,circle_y,c='green')
        # plt.show()
        # time.sleep(100)

    def save_robot_fig(self):
        pass
        if DRAW_ROBOT_FIG:
            # plt.show()
            save_fig()
        
    def draw_circle(self,x,y,R,color='green'):
        th = np.arange(0,2*math.pi,math.pi/20)
        circle_x = x + R*np.cos(th)
        circle_y = y + R*np.sin(th)
        plt.scatter(x,y,c=color)
        plt.scatter(circle_x,circle_y,c=color)


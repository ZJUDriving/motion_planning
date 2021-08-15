#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np
from curve import *
import matplotlib.pyplot as plt
from Utils.tool import get_arange

class CartesianFrenetConverter:
    def __init__(self,ex,ey,rx_list,ry_list,vec):
        self.d_x = 0.1          # 量化间隔
        # 找到原点
        # T_x = np.diff(rx_list,n=1)      # 计算x维度间隔
        self.ref_curve = Curve(rx_list,self.d_x,ry_list,vec)
        # xx = np.arange(rx_list[0],rx_list[-1],self.d_x)
        # plt.plot(xx,self.ref_curve.calc_point_arr(xx,0))
        # plt.scatter(ex,ey,c='red')
        # # plt.show()
        # print(rx_list)
        # print(T_x)
        # print(ry_list)
        # print(vec)
        min_dist, min_p = self.ref_curve.projection(ex,ey)
        self.rx_ori = min_p[0] # Frenet原点对应笛卡尔系x
        self.ry_ori = min_p[1]
        self.rx_max = rx_list[-1]
        # print("ori")
        # print(ex,ey)
        # print(self.rx_ori,self.ry_ori)
       
    def cartesian_to_frenet(self, x, y, vx=0.0, vy=0.0, order=0):
        # 计算投影点
        min_dist, min_p = self.ref_curve.projection(x,y)
        rx = min_p[0]
        ry = min_p[1]
        rtheta = np.arctan(self.ref_curve.calc_point(rx,1))
        dx = x - rx
        dy = y - ry
        if dy * math.cos(rtheta) - dx * math.sin(rtheta) > 0:
            l_sign = 1.0
        else:
            l_sign = -1.0
        l = l_sign * min_dist
        s = self.get_s(rx)
        if order == 0:
            return s, l
        else:
            kr = self.get_curvature(rx)
            vtheta = math.atan2(vy,vx)
            speed = math.sqrt(vx**2+vy**2)
            s_dt = speed * math.cos(vtheta - rtheta) / (1 - kr * l)
            return s, l, s_dt

    def frenet_to_cartesian(self,s,l):
        rx,ry = self.get_rxy(s)
        rtheta = np.arctan(self.ref_curve.calc_point(rx,1))
        x = rx - l * math.sin(rtheta)
        y = ry + l * math.cos(rtheta)
        return x,y

    def get_curvature(self, rx):
        dl = self.ref_curve.calc_point(rx, 1)
        ddl = self.ref_curve.calc_point(rx, 2)
        kr = 1.0*abs(ddl)/((1+dl**2)**1.5)
        return kr

    
    def get_s(self, rx):
        return self.ref_curve.calc_arc_len(self.rx_ori, rx)

    def get_rxy(self, s):
        rx_list = get_arange(self.rx_ori,self.rx_max,self.d_x) # np.arange(self.rx_ori,self.rx_max,self.d_x) #  # 
        found = False
        for i in range(len(rx_list)):
            rx_next = rx_list[i]
            tmp = self.ref_curve.calc_arc_len(self.rx_ori, rx_next)
            if s <= tmp:
                rx = rx_next
                found = True
                break
        if not found:
            rx = self.rx_max
            print("Error, cannot get rxy")
        ry = self.ref_curve.calc_point(rx,0)
        return rx,ry

    def show(self):
        # plt.figure()
        # plt.scatter(self.rx_ori,self.ry_ori,'green')
        rx_list = get_arange(self.rx_ori,self.rx_max,self.d_x) # np.arange(self.rx_ori,self.rx_max,self.d_x)#  # 
        ry_list = self.ref_curve.calc_point_arr(rx_list,0)
        plt.plot(rx_list,ry_list)

        
        





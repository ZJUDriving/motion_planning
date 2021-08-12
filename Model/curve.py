#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from Utils.tool import cal_dist_arr

# TODO:笛卡尔空间下轨迹那个维度的自变量x，如果是车头不垂直应该问题不大？
# TODO:把量化间隔固定，直接存储整条曲线？
# TODO:曲线三阶导数在连接点处不连续，没有考虑
# TODO:样条插值中间点的状态自适应给定？

""" 样条插值 """
class Curve:
    def __init__(self, t, dt, x, vec):   # x比T多一个数
        """
            :param T        自变量列表的一阶差分
            :param t_bios   自变量列表的起始值
            :param dt       计算投影时的离散采样间隔
            :param x        因变量列表
            :param vec      起点处的v
        """
        self.T = np.diff(t)
        self.n = len(self.T)
        self.curve_list = []
        self.dt = dt
        self.t_bios = t[0]
        # 简单起见，v和a全取0.0
        # 自适应得到v
        v = 1.0*(x[2:]-x[:-2]) / (self.T[1:]+self.T[:-1])
        v = np.insert(v,0,vec)
        v = np.append(v,vec)
        a = 1.0*(v[2:]-v[:-2]) / (self.T[1:]+self.T[:-1])
        a = np.insert(a,0,0.0)
        a = np.append(a,0.0)
        for i in range(self.n):
            # self.curve_list.append(QuinticPoly(x[i],vec,0.0,x[i+1],vec,0.0,self.T[i]))
            self.curve_list.append(QuinticPoly(x[i],v[i],a[i],x[i+1],v[i+1],a[i+1],self.T[i]))

    def calc_point_arr(self, t_arr, order):
        t_arr = t_arr - self.t_bios
        st_i, st_t = self.get_i(t_arr[0])
        res_list = []
        t_sum = np.sum(self.T[:st_i+1])
        i = st_i
        pre_j = 0
        for j in range(len(t_arr)):
            if t_arr[j] > t_sum:
                sub_res = self.curve_list[i].calc_point(t_arr[pre_j:j]-t_sum+self.T[i], order)
                res_list.append(sub_res)
                i += 1
                pre_j = j
                if i < self.n:
                    t_sum += self.T[i]
                elif i >= self.n:
                    print("Error, out of limit")
                    break
        if i >= self.n:
            i = self.n-1
        if pre_j < len(t_arr):
            sub_res = self.curve_list[i].calc_point(t_arr[pre_j:]-t_sum+self.T[i], order)
        res_list.append(sub_res)
        res_arr = np.concatenate(tuple(res_list))
        return res_arr

    def calc_point(self, t, order):
        t = t - self.t_bios
        ii, tt = self.get_i(t)
        return self.curve_list[ii].calc_point(tt,order)

    def get_i(self, t):
        t_sum = 0.0
        if t < 0.0:
            print("Error, t is smaller than 0!")
            return 0, 0.0
        for i in range(self.n): 
            t_sum += self.T[i]
            if t <= t_sum:
                return i, t-t_sum+self.T[i]
        print("Error, t is too bigger!")
        return self.n-1, t_sum

    def calc_arc_len(self,t0,t1):
        t0 = t0 - self.t_bios
        t1 = t1 - self.t_bios
        if t0 > t1:     # 起点后方的点暂时不考虑
            return 0.0
        st_i, st_t = self.get_i(t0)
        en_i, en_t = self.get_i(t1)
        if st_i == en_i:
            arc_len = self.curve_list[st_i].calc_arc_len(st_t,en_t,self.dt)
        else:
            arc_len = 0.0
            arc_len = self.curve_list[st_i].calc_arc_len(st_t,self.T[st_i],self.dt)
            for i in range(st_i+1,en_i):
                arc_len += self.curve_list[i].calc_arc_len(0,self.T[i],self.dt)
            arc_len += self.curve_list[en_i].calc_arc_len(0,en_t,self.dt)
        return arc_len
    
    def projection(self,t,x):
        p = np.array([t,x])
        arr = 0.0   # TODO：得到2*n的曲线矩阵
        t_max = np.sum(self.T)
        # print(t_max)
        t_arr = np.arange(0,t_max,self.dt) + self.t_bios
        x_arr = self.calc_point_arr(t_arr,0)
        rp_arr = np.vstack((t_arr,x_arr))
        min_dist, min_p = cal_dist_arr(rp_arr,p)
        return min_dist, min_p

""" 五次多项式插值 """
class QuinticPoly:
    def __init__(self, xs, vxs, axs, xe, vxe, axe, T):
        # 计算五次多项式系数
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.xe = xe
        self.vxe = vxe
        self.axe = axe

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        # A = np.array([[T ** 3, T ** 4, T ** 5],
        #               [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
        #               [6 * T, 12 * T ** 2, 20 * T ** 3]])
        # b = np.array([xe - self.a0 - self.a1 * T - self.a2 * T ** 2,
        #               vxe - self.a1 - 2 * self.a2 * T,
        #               axe - 2 * self.a2])
        # x = np.linalg.solve(A, b)

        # self.a3 = x[0]
        # self.a4 = x[1]
        # self.a5 = x[2]

        self.a3 = 1.0*(20*xe-20*xs-(8*vxe+12*vxs)*T-(3*axs-axe)*T*T)/(2*T*T*T)
        self.a4 = 1.0*(30*xs-30*xe+(14*vxe+16*vxs)*T+(3*axs-2*axe)*T*T)/(2*T*T*T*T)
        self.a5 = 1.0*(12*xe-12*xs-(6*vxe+6*vxs)*T-(axs-axe)*T*T)/(2*T*T*T*T*T)

    def calc_point(self, t, order):
        if order == 0:
            xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
                self.a3 * t ** 3 + self.a4 * t ** 4 + self.a5 * t ** 5
        elif order == 1:
            xt = self.a1 + 2 * self.a2 * t + \
                3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3 + 5 * self.a5 * t ** 4
        elif order == 2:
            xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2 + 20 * self.a5 * t ** 3
        elif order == 3:
            xt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t ** 2
        else:
            print("Error, order is not valid")
            xt = 0.0
        return xt

    def calc_arc_len(self,t0,t1,dt):
        if t0 <= t1:
            len_sign = 1
            t = np.arange(t0,t1,dt) # 开闭区间不一定准。
        else:
            len_sign = -1
            t = np.arange(t1,t0,dt)
        dxt = self.calc_point(t,1)
        return len_sign*dt*np.sum(np.sqrt(1+dxt**2))

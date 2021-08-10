#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" 
局部速度规划器 
"""

import numpy as np
import matplotlib.pyplot as plt
from Utils.tool import save_fig, to_point, DRAW_ST_FIG
from Model.curve import Curve

# TODO:最后一点的位置非常奇怪
# TODO:和之前的路径点s并不一一对应，需要重新发送s点
# TODO:加速度限制
# TODO:减少运算时间
# TODO:在不停车的情况下测试

class SpeedPlanner:
    def __init__(self, st_map, cur_vel, speed_lim):
        self.st_map = st_map
        self.cur_vel = cur_vel
        self.speed_lim = speed_lim
        self.dt = self.st_map.dt
        self.ds = self.st_map.ds
        self.path_s_ind = []

    def plan(self):
        if DRAW_ST_FIG:
            plt.figure()
            self.st_map.draw_st_fig()

        s_arr = []
        t_arr = []
        curve_speed_buff = []
        speed_found = self.find_speed()
        if speed_found:
            p_j = 0
            t = 0.0
            for cur_j in self.path_s_ind:
                s_arr.append(cur_j*self.ds)
                t_arr.append(t)
                t += self.dt
                # v = 1.0*(cur_j-p_j)*self.ds/self.dt
                # speed_buff.append(v)
            # s-t插值，在取每个s路径点对应的速度，即一阶导数
            curve_dt = self.dt*1.0/5
            vec = 0.0
            t_arr = np.array(tuple(t_arr))
            s_arr = np.array(tuple(s_arr))
            curve_speed = Curve(np.diff(t_arr),t_arr[0],curve_dt,s_arr,vec)
            tt = np.arange(t_arr[0],t_arr[-1],curve_dt)
            vv = curve_speed.calc_point_arr(tt,1)
            if DRAW_ST_FIG:
                ss = curve_speed.calc_point_arr(tt,0)
                plt.plot(tt,ss,c='red')
        if DRAW_ST_FIG:
            save_fig()
        return vv   

    def find_speed(self):
        pre_j = 0
        for i in range(1,self.st_map.n_t):
            for j in range(pre_j,self.st_map.n_s):
                flag = self.find_speed_point(i,j)

        # 找到最优终点
        tmp_i_cost = []
        for i in range(1,self.st_map.n_t):
            tmp_i_cost.append(self.st_map.map[i][-1].cost)
        min_i_cost = min(tmp_i_cost)

        tmp_j_cost = []
        for j in range(1,self.st_map.n_s):
            tmp_j_cost.append(self.st_map.map[-1][j].cost)
        min_j_cost = min(tmp_j_cost)
        if min_i_cost <= min_j_cost:
            end_i = tmp_i_cost.index(min_i_cost)
            end_j = self.st_map.n_s-1
        else:
            end_i = self.st_map.n_t-1
            end_j = tmp_j_cost.index(min_j_cost)

        if DRAW_ST_FIG:
            plt.scatter(end_i*self.dt, end_j*self.ds, c='red')
        self.path_s_ind.append(end_j)
        j = end_j
        for k in range(end_i):
            i = end_i - k
            j = self.st_map.map[i][j].pre_ind
            self.path_s_ind.insert(0,j)
            if DRAW_ST_FIG:
                plt.scatter((i-1)*self.dt, j*self.ds, c='red')
            
        return True


    def find_speed_point(self,cur_i,cur_j):
        flag = 1
        if cur_i == 1:
            p_j = 0
            cost, flag = self.cal_cost(cur_i,cur_j,p_j)
            self.st_map.map[cur_i][cur_j].cost = cost
            self.st_map.map[cur_i][cur_j].pre_ind = 0
        else:
            tmp_cost = []
            for p_j in range(cur_j+1):
                cost, tmp_flag = self.cal_cost(cur_i,cur_j,p_j)
                if tmp_flag == 1:
                    cost += self.st_map.map[cur_i-1][p_j].cost
                    tmp_cost.append(cost)
            if tmp_cost:
                min_cost = min(tmp_cost)
                self.st_map.map[cur_i][cur_j].cost = min_cost
                self.st_map.map[cur_i][cur_j].pre_ind = tmp_cost.index(min_cost)
            else:
                flag = 0
        return flag

    def cal_cost(self,cur_i,cur_j,p_j):
        flag = 1   # 可行情况
        # 利用前面的信息计算v,a,j
        # 前几个点可以默认静止  
        v = 1.0*(cur_j-p_j)*self.ds/self.dt
        if cur_i <= 1:
            a = (v-self.cur_vel) / self.dt 
            j = a / self.dt
        elif cur_i <= 2:
            pp_j = self.st_map.map[cur_i-1][p_j].pre_ind
            if pp_j == -1:
                print("Error! (%d,%d,%d)" % (cur_i,cur_j,p_j))
            a = 1.0*(cur_j-2*p_j+pp_j)*self.ds/((self.dt)**2)
            j = a / self.dt
        else:
            pp_j = self.st_map.map[cur_i-1][p_j].pre_ind
            ppp_j = self.st_map.map[cur_i-2][pp_j].pre_ind
            if pp_j == -1 or ppp_j == -1:
                print("Error!! (%d,%d,%d)" % (cur_i,cur_j,p_j))
            a = 1.0*(cur_j-2*p_j+pp_j)*self.ds/((self.dt)**2)
            j = 1.0*(cur_j-3*p_j+3*pp_j-ppp_j)*self.ds/((self.dt)**3)
        # 障碍物Cost
        ob_cost = 0.0
        k_safe = 1.0        # 3.0
        back_safe_dist = 12.0   # 20
        front_safe_dist = k_safe * v
        ob_state = self.st_map.ob_mat[cur_i]
        s = cur_j * self.ds
        for k in range(len(ob_state.s_down)):   # 对于当前存在的每个障碍物
            if s >= ob_state.s_down[k] and s <= ob_state.s_up[k]:   # 撞到了
                ob_cost += 1e8
            elif s < ob_state.s_down[k] and s >= ob_state.s_down[k] - front_safe_dist:  # 障碍物在前方
                ob_cost += 1e3 * (front_safe_dist + s - ob_state.s_down[k])**2
            elif s > ob_state.s_up[k] and s <= ob_state.s_up[k] + back_safe_dist:
                ob_cost += 1e3 * (back_safe_dist + s - ob_state.s_up[k])**2
        # 速度Cost
        if v > self.speed_lim:  # 超速
            speed_cost = 10*10*(v**2)*self.dt
        else:   # 未超速，奖励
            delta_v = (v-self.speed_lim)/self.speed_lim
            speed_cost = -10*10*delta_v*self.dt
        # 加速度和加加速度Cost
        acc_cost = (a**2)*self.dt
        jerk_cost = (j**2)*self.dt
        total_cost = ob_cost + speed_cost + acc_cost + jerk_cost
        return total_cost, flag
        

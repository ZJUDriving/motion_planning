#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
局部路径规划器
"""

# TODO:DP计算每条样条轨迹的cost时，v和a通过前后得到（目前直接给0）

import numpy as np
import matplotlib.pyplot as plt

from Utils.tool import get_arange, cal_dist_arr, DRAW_SL_FIG
from Model.curve import QuinticPoly, Curve


class PathPlanner():
    def __init__(self, sl_map):
        self.sl_map = sl_map
        self.init_ind = -2          # 储存起始下标
        self.cost_map = np.zeros((self.sl_map.n_s,self.sl_map.n_l))
        self.index_map = self.init_ind*np.ones((self.sl_map.n_s,self.sl_map.n_l))
        self.d_s = 0.5              # 插值间隔
        self.w_d = 0.5              # cost线性组合系数
        self.path_ind_list = []     # 路径点下标列表
        self.no_path_cost = 1e4     # 没有找到可行路径的cost界限

    def plan(self):
        # DP搜索
        if DRAW_SL_FIG: # 绘制Frenet系采样地图
            self.sl_map.draw_sl_fig()
        
        path_found = self.find_path()
        curve_path = []
        if path_found:
            path_s = []
            path_l = []
            path_s.append(self.sl_map.ego_point[0])
            path_l.append(self.sl_map.ego_point[1])
            for s in range(self.sl_map.n_s):
                tmp_l = self.path_ind_list[s]
                # tmp_l = self.sl_map.mid_l
                path_s.append(self.sl_map.s_map[s][tmp_l])
                path_l.append(self.sl_map.l_map[s][tmp_l])
                if DRAW_SL_FIG:    # 选取的路径点
                    plt.scatter(path_s[-1],path_l[-1],c='red')

            # 路径五次样条插值
            # send_num = int(self.sl_map.n_s/2)+1    # 除无人车位置外，发送的路径点个数（插值前）
            # path_s = path_s[:send_num+1]
            # path_l = path_l[:send_num+1]
            path_s = np.array(tuple(path_s))
            path_l = np.array(tuple(path_l))
            curve_path = Curve(path_s,self.d_s,path_l,0.0)
            # ss = np.arange(path_s[0],path_s[-1],self.d_s)
            # ll = curve_path.calc_point_arr(ss,0)
            # for j in range(len(ss)):
            #     path_buff.append(to_point(ss[j],ll[j]))
            # if DRAW_SL_FIG:
            #     plt.plot(ss,ll,c='red')
            #     save_fig()
            # plt.show()
            # time.sleep(100)
        return path_found, curve_path
    
    """ DP递推搜索最优路径组合 """
    def find_path(self):
        # 找到起点
        for l in range(self.sl_map.n_l):
            self.find_path_point(0,l)
        # DP搜索最优路径
        for s in range(1,self.sl_map.n_s):
            for l in range(self.sl_map.n_l):
                self.find_path_point(s,l)
            tmp_cost = self.cost_map[s,:]
            if np.min(tmp_cost) > 10*self.no_path_cost:
                print("Path not found!")
                return False
        # 找到最优终点
        tmp_cost = self.cost_map[-1,:]
        end_l = np.argmin(tmp_cost)
        # 回溯得到整条路径
        self.path_ind_list = []
        self.path_ind_list.append(end_l)
        for i in range(1,self.sl_map.n_s):
            end_s = self.sl_map.n_s - i
            end_l = int(self.index_map[end_s][end_l])
            self.path_ind_list.insert(0,end_l)
        return True

    """ 找到(s,l)的前一个最优点l值，记录在index_map中 """
    def find_path_point(self,s,l):
        if s == 0:   # 边界条件，第一列位置
            self.cost_map[s][l] = self.cal_cost(-1,-1,s,l)
            self.index_map[s][l] = -1
        else:
            tmp_cost = np.zeros((1,self.sl_map.n_l)) 
            for j in range(self.sl_map.n_l):
                tmp_cost[0][j] = self.cal_cost(s-1,j,s,l) + self.cost_map[s-1][j]
            self.cost_map[s][l] = np.amin(tmp_cost)     # 暂存cost，用于后续计算
            self.index_map[s][l] = np.argmin(tmp_cost)  # 记录后续列下标在当前点位置  
        return True

    """ 计算路径点之间的cost """
    def cal_cost(self,s_1,l_1,s_2,l_2):
        if s_1 != -1:
            p1 = np.array([self.sl_map.s_map[s_1][l_1],self.sl_map.l_map[s_1][l_1]])
        else:   # 初始位置
            p1 = self.sl_map.ego_point
        p2 = np.array([self.sl_map.s_map[s_2][l_2],self.sl_map.l_map[s_2][l_2]])
        # 五次多项式cost
        ss, ll, dll, ddll, dddll = self.get_path(p1[0],p1[1],p2[0],p2[1],1)
        mid_l_value = self.sl_map.l_map[s_1][self.sl_map.mid_l]
        guide_cost = self.d_s*np.sum((ll-mid_l_value)**2)
        smooth_cost = self.d_s*(np.sum(dll**2) + np.sum(ddll**2) + np.sum(dddll**2))
        # 障碍物cost
        arr = np.vstack((ss,ll))
        if self.sl_map.ob_list:
            tmp_dist = []
            for ob_point in self.sl_map.ob_list:
                dist, min_p = cal_dist_arr(arr,ob_point)
                tmp_dist.append(dist)
            min_dist = min(tmp_dist)
            if min_dist < self.sl_map.ob_dist:
                ob_cost = 3*self.no_path_cost
            else: 
                ob_cost = 0
        else:   # 不存在障碍物
            ob_cost = 0
            
        return self.w_d*guide_cost + (1-self.w_d)*smooth_cost + ob_cost

    """ 五次多项式插值得到分段路径，tips = 1时返回导数信息 """
    def get_path(self,p1s,p1l,p2s,p2l,tips):
        # s = np.array([p1s,p2s])
        # x = np.array([p1l,p2l])
        # cc = Curve(s,self.d_s,x,0.0)
        # ss = np.arange(p1s,p2s,self.d_s)
        # ll = cc.calc_point_arr(ss,0)
        # if tips == 1:
        #     dll = cc.calc_point_arr(ss,1)
        #     ddll = cc.calc_point_arr(ss,2)
        #     dddll = cc.calc_point_arr(ss,3)
        #     return ss, ll, dll, ddll, dddll
        # else:
        #     return ss, ll
        qu_poly = QuinticPoly(p1l,0,0,p2l,0,0,(p2s-p1s))
        ss = get_arange(p1s,p2s,self.d_s) # np.arange(p1s,p2s,self.d_s) # get_arange(p1s,p2s,self.d_s) # 
        ll = qu_poly.calc_point(ss,0)
        if tips == 1:
            dll = qu_poly.calc_point(ss,1)
            ddll = qu_poly.calc_point(ss,2)
            dddll = qu_poly.calc_point(ss,3)
            return ss, ll, dll, ddll, dddll
        else:
            return ss, ll


        


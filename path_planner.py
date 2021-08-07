#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
实现五次多项式DP，障碍物检查为静态且暴力
初步实现车道切换，但比较暴力
编写曲线类，将车道线利用五次多项式插值得到连续表达式
编写Frenet坐标系转换类，得到一般曲线的Frenet转换
DP从起点开始搜索，DP插值统一用Curve类（速度较慢）
"""

import carla
import numpy as np
import math
import matplotlib.pyplot as plt
import time

# from scipy.interpolate import interp1d
# from scipy.integrate import quad
from tool import *
from curve import QuinticPoly, Curve
from cartesian_frenet_conversion import CartesianFrenetConverter

STEP_COUNT = 0      # 路径更新下标
FIG_COUNT = 0       # 图片下标
SAVE_PATH = "../output/000"
DRAW_DEBUG = True   # 在Carla中绘制结果
DRAW_FRENET_FIG = False    # 绘制路径规划结果并保存图片
DRAW_WORLD_FIG = False
DRAW_ROBOT_FIG = False


def save_fig():
    global FIG_COUNT
    mkdir(SAVE_PATH)
    plt.savefig(SAVE_PATH + "/fig" + str(FIG_COUNT) + ".png")
    FIG_COUNT = FIG_COUNT + 1


""" 路径规划对外接口 """
class PlannerInterface():
    def __init__(self, world, _waypoint_buffer, _vehicle, ob_list):
        self.world = world
        self._waypoint_buffer = _waypoint_buffer
        self._vehicle = _vehicle
        self.ob_list = ob_list
        
    
    def run_step(self):
        d_s = 0.3
        # print(self._waypoint_buffer)
        command = Command.LANEFOLLOW
        waypoint_ahead = []
        ob_box, ob_rot = get_ob_box(self.world, self.ob_list)
        for i in range(len(self._waypoint_buffer)):
            waypoint_ahead.append(self._waypoint_buffer[i][0])
        global STEP_COUNT
        print("----------- %d ------------" % STEP_COUNT)
        if self._waypoint_buffer[1][1] == RoadOption.CHANGELANELEFT or \
            self._waypoint_buffer[1][1] == RoadOption.CHANGELANERIGHT:
            command = Command.CHANGELANELEFT
            print("Change line [%d]" % STEP_COUNT)
        if DRAW_WORLD_FIG:
            plt.figure()
            ego_pos = self.get_point(self._vehicle.get_transform().location)
            plt.scatter(ego_pos[0],ego_pos[1],c='yellow')
            for way_p in self._waypoint_buffer:
                posi = self.get_point(way_p[0].transform.location)
                print(way_p[1])
                print(posi)
                plt.scatter(posi[0],posi[1],c='blue')
        
        # 开始规划
        # if command == Command.CHANGELANELEFT:
        start_time = time.time()
        self.coor_trans(waypoint_ahead,ob_box,command)
        self.planner = PathPlanner(self.p_map)
        local_buff = self.planner.plan()
        end_time = time.time()
        print("[INFO] time_cost: %f" % (end_time - start_time))
        if DRAW_DEBUG:
            debug = self._vehicle.get_world().debug
            for pos in local_buff:
                # life_time=4.0
                debug.draw_line(pos, pos, 0.5, carla.Color(255,0,0,0),0)
                posi = self.get_point(pos)
                plt.scatter(posi[0],posi[1],c='red')
            for way_point in waypoint_ahead:
                posi = self.get_point(way_point.transform.location)
                plt.scatter(posi[0],posi[1],c='blue')
            # plt.show()
            # time.sleep(100)
        if DRAW_WORLD_FIG:
            for pos in local_buff:
                posi = self.get_point(pos)
                plt.scatter(posi[0],posi[1],c='red')
            save_fig()
        
        STEP_COUNT = STEP_COUNT + 1
        return local_buff
        
    """ 完成坐标转换 """
    def coor_trans(self, waypoint_ahead, ob_box, command):
        l_width = 3.5   # 车道宽度
        n_l = 7         # 车道纵向采点个数（奇数）
        n_s = len(waypoint_ahead)   # 车道横向采点个数，不含无人车自身位置
        d_l = l_width / (n_l-1)
        s_map, l_map = [], []
        # 根据车道线和当前位置计算坐标变换矩阵
        if command == Command.CHANGELANELEFT:   # 如果切换车道，则车道参考线的方向从第二列点开始计算
            cal_theta_ind = 1
        else:
            cal_theta_ind = 0

        # st_theta = math.atan2(next_pos[1]-st_pos[1], next_pos[0]-st_pos[0])
        # st_rot = np.array([[math.cos(st_theta),-math.sin(st_theta)], [math.sin(st_theta),math.cos(st_theta)]])
        ego_pos = self.get_point(self._vehicle.get_transform().location)
        ego_vec = self._vehicle.get_transform().rotation.get_forward_vector() 
        ego_theta = math.atan2(ego_vec.y, ego_vec.x) 
        ego_rot = np.array([[math.cos(ego_theta),-math.sin(ego_theta)], [math.sin(ego_theta),math.cos(ego_theta)]])
        # 添加机器人坐标系下的地图信息
        ref_line = []
        ry_list_w = []
        for way_point in waypoint_ahead:
            pos = self.get_point(way_point.transform.location)
            ref_line.append(pos)
        self.p_map = PlannerMap(ego_rot,ego_pos)
        self.p_map.add_ref_line(ref_line,l_width,n_l,n_s,cal_theta_ind)
        self.p_map.add_obstacle(self.get_point(ob_box.location))
        self.p_map.ob_dist = math.sqrt(ob_box.extent.x**2+ob_box.extent.y**2)
        if DRAW_FRENET_FIG: # 绘制Frenet系采样地图
            self.p_map.show()

    def get_point(self, loc):
        return to_point(loc.x, loc.y)


""" 机器人坐标系地图与Frenet坐标系地图 """
class PlannerMap():
    def __init__(self, R, t):
        self.R = R          # 机器人坐标系到世界坐标系的转移矩阵，TODO：真实情况下矩阵应该根据位置实时更新
        self.t = t
        self.final_point = []
        self.ob_point = []
        self.ob_dist = 0.0
        self.st_l = 0       # Frenet系下规划第一列下标     
        
    """ 添加车道参考线，同时创建Frenet栅格地图 """
    def add_ref_line(self, ref_line, l_width, n_l, n_s, cal_theta_ind=0):
        self.n_l = n_l
        self.n_s = n_s
        self.mid_l = int((self.n_l-1)/2)
        self.l_width = l_width
        rx_list = []
        ry_list = []
        for world_point in ref_line:
            point = self.world_to_map(world_point)
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
        self.converter = CartesianFrenetConverter(0.0,0.0,np.array(rx_list),np.array(ry_list),line_vec)
        so,lo = self.converter.cartesian_to_frenet(0.0,0.0)
        self.ego_point = np.array([so,lo])
        self.s_map = []
        self.l_map = []
        if DRAW_ROBOT_FIG:
            # self.converter.show()
            # plt.scatter(0.0,0.0,c='yellow')
            plt.scatter(rx_list[0],ry_list[0],c='green')
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
        self.s_map = np.concatenate(tuple(self.s_map),axis=0).reshape(self.n_s,self.n_l)
        self.l_map = np.concatenate(tuple(self.l_map),axis=0).reshape(self.n_s,self.n_l)
        # print(self.s_map)
        # print(self.l_map)
        if DRAW_ROBOT_FIG:
            # plt.show()
            save_fig()

    def add_obstacle(self, ob):
        ob = self.world_to_map(ob)
        s,l = self.converter.cartesian_to_frenet(ob[0],ob[1])
        self.ob_point = np.array([s,l])
    
    def world_to_map(self, point):
        point = point - self.t
        point = np.dot(self.R.transpose(),point.reshape(2,1)).reshape(2)
        return point
    
    def map_to_world(self, point):
        point = np.dot(self.R,point.reshape(2,1))
        point = point.reshape(2) + self.t
        return point

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

    def show(self):
        plt.figure()
        # [1:self.n_s+1][:]
        s_list = self.s_map.reshape(1,self.n_s*self.n_l)
        l_list = self.l_map.reshape(1,self.n_s*self.n_l)
        plt.scatter(s_list,l_list,c='blue')
        plt.scatter(self.ego_point[0],self.ego_point[1],c='yellow')
        # plt.scatter(self.final_point[0],self.final_point[1],c='yellow')
        
        # plt.scatter(self.start_point[0],self.start_point[1],c='red')
        if abs(self.ob_point[1]) - self.ob_dist < self.l_width:     # 是否绘制障碍物
            th = np.arange(0,2*math.pi,math.pi/20)
            circle_x = self.ob_point[0] + self.ob_dist*np.cos(th)
            circle_y = self.ob_point[1] + self.ob_dist*np.sin(th)
            plt.scatter(self.ob_point[0],self.ob_point[1],c='green')
            plt.scatter(circle_x,circle_y,c='green')
        # plt.show()
        # time.sleep(100)

    


""" 局部路径规划器 """
class PathPlanner():
    def __init__(self, p_map):
        self.p_map = p_map
        self.init_ind = -2  # 储存起始下标
        self.cost_map = np.zeros((self.p_map.n_s,self.p_map.n_l))
        self.index_map = self.init_ind*np.ones((self.p_map.n_s,self.p_map.n_l))
        self.d_s = 0.3      # 插值间隔
        self.w_d = 0.5      # cost线性组合系数
        self.path_ind_list = []


    def plan(self):
        # DP搜索
        tmp_l = 0   # self.p_map.st_l# int((self.p_map.n_l-1)/2)
        res = self.find_path()
        refer_path = np.zeros((2,self.p_map.n_s+1)) # 选取的路径点
        refer_path[0][0] = self.p_map.ego_point[0]  # 无人车的位置
        refer_path[1][0] = self.p_map.ego_point[1]
        for s in range(self.p_map.n_s):
            tmp_l = self.path_ind_list[s]
            refer_path[0][s+1] = self.p_map.s_map[s][tmp_l]
            refer_path[1][s+1] = self.p_map.l_map[s][tmp_l]
            if DRAW_FRENET_FIG:    # 选取的路径点
                plt.scatter(refer_path[0][s+1],refer_path[1][s+1],c='red')
        # 五次样条插值
        local_buff = []
        for i in range(2):
            ss, ll = self.get_path(refer_path[0][i],refer_path[1][i],refer_path[0][i+1],refer_path[1][i+1],0)
            for j in range(len(ss)):
                rx, ry = self.p_map.converter.frenet_to_cartesian(ss[j],ll[j])
                node = self.p_map.map_to_world(to_point(rx, ry))
                local_buff.append(carla.Location(x=node[0],y=node[1]))
            if DRAW_FRENET_FIG:
                plt.plot(ss,ll,c='red')
        if DRAW_FRENET_FIG:
            save_fig()
        # plt.show()
        # time.sleep(100)
        return local_buff
    
    def cal_cost(self,s_1,l_1,s_2,l_2):
        if s_1 != -1:
            p1 = np.array([self.p_map.s_map[s_1][l_1],self.p_map.l_map[s_1][l_1]])
        else:   # 初始位置
            p1 = self.p_map.ego_point
        p2 = np.array([self.p_map.s_map[s_2][l_2],self.p_map.l_map[s_2][l_2]])
        # 五次多项式cost
        ss, ll, dll, ddll, dddll = self.get_path(p1[0],p1[1],p2[0],p2[1],1)
        mid_l_value = self.p_map.l_map[s_1][self.p_map.mid_l]
        guide_cost = self.d_s*np.sum((ll-mid_l_value)**2)
        smooth_cost = self.d_s*(np.sum(dll**2) + np.sum(ddll**2) + np.sum(dddll**2))
        # 障碍物cost
        arr = np.vstack((ss,ll))
        min_dist, min_p = cal_dist_arr(arr,self.p_map.ob_point)
        if min_dist < self.p_map.ob_dist:
            ob_cost = 30000
        else:
            ob_cost = 0
        return self.w_d*guide_cost + (1-self.w_d)*smooth_cost + ob_cost

    """ DP递推搜索最优路径组合 """
    def find_path(self):
        # 找到起点
        for l in range(self.p_map.n_l):
            self.find_path_point(0,l)
        # DP搜索最优路径
        for s in range(1,self.p_map.n_s):
            for l in range(self.p_map.n_l):
                self.find_path_point(s,l)
        # 找到最优终点
        tmp_cost = self.cost_map[-1,:]
        end_l = np.argmin(tmp_cost)
        # 回溯得到整条路径
        self.path_ind_list = []
        self.path_ind_list.append(end_l)
        for i in range(1,self.p_map.n_s):
            end_s = self.p_map.n_s - i
            end_l = int(self.index_map[end_s][end_l])
            self.path_ind_list.insert(0,end_l)
        return True

    """ 找到(s,l)的前一个最优点l值，记录在index_map中 """
    def find_path_point(self,s,l):
        if s == 0:   # 边界条件，第一列位置
            self.cost_map[s][l] = self.cal_cost(-1,-1,s,l)
            self.index_map[s][l] = -1
        else:
            tmp_cost = np.zeros((1,self.p_map.n_l)) 
            for j in range(self.p_map.n_l):
                tmp_cost[0][j] = self.cal_cost(s-1,j,s,l) + self.cost_map[s-1][j]
            self.cost_map[s][l] = np.amin(tmp_cost)     # 暂存cost，用于后续计算
            self.index_map[s][l] = np.argmin(tmp_cost)  # 记录后续列下标在当前点位置    
        return True

    """ 五次多项式插值得到分段路径，tips = 1时返回导数信息 """
    def get_path(self,p1s,p1l,p2s,p2l,tips):
        # s = np.array([p2s-p1s])
        # x = np.array([p1l,p2l])
        # cc = Curve(s,p1s,self.d_s,x,0.0)
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
        ss = np.arange(p1s,p2s,self.d_s)
        ll = qu_poly.calc_point(ss,0)
        if tips == 1:
            dll = qu_poly.calc_point(ss,1)
            ddll = qu_poly.calc_point(ss,2)
            dddll = qu_poly.calc_point(ss,3)
            return ss, ll, dll, ddll, dddll
        else:
            return ss, ll


        


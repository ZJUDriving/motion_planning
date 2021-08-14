#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
路径规划器与仿真环境的接口
"""


import carla
import numpy as np
import math
import matplotlib.pyplot as plt
import time

from Utils.tool import get_ob_box, to_point, save_fig, RoadOption, Command
from Utils.tool import STEP_COUNT, DRAW_DEBUG, DRAW_WORLD_FIG, DRAW_SPEED_FIG, DRAW_SL_FIG
from Model.obstacle import *
from robot_map import RobotMap
from Planning.DP_Path.path_planner import PathPlanner
from Planning.DP_Speed.speed_planner import SpeedPlanner
from Planning.DP_Path.sl_map import SLMap
from Planning.DP_Speed.st_map import STMap



class PlannerInterface():
    def __init__(self, world, _waypoint_buffer, _vehicle, ob_list):
        self.world = world
        self._waypoint_buffer = _waypoint_buffer
        self._vehicle = _vehicle
        self.ob_list = ob_list
        self.dynamic_ob = []
        self.static_ob = []
    
    def run_step(self):
        command = Command.LANEFOLLOW
        waypoint_ahead = []
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
                plt.scatter(posi[0],posi[1],c='blue')
        
        """ 规划核心代码 """
        start_time = time.time()
        # 完成坐标转换，同时创建Robot图
        self.coor_trans(waypoint_ahead, command)    
        # 路径规划
        path_found, curve_path = self.path_plan()
        if not path_found:   # 未找到路径
            return [], []
        # 速度规划
        speed_found, path_buff, speed_buff = self.speed_plan(curve_path)
        if not speed_found:
            return [], []
        # 规划路径转换到世界坐标系下
        path_buff = self.sl_map.path_convert(path_buff)  
        path_buff = self.robot_map.path_convert(path_buff)  
        self.robot_map.save_robot_fig()
        end_time = time.time()
        print("[INFO] time_cost: %f" % (end_time - start_time))
        
        if DRAW_SPEED_FIG:
            plt.figure()
            plt.plot(speed_buff)
            save_fig()
        if DRAW_WORLD_FIG:
            for node in path_buff:
                plt.scatter(node[0],node[1],c='red')
            save_fig()
        path_buff_carla = []
        for node in path_buff:
            path_buff_carla.append(carla.Location(x=node[0],y=node[1]))
        if DRAW_DEBUG:
            debug = self._vehicle.get_world().debug
            for pos in path_buff_carla:
                # life_time=4.0
                debug.draw_line(pos, pos, 0.5, carla.Color(255,0,0,0),0)
                posi = self.get_point(pos)
                plt.scatter(posi[0],posi[1],c='red')
            for way_point in waypoint_ahead:
                posi = self.get_point(way_point.transform.location)
                plt.scatter(posi[0],posi[1],c='blue')
        STEP_COUNT = STEP_COUNT + 1
        return path_buff_carla, speed_buff
        
    """ 完成坐标转换，创建Robot坐标系图 """
    def coor_trans(self, waypoint_ahead, command):
        l_width = 3.5   # 车道宽度
        n_l = 5         # 车道纵向采点个数（奇数）
        n_s = len(waypoint_ahead)   # 车道横向采点个数，不含无人车自身位置
        # 根据车道线和当前位置计算坐标变换矩阵
        if command == Command.CHANGELANELEFT:   # 如果切换车道，则车道参考线的方向从第二列点开始计算
            cal_theta_ind = 1
        else:
            cal_theta_ind = 0
        # st_theta = math.atan2(next_pos[1]-st_pos[1], next_pos[0]-st_pos[0])
        # st_rot = np.array([[math.cos(st_theta),-math.sin(st_theta)], [math.sin(st_theta),math.cos(st_theta)]])
        ego_pos = self.get_point(self._vehicle.get_transform().location)
        ego_vel = self.get_point(self._vehicle.get_velocity())
        ego_vec = self._vehicle.get_transform().rotation.get_forward_vector() 
        ego_theta = math.atan2(ego_vec.y, ego_vec.x) 
        ego_rot = np.array([[math.cos(ego_theta),-math.sin(ego_theta)], [math.sin(ego_theta),math.cos(ego_theta)]])
        # 添加机器人坐标系下的地图信息
        ref_line = []   # 参考线
        for way_point in waypoint_ahead:
            pos = self.get_point(way_point.transform.location)
            ref_line.append(pos)
        self.robot_map = RobotMap(ego_rot,ego_pos)
        self.robot_map.add_robot(ego_pos, ego_vel, 0.0)
        self.robot_map.add_ref_line(ref_line,l_width,n_l,n_s,cal_theta_ind)
        debug = self.world.debug
        for ob in self.ob_list:
            ob_vel = ob.get_velocity()
            ob_vel = to_point(ob_vel.x, ob_vel.y)
            ob_box, ob_rot = get_ob_box(self.world, ob)
            ob_pos = self.get_point(ob_box.location)
            ob_dist = math.sqrt(ob_box.extent.x**2+ob_box.extent.y**2)
            is_near_ego_car = self.robot_map.add_obstacle(ob_pos,ob_vel,ob_dist)
            if DRAW_DEBUG and is_near_ego_car:
                debug.draw_box(ob_box, ob_rot, 0.2, carla.Color(0,255,0,0),life_time=1.0)

    def path_plan(self):
        # 路径规划
        self.sl_map = SLMap(self.robot_map)
        path_planner = PathPlanner(self.sl_map)     # 初始化路径规划器
        path_buff = path_planner.plan()             # 进行路径规划
        return path_buff

    def speed_plan(self, curve_path):
        # 创建ST图
        path_buff = self.sl_map.path_sampling(curve_path, draw=DRAW_SL_FIG)
        end_point = path_buff[-1]
        plan_time = 5
        self.st_map = STMap(self.sl_map.converter, end_point[0], plan_time)
        self.st_map.add_obstacle(path_buff, self.sl_map.st_ob_traj, self.sl_map.dy_ob_traj)
        # 速度规划
        cur_vel = self.sl_map.robot_vel
        # print(cur_vel)
        # cur_speed = math.sqrt(cur_vel.x**2 + cur_vel.y**2)
        speed_lim = 25.0 * 5.0 / 18.0
        # print(speed_lim)
        speed_planner = SpeedPlanner(self.st_map, cur_vel, speed_lim)
        ss, speed_buff = speed_planner.plan()
        # 得到路径点和速度序列
        # ll = self.sl_map.converter.ref_curve.calc_point_arr(ss, 0)
        path_buff = self.sl_map.path_sampling(curve_path, False, ss)
        speed_found = True
        return speed_found, path_buff, speed_buff



        
    def get_point(self, loc):
        return to_point(loc.x, loc.y)

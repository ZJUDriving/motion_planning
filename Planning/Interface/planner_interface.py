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

from Utils.tool import *
from Planning.DP_Path.path_planner import PathPlanner
from Planning.DP_Path.planner_map import PlannerMap

STEP_COUNT = 0      # 路径更新下标
DRAW_DEBUG = True   # 在Carla中绘制结果
DRAW_WORLD_FIG = False


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
        # ob_box, ob_rot = get_ob_box(self.world, self.ob_list)
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
        self.coor_trans(waypoint_ahead, command)
        self.planner = PathPlanner(self.p_map)
        path_buff = self.planner.plan()
        path_buff = self.p_map.frenet_to_world(path_buff)
        end_time = time.time()
        print("[INFO] time_cost: %f" % (end_time - start_time))
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
            # plt.show()
            # time.sleep(100)
        
        STEP_COUNT = STEP_COUNT + 1
        return path_buff_carla
        
    """ 完成坐标转换 """
    def coor_trans(self, waypoint_ahead, command):
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
        debug = self.world.debug
        for ob in self.ob_list:
            ob_vel = ob.get_velocity()
            ob_vel = to_point(ob_vel.x, ob_vel.y)
            ob_box, ob_rot = get_ob_box(self.world, ob)
            res = self.p_map.add_obstacle(self.get_point(ob_box.location),math.sqrt(ob_box.extent.x**2+ob_box.extent.y**2),ob_vel)
            if res:
                print("draw")
                debug.draw_box(ob_box, ob_rot, 0.2, carla.Color(0,255,0,0),life_time=1.0)
        
    def get_point(self, loc):
        return to_point(loc.x, loc.y)

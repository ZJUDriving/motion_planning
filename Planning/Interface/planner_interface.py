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
from Model.obstacle import *
from Planning.DP_Path.path_planner import PathPlanner
from Planning.DP_Path.sl_map import SLMap
from Planning.DP_Speed.st_map import STMap


STEP_COUNT = 0      # 路径更新下标
DRAW_DEBUG = True   # 在Carla中绘制结果
DRAW_WORLD_FIG = False
DRAW_ST_FIG = False


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
        
        # 开始规划
        # if command == Command.CHANGELANELEFT:
        start_time = time.time()
        self.coor_trans(waypoint_ahead, command)    # 完成坐标转换，创建SL图
        self.planner = PathPlanner(self.sl_map)     # 初始化路径规划器
        path_buff = self.planner.plan()             # 进行路径规划

        if not path_buff:   # 未找到路径
            return []

        end_point = path_buff[-1]
        print(end_point)
        plan_time = 5
        self.st_map = STMap(end_point[0], plan_time)
        self.st_map.add_obstacle(path_buff, [], self.sl_map.dynamic_ob)
        path_buff = self.sl_map.frenet_to_world(path_buff)  # 规划路径转换到世界坐标系下
        end_time = time.time()
        print("[INFO] time_cost: %f" % (end_time - start_time))

        self.sl_map.save_robot_fig()
        if DRAW_WORLD_FIG:
            for node in path_buff:
                plt.scatter(node[0],node[1],c='red')
            save_fig()
        if DRAW_ST_FIG:
            plt.figure()
            self.st_map.show()
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
        
    """ 完成坐标转换，创建SL图 """
    def coor_trans(self, waypoint_ahead, command):
        l_width = 3.5   # 车道宽度
        n_l = 7         # 车道纵向采点个数（奇数）
        n_s = len(waypoint_ahead)   # 车道横向采点个数，不含无人车自身位置
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
        for way_point in waypoint_ahead:
            pos = self.get_point(way_point.transform.location)
            ref_line.append(pos)
        self.sl_map = SLMap(ego_rot,ego_pos)
        self.sl_map.add_ref_line(ref_line,l_width,n_l,n_s,cal_theta_ind)
        debug = self.world.debug
        for ob in self.ob_list:
            ob_vel = ob.get_velocity()
            ob_vel = to_point(ob_vel.x, ob_vel.y)
            ob_box, ob_rot = get_ob_box(self.world, ob)
            ob_pos = self.get_point(ob_box.location)
            ob_dist = math.sqrt(ob_box.extent.x**2+ob_box.extent.y**2)
            is_near_ego_car = self.sl_map.add_obstacle(ob_pos,ob_dist,ob_vel)
            # if is_near_ego_car:
                # if check_static(ob_vel):
                #     self.static_ob.append(VehicleBox(ob_pos,ob_vel,ob_dist))
                # else:
                #     self.dynamic_ob.append(DynamicObstacle(ob_pos,ob_vel,ob_dist))
            if DRAW_DEBUG and is_near_ego_car:
                # print("draw")
                debug.draw_box(ob_box, ob_rot, 0.2, carla.Color(0,255,0,0),life_time=1.0)
        
        
    def get_point(self, loc):
        return to_point(loc.x, loc.y)

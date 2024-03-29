#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" This module contains a local planner to perform
low-level waypoint following based on PID controllers. """

from collections import deque
import math
import carla
from Tracking.controller import VehiclePIDController
from planner_interface import PlannerInterface
from Utils.misc import distance_vehicle, draw_waypoints, compute_distance
from Utils.tool import RoadOption, Behavior


class LocalPlanner(object):
    """
    LocalPlanner implements the basic behavior of following a trajectory
    of waypoints that is generated on-the-fly.
    The low-level motion of the vehicle is computed by using two PID controllers,
    one is used for the lateral control
    and the other for the longitudinal control (cruise speed).

    When multiple paths are available (intersections)
    this local planner makes a random choice.
    """

    # Minimum distance to target waypoint as a percentage
    # (e.g. within 80% of total distance)

    # FPS used for dt
    

    def __init__(self, agent, fps):
        """
        :param agent: agent that regulates the vehicle
        :param vehicle: actor to apply to local planner logic onto
        """
        self.world = agent.vehicle.get_world()
        self._vehicle = agent.vehicle
        self._map = agent.vehicle.get_world().get_map()

        self._target_speed = None
        self.sampling_radius = None
        self._min_distance = None
        self._current_waypoint = None
        self.target_road_option = None
        self._next_waypoints = None
        self.target_waypoint = None
        self._vehicle_controller = None
        self._global_plan = None
        self._pid_controller = None
        self.waypoints_queue = deque(maxlen=20000)  # queue with tuples of (waypoint, RoadOption)
        self._buffer_size = 7
        self._waypoint_buffer = deque(maxlen=self._buffer_size)
        # self.pre_check_ind = 0
        self.pre_ind = 3        # 已到达的waypoint
        # self.ref_line_size = 7
        # self.ref_line = deque(maxlen=self.ref_line_size)

        self.FPS = fps
        self._init_controller()  # initializing controller

        self.ob_list = agent.ob_list
        self.re_plan = True
        self.local_ind = 0
        self.path_buff = []
        self.local_goal = []
        self.stop_flag = False
        self.replan_ind = 10
        self.time_flag = -1

    def reset_vehicle(self):
        """Reset the ego-vehicle"""
        self._vehicle = None
        print("Resetting ego-vehicle!")

    def _init_controller(self):
        """
        Controller initialization.

        dt -- time difference between physics control in seconds.
        This is can be fixed from server side
        using the arguments -benchmark -fps=F, since dt = 1/F

        target_speed -- desired cruise speed in km/h

        min_distance -- minimum distance to remove waypoint from queue

        lateral_dict -- dictionary of arguments to setup the lateral PID controller
                            {'K_P':, 'K_D':, 'K_I':, 'dt'}

        longitudinal_dict -- dictionary of arguments to setup the longitudinal PID controller
                            {'K_P':, 'K_D':, 'K_I':, 'dt'}
        """
        # Default parameters
        self.args_lat_hw_dict = {
            'K_P': 0.75,
            'K_D': 0.02,
            'K_I': 0.4,
            'dt': 1.0 / self.FPS}
        self.args_lat_city_dict = {
            'K_P': 0.58,
            'K_D': 0.02,
            'K_I': 0.5,
            'dt': 1.0 / self.FPS}
        self.args_long_hw_dict = {
            'K_P': 0.37,
            'K_D': 0.024,
            'K_I': 0.032,
            'dt': 1.0 / self.FPS}
        self.args_long_city_dict = {
            'K_P': 0.15,
            'K_D': 0.05,
            'K_I': 0.07,
            'dt': 1.0 / self.FPS}

        self._current_waypoint = self._map.get_waypoint(self._vehicle.get_location())

        self._global_plan = False

        self._target_speed = self._vehicle.get_speed_limit()

        self._min_distance = 3

    def set_speed(self, speed):
        """
        Request new target speed.

            :param speed: new target speed in km/h
        """

        self._target_speed = speed

    def set_global_plan(self, current_plan, clean=False):
        """
        Sets new global plan.

            :param current_plan: list of waypoints in the actual plan
        """
        for elem in current_plan:
            self.waypoints_queue.append(elem)

        if clean:
            vehicle_transform = self._vehicle.get_transform()
            self._waypoint_buffer.clear()
            for _ in range(self._buffer_size):
                if self.waypoints_queue:
                    self._waypoint_buffer.append(
                        self.waypoints_queue.popleft())
                else:
                    break
            # for i in range(len(self._waypoint_buffer)):
            #     if distance_vehicle(self._waypoint_buffer[i], vehicle_transform) > self._min_distance:
            #         break
            # self.pre_check_ind = i
            # print("Check index: %d" % self.pre_check_ind)
            

        self._global_plan = True

    def get_incoming_waypoint_and_direction(self, steps=3):
        """
        Returns direction and waypoint at a distance ahead defined by the user.

            :param steps: number of steps to get the incoming waypoint.
        """
        if len(self.waypoints_queue) > steps:
            return self.waypoints_queue[steps]

        else:
            try:
                wpt, direction = self.waypoints_queue[-1]
                return wpt, direction
            except IndexError as i:
                print(i)
                return None, RoadOption.VOID
        return None, RoadOption.VOID

    def get_current_waypoint(self):
        # next_wpt = self._waypoint_buffer[0]
        return self._current_waypoint

    def run_step(self, target_speed=None, debug=False):
        """
        Execute one step of local planning which involves
        running the longitudinal and lateral PID controllers to
        follow the waypoints trajectory.

            :param target_speed: desired speed
            :param debug: boolean flag to activate waypoints debugging
            :return: control
        """
        if self.time_flag > 0:  # 为了使同步世界能运行下去
            self.time_flag -= 1
            print("Wait for replan")
            return Behavior.STOP, self.stop_now()
        elif self.time_flag == 0:
            self.re_plan = True
            self.time_flag = -1
        
        # if target_speed is not None:
        #     self._target_speed = target_speed
        # else:
        #     self._target_speed = self._vehicle.get_speed_limit()

        

        # # Target waypoint
        # self.target_waypoint, self.target_road_option = self._waypoint_buffer[0]
        # print(self.target_road_option)
        # print(self.target_road_option == RoadOption.LANEFOLLOW)

        # Purge the queue of obsolete waypoints
        vehicle_transform = self._vehicle.get_transform()
        # 更新waypoint buff
        behavior = self.update_waypoint(vehicle_transform)
        if behavior == Behavior.STOP:
            return behavior, self.stop_now()

        # 局部路径和速度规划      
        if self.re_plan:
            self.local_plan()

        # 更新局部路径点，得到控制量
        behavior, control = self.local_contorl(vehicle_transform)
        # if debug:
        #     draw_waypoints(self._vehicle.get_world(),
        #                    [self.target_waypoint], 1.0)
        return behavior, control

    def update_waypoint(self, vehicle_transform):

        if len(self.waypoints_queue) == 0:
            print("No way point")
            return Behavior.STOP

        # Buffering the waypoints
        if not self._waypoint_buffer:
            print("No waypoint buff!!!!")
            for i in range(self._buffer_size):
                if self.waypoints_queue:
                    self._waypoint_buffer.append(
                        self.waypoints_queue.popleft())
                else:
                    break

        # Current vehicle waypoint
        self._current_waypoint = self._map.get_waypoint(self._vehicle.get_location())

        max_index = -1
        # for i, (waypoint, _) in enumerate(self._waypoint_buffer):
        #     if distance_vehicle(
        #             waypoint, vehicle_transform) < self._min_distance:
        #         max_index = i
        for i in range(self.pre_ind, len(self._waypoint_buffer)):
            if distance_vehicle(self._waypoint_buffer[i][0], vehicle_transform) < self._min_distance:
                max_index = i - self.pre_ind
        if max_index >= 0:
            for i in range(max_index + 1):
                self.add_waypoint()
            self.re_plan = True
            # print("in")
        return Behavior.VOID

    def local_plan(self):
        print("replan")
        if self._waypoint_buffer:
            planner = PlannerInterface(self.world, self._waypoint_buffer, self._vehicle, self.ob_list)
        else:
            print("Waypoint buff is empty.")
        self.path_buff, self.speed_buff = planner.run_step()
        self.re_plan = False
        self.local_ind = 0
        # print(max(self.speed_buff))
        # print(self.cal_dist(self.path_buff[0],self.path_buff[-1]) < 1.0)

    def local_contorl(self, vehicle_transform):
        if len(self.speed_buff) == 0 or max(self.speed_buff) < 0.1 or \
                self.cal_dist(self.path_buff[0],self.path_buff[-1]) < 1.0:
            print("No speed")
            tmp_ind = -1
        else:
            # 确定当前目标点和期望速度
            if len(self.path_buff) == 0:   # 没有找到路，停车
                tmp_ind = -1
            else:
                if self.local_ind < len(self.path_buff) - self.replan_ind:   # 根据路径更新下一个局部目标点
                    tmp_ind = self.local_ind
                    for i in range(self.local_ind,len(self.path_buff)):
                        # print(self.path_buff[i])
                        # print(vehicle_transform.location)
                        if self.cal_dist(self.path_buff[i], vehicle_transform.location) < self._min_distance:
                            tmp_ind = i
                    self.local_ind = tmp_ind
                else:
                    tmp_ind = len(self.path_buff) - self.replan_ind
                    print("Path end")
                    # self.add_waypoint()
                    self.re_plan = True
        # 控制器
        if tmp_ind >= 0:
            # print("buff index %d" % tmp_ind)
            self.target_waypoint = self.path_buff[tmp_ind]
            self._target_speed = self.speed_conversion(self.speed_buff[tmp_ind], "m")

            if self._target_speed > 50:
                args_lat = self.args_lat_hw_dict
                args_long = self.args_long_hw_dict
            else:
                args_lat = self.args_lat_city_dict
                args_long = self.args_long_city_dict

            self._pid_controller = VehiclePIDController(self._vehicle,
                                                        args_lateral=args_lat,
                                                        args_longitudinal=args_long)

            # print(self.speed_buff[tmp_ind])
            # print(self._target_speed)
            # self._target_speed = 15.0 # 单位km/h，*5/18=m/s
            control = self._pid_controller.run_step(self._target_speed, self.target_waypoint)
            control.brake = 0.0  
            return Behavior.LANEFOLLOW, control
        else:
            print("Stop now")
            
            # control.hand_brake = False
            # control.manual_gear_shift = False
            # self.re_plan = True
            self.time_flag = 50
            return Behavior.STOP, self.stop_now()

    def stop_now(self):
        control = carla.VehicleControl()
        control.steer = 0.0
        control.throttle = 0.0
        control.brake = 1.0
        return control

    def add_waypoint(self, del_pre=True):
        if del_pre and self._waypoint_buffer:
            self._waypoint_buffer.popleft()
        if self.waypoints_queue:
            next_p_found = True
            next_p = self.waypoints_queue.popleft()
            last_p = self._waypoint_buffer[-1]
            # 防止弹出相同的点
            while not self.check_valid(next_p[0].transform.location,last_p[0].transform.location):
                if self.waypoints_queue:
                    next_p = self.waypoints_queue.popleft()
                else:
                    next_p_found = False
                    break
            if next_p_found:
                self._waypoint_buffer.append(next_p)

    def speed_conversion(self, speed, speed_type):
        if speed_type == "k":
            return speed * 5.0 / 18.0
        else:
            return speed * 18.0 / 5.0

    def check_valid(self,next_p,last_p):
        is_valid = True
        if self.cal_dist(next_p,last_p) < 0.01:     # 防止路径点过近
            is_valid = False
        if self.check_behind(next_p):               # 防止路径点在车后方
            is_valid = False
        if self.check_back(next_p,last_p):          # 防止路径点回退
            is_valid = False
        return is_valid
    
    def cal_dist(self,p1,p2):
        return math.sqrt((p1.x-p2.x)**2+(p1.y-p2.y)**2)

    def check_behind(self,next_p):
        vehicle_transform = self._vehicle.get_transform()
        vec = vehicle_transform.get_forward_vector()
        loc = vehicle_transform.location
        return (next_p.x-loc.x)*vec.x + (next_p.y-loc.y)*vec.y < 0 

    def check_back(self,next_p,last_p):
        vehicle_transform = self._vehicle.get_transform()
        vec = vehicle_transform.get_forward_vector()
        return (next_p.x-last_p.x)*vec.x + (next_p.y-last_p.y)*vec.y < 0    

    

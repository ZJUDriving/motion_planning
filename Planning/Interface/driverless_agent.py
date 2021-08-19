#!/usr/bin/env python
# -*- coding: utf-8 -*-

import carla
from local_planner import LocalPlanner
from global_route_planner import GlobalRoutePlanner
from global_route_planner_dao import GlobalRoutePlannerDAO
from Utils.types_behavior import Cautious, Aggressive, Normal
from Utils.misc import get_speed, positive
from Utils.tool import RoadOption

class DriverlessAgent():

    def __init__(self, vehicle, ob_list, fps, ignore_traffic_light=False, behavior='normal'):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param ignore_traffic_light: boolean to ignore any traffic light
            :param behavior: type of agent to apply
        """
        self._vehicle = vehicle
        self._proximity_tlight_threshold = 5.0  # meters
        self._proximity_vehicle_threshold = 10.0  # meters
        self._local_planner = None
        self._world = self._vehicle.get_world()
        self.ob_list = ob_list
        try:
            self._map = self._world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
        self._last_traffic_light = None

        self.vehicle = vehicle
        self.ignore_traffic_light = ignore_traffic_light
        self._local_planner = LocalPlanner(self,fps)
        self._grp = None
        self.look_ahead_steps = 0

        # Vehicle information
        self.speed = 0
        self.speed_limit = 0
        self.direction = None
        self.incoming_direction = None
        self.incoming_waypoint = None
        self.start_waypoint = None
        self.end_waypoint = None
        self.is_at_traffic_light = 0
        self.light_state = "Green"
        self.light_id_to_ignore = -1
        self.min_speed = 5
        self.behavior = None
        self._sampling_resolution = 4.5
        self.FPS = fps

        if behavior == 'cautious':
            self.behavior = Cautious()
        elif behavior == 'normal':
            self.behavior = Normal()
        elif behavior == 'aggressive':
            self.behavior = Aggressive()

    def get_local_planner(self):
        """Get method for protected member local planner"""
        return self._local_planner

    def update_information(self):
        """
        This method updates the information regarding the ego
        vehicle based on the surrounding world.
        """
        self.speed = get_speed(self.vehicle)
        self.speed_limit = self.vehicle.get_speed_limit()
        self._local_planner.set_speed(self.speed_limit)
        self.direction = self._local_planner.target_road_option
        if self.direction is None:
            self.direction = RoadOption.LANEFOLLOW

        self.look_ahead_steps = int((self.speed_limit) / 10)

        self.incoming_waypoint, self.incoming_direction = self._local_planner.get_incoming_waypoint_and_direction(
            steps=self.look_ahead_steps)
        if self.incoming_direction is None:
            self.incoming_direction = RoadOption.LANEFOLLOW

        self.is_at_traffic_light = self.vehicle.is_at_traffic_light()
        if self.ignore_traffic_light:
            self.light_state = "Green"
        else:
            # This method also includes stop signs and intersections.
            self.light_state = str(self.vehicle.get_traffic_light_state())

    def set_destination(self, start_location, end_location, clean=False):
        """
        This method creates a list of waypoints from agent's position to destination location
        based on the route returned by the global router.

            :param start_location: initial position
            :param end_location: final position
            :param clean: boolean to clean the waypoint queue
        """
        if clean:
            self._local_planner.waypoints_queue.clear()
        self.start_waypoint = self._map.get_waypoint(start_location)
        self.end_waypoint = self._map.get_waypoint(end_location)

        route_trace = self._trace_route(self.start_waypoint, self.end_waypoint)
        # print(route_trace)
        if True:
            debug = self.vehicle.get_world().debug
            # world_snapshot = world.get_snapshot()
            for way_point in route_trace:
                # print(way_point[1])
                pos = way_point[0].transform.location
                # print(pos)
                if way_point[1] == RoadOption.STRAIGHT or way_point[1] == RoadOption.LANEFOLLOW:
                    debug.draw_line(pos, pos, 0.8, carla.Color(0,0,255,0),0)
                elif way_point[1] == RoadOption.CHANGELANELEFT or way_point[1] == RoadOption.CHANGELANERIGHT:
                    debug.draw_line(pos, pos, 0.8, carla.Color(0,255,0,0),0)
                else:
                    debug.draw_line(pos, pos, 0.8, carla.Color(0,0,0,0),0)
                # if way_point[1].lane_change == carla.LaneChange.Right:
                #     debug.draw_line(pos, pos, 0.8, carla.Color(0,0,255,0),0)
                # elif way_point[0].lane_change == carla.LaneChange.Left:
                #     debug.draw_line(pos, pos, 0.8, carla.Color(0,255,0,0),0)
                # elif way_point[0].lane_change == carla.LaneChange.NONE:
                #     debug.draw_line(pos, pos, 0.8, carla.Color(0,0,0,0),0)
                # elif way_point[0].lane_change == carla.LaneChange.Both:
                #     debug.draw_line(pos, pos, 0.8, carla.Color(255,255,0,0),0)
                # road_line = pos + carla.Location(y=way_point[0].lane_width/2.0)
                # debug.draw_line(road_line, road_line+carla.Location(x=4.0,z=0.5), 0.3, carla.Color(255,0,0,0),0)
                # road_line = pos + carla.Location(y=-way_point[0].lane_width/2.0)
                # debug.draw_line(road_line, road_line+carla.Location(x=4.0,z=0.5), 0.3, carla.Color(255,0,0,0),0)
            # time.sleep(100) 

        self._local_planner.set_global_plan(route_trace, clean)

    def _trace_route(self, start_waypoint, end_waypoint):
        """
        This method sets up a global router and returns the
        optimal route from start_waypoint to end_waypoint.

            :param start_waypoint: initial position
            :param end_waypoint: final position
        """
        # Setting up global router
        if self._grp is None:
            wld = self.vehicle.get_world()
            dao = GlobalRoutePlannerDAO(
                wld.get_map(), sampling_resolution=self._sampling_resolution)
            grp = GlobalRoutePlanner(dao)
            grp.setup()
            self._grp = grp

        # Obtain route plan
        route = self._grp.trace_route(
            start_waypoint.transform.location,
            end_waypoint.transform.location)

        return route

    def _overtake(self, location, waypoint, vehicle_list):
        """
        This method is in charge of overtaking behaviors.

            :param location: current location of the agent
            :param waypoint: current waypoint of the agent
            :param vehicle_list: list of all the nearby vehicles
        """

        left_turn = waypoint.left_lane_marking.lane_change
        right_turn = waypoint.right_lane_marking.lane_change

        left_wpt = waypoint.get_left_lane()
        right_wpt = waypoint.get_right_lane()

        if (left_turn == carla.LaneChange.Left or left_turn ==
                carla.LaneChange.Both) and waypoint.lane_id * left_wpt.lane_id > 0 and left_wpt.lane_type == carla.LaneType.Driving:
            print("Overtaking to the left!")
            self.behavior.overtake_counter = 200
            self.set_destination(left_wpt.transform.location, self.end_waypoint.transform.location, clean=True)
        elif right_turn == carla.LaneChange.Right and waypoint.lane_id * right_wpt.lane_id > 0 and right_wpt.lane_type == carla.LaneType.Driving:
            print("Overtaking to the right!")
            self.behavior.overtake_counter = 200
            self.set_destination(right_wpt.transform.location, self.end_waypoint.transform.location, clean=True)

    def run_step(self, debug=False):
        """
        Execute one step of navigation.

            :param debug: boolean for debugging
            :return control: carla.VehicleControl
        """
        control = None
        # if self.behavior.tailgate_counter > 0:
        #     self.behavior.tailgate_counter -= 1
        if self.behavior.overtake_counter > 0:
            self.behavior.overtake_counter -= 1

        ego_vehicle_loc = self.vehicle.get_location()
        ego_vehicle_wp = self._map.get_waypoint(ego_vehicle_loc)

        # # 1: Red lights and stops behavior

        # if self.traffic_light_manager(ego_vehicle_wp) != 0:
        #     return self.emergency_stop()

        # # 2.1: Pedestrian avoidancd behaviors
        # walker_state, walker, w_distance = self.pedestrian_avoid_manager(
        #     ego_vehicle_loc, ego_vehicle_wp)
        # if walker_state:
        #     # Distance is computed from the center of the two cars,
        #     # we use bounding boxes to calculate the actual distance
        #     distance = w_distance - max(
        #         walker.bounding_box.extent.y, walker.bounding_box.extent.x) - max(
        #             self.vehicle.bounding_box.extent.y, self.vehicle.bounding_box.extent.x)
        #     # Emergency brake if the car is very close.
        #     if distance < self.behavior.braking_distance:
        #         return self.emergency_stop()

        # # 2.2: Car following behaviors
        # vehicle_state, vehicle, distance = self.collision_and_car_avoid_manager(
        #     ego_vehicle_loc, ego_vehicle_wp)
        # if vehicle_state:
        #     # Distance is computed from the center of the two cars,
        #     # we use bounding boxes to calculate the actual distance
        #     distance = distance - max(
        #         vehicle.bounding_box.extent.y, vehicle.bounding_box.extent.x) - max(
        #             self.vehicle.bounding_box.extent.y, self.vehicle.bounding_box.extent.x)

        #     # Emergency brake if the car is very close.
        #     if distance < self.behavior.braking_distance:
        #         return self.emergency_stop()
        #     else:
        #         control = self.car_following_manager(vehicle, distance)

        # # 4: Intersection behavior
        # # Checking if there's a junction nearby to slow down
        # elif self.incoming_waypoint.is_junction and (self.incoming_direction == RoadOption.LEFT or self.incoming_direction == RoadOption.RIGHT):
        #     control = self._local_planner.run_step(
        #         target_speed=min(self.behavior.max_speed, self.speed_limit - 5), debug=debug)

        # 5: Normal behavior
        # Calculate controller based on no turn, traffic light or vehicle in front

        control = self._local_planner.run_step(
            target_speed= min(self.behavior.max_speed, self.speed_limit - self.behavior.speed_lim_dist), debug=debug)
        
        return control

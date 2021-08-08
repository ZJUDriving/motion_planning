#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" 添加路径，导入库 """
import glob
import os
import sys
try:
    sys.path.append(glob.glob('./robot/driverless/CARLA/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# try:
#     sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
# except IndexError:
#     pass

import carla
import random
import time
from enum import Enum
import numpy as np
import cv2
# from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
from driverless_agent import DriverlessAgent

from tool import get_ob_box
from spawn_npc_fun import spawn_npc

IM_WIDTH = 640
IM_HEIGHT = 480
SHOW_CAM = False
DRAW_ROAD_LINE = False
DRAW_OB = False
TOWN = 'Town03' # 地图编号
TEST_ID = 6     # 测试条件
FPS = 20 # 50慢 # 20快


class ObState(Enum):
    VOID = -1
    ASSIGN = 1
    RANDOM = 2

""" 实时显示相机图像 """
def process_img(image):
    # image.save_to_disk('output/000/%06d.png' % image.frame)
    i = np.array(image.raw_data)
    i2 = i.reshape((IM_HEIGHT, IM_WIDTH, 4))
    i3 = i2[:, :, :3]
    cv2.imshow("", i3)
    cv2.waitKey(1)
    return i3/255.0


actor_list = []
try:
    """ 测试条件 """
    if TEST_ID == 1:    # 单直道避障超车
        # ego_pos = [-115.4, 4.0, 11]
        # ego_ori = [0, 180, 0]
        # ego_pos = [-315.6, 33.3, 2.0]
        # ego_ori = [0, 0, 0]
        # target_pos = [347.2, 35.6, 3.0]
        # ego_pos = [210, -4.5, 1.0]
        ego_pos = [210, -4.5+3.2,1.0]
        ego_ori = [0, 180, 0]
        target_pos = [30.2, -4.8, 1.0]
        # target_pos = [5.9, -115.3, 1.0]

        # ob_pos = [-200, 33.3, 5.0]
        # ob_ori = [0, 0, 0]
        # ob_pos = [180, -6.7, 1.0]
        ob_pos = [180, -6.7+3.5, 1.0]
        # ob_pos = [180, -6.7+3.5+3.0, 1.0]
        ob_ori = [0, 180, 0]
        ob_state = ObState.ASSIGN
    elif TEST_ID == 2:      # 双车道避障超车+切换车道
        ego_pos = [210, -4.5 ,1.0]
        ego_ori = [0, 180, 0]
        target_pos = [30.2, -4.8, 1.0]
        # ob_pos = [180, -6.7, 1.0]
        ob_pos = [180, -6.7-1.0, 1.0]
        # ob_pos = [180, -6.7+1.0, 1.0]
        ob_ori = [0, 180, 0]
        ob_state = ObState.ASSIGN
    elif TEST_ID == 3:      # 动态障碍物
        ego_pos = [210, -4.5 ,1.0]
        ego_ori = [0, 180, 0]
        target_pos = [30.2, -4.8, 1.0]
        ob_pos = [180, -6.7-0.3, 1.0]
        ob_ori = [0, 180, 0]
        ob_state = ObState.ASSIGN
    elif TEST_ID == 4:      # 直道+弯道
        ego_pos = [210, -4.5 ,1.0]
        ego_ori = [0, 180, 0]
        target_pos = [5.9, -115.3, 1.0]
        ob_pos = [180, -6.7, 1.0]
        ob_ori = [0, 180, 0]
        ob_state = ObState.ASSIGN
    elif TEST_ID == 5:      # 弯道
        ego_pos = [30.2+15, -4.8-3.5, 1.0]
        ego_ori = [0, 180, 0]
        target_pos = [5.9, -115.3+40, 1.0]
        ob_pos = [180, -6.7, 1.0]
        ob_ori = [0, 180, 0]
        ob_state = ObState.VOID
    elif TEST_ID == 6:      # 随机动态障碍物（车辆）+直道和弯道
        ego_pos = [210, -4.5 ,1.0]
        ego_ori = [0, 180, 0]
        target_pos = [5.9, -115.3, 1.0]
        ob_state = ObState.RANDOM
        # ob_pos = [180, -6.7, 1.0]
        # ob_ori = [0, 180, 0]

    """ 连接世界，创建汽车 """
    port = 2000
    client = carla.Client('localhost', port)  # 连接到服务器
    client.set_timeout(2.0)
    world = client.load_world(TOWN)
    # world = client.get_world()      # 获取世界
    
    blueprint_library = world.get_blueprint_library()   # 访问蓝图
    vehicle_bp = blueprint_library.find('vehicle.tesla.model3') # 提供特斯拉模型3的默认蓝图
    vehicle_bp.set_attribute('color', '255,0,0')
    ego_point = carla.Transform(carla.Location(x=ego_pos[0],y=ego_pos[1],z=ego_pos[2]),
        carla.Rotation(pitch=ego_ori[0],yaw=ego_ori[1],roll=ego_ori[2]))
    # ego_point = random.choice(world.get_map().get_spawn_points()) # 随机出生点
    ego_car = world.spawn_actor(vehicle_bp, ego_point)  # 创建汽车
    # ego_car.apply_control(carla.VehicleControl(throttle=0.4, steer=0.0))    # 油门，转向
    # ego_car.set_autopilot(True)  # 设置自动驾驶
    actor_list.append(ego_car)
    print("[INFO] Create car")

    """ 创建障碍物 """
    vehicles_list = []
    if ob_state == ObState.ASSIGN:   # 创建人为指定位置的障碍物
        obstacle_bp = blueprint_library.find('vehicle.tesla.model3')    # 创建障碍物
        obstacle_bp.set_attribute('color', '0,0,0')
        ob_point = carla.Transform(carla.Location(x=ob_pos[0],y=ob_pos[1],z=ob_pos[2]),
            carla.Rotation(pitch=ob_ori[0],yaw=ob_ori[1],roll=ob_ori[2]))
        obstacle = world.spawn_actor(obstacle_bp, ob_point)
        if TEST_ID == 3:
            throttle = 0.2
            brake = 0.0
        else:
            throttle = 0.0
            brake = 1.0
        obstacle.apply_control(carla.VehicleControl(throttle=throttle, steer=0.0, brake=brake))
        actor_list.append(obstacle)
        vehicles_list.append(obstacle)
        print("[INFO] Create obstacle")
    if SHOW_CAM and ob_state == ObState.ASSIGN:     # 加载相机
        camera_bp = blueprint_library.find('sensor.camera.rgb') 
        camera_bp.set_attribute('image_size_x', str(IM_WIDTH))
        camera_bp.set_attribute('image_size_y', str(IM_HEIGHT))
        camera_bp.set_attribute('fov', '110')
        # 后上方carla.Transform(carla.Location(x=-5.5, z=2.5), carla.Rotation(pitch=8.0))
        # cam_point = carla.Transform(carla.Location(x=2.5, z=0.7))
        cam_point = carla.Transform(carla.Location(x=-5.5, z=2.5), carla.Rotation(pitch=8.0))
        # camera = world.spawn_actor(camera_bp, cam_point, attach_to=ego_car)
        camera = world.spawn_actor(camera_bp, cam_point, obstacle, carla.AttachmentType.SpringArm)
        actor_list.append(camera)
        camera.listen(lambda data: process_img(data))
    if ob_state == ObState.RANDOM:  # 创建随机自主运动的障碍物
        vehicles_id = spawn_npc(FPS)
        for id in vehicles_id:
            vehicles_list.append(world.get_actor(id))
    time.sleep(1.0)

    """ 初始绘制 """
    debug = world.debug
    if DRAW_ROAD_LINE:    # 绘制道路边界
        roadline_arr = world.get_level_bbs(carla.CityObjectLabel.Roads)
        # ob_arr = world.get_level_bbs(carla.CityObjectLabel.Vehicles)
        # world_snapshot = world.get_snapshot()
        for roadline in roadline_arr:
            debug.draw_box(roadline,roadline.rotation, 0.5, carla.Color(0,0,255,0),0)
        # transform = ego_car.get_transform()
        # del_ind = -1
        # for i, ob in enumerate(ob_arr):
        #     if ob.contains(transform.location,carla.Transform()) :
        #         print(i)
        #         del_ind = i
        #         break
        # if del_ind != -1:
        #    del(ob_arr[del_ind])
        # print(len(ob_arr))
        # for ob in ob_arr:
        #     debug.draw_box(ob,ob.rotation, 0.2, carla.Color(0,255,0,0),0) 
    if DRAW_OB and ob_state == ObState.ASSIGN: # 障碍物边框
        # time.sleep(0.5)
        # for vehicle in vehicles_list:
        #     ob_box, ob_rot = get_ob_box(world,vehicle)
        #     # ob_vertices = ob_box.get_world_vertices(carla.Transform())        # 得到障碍物的顶点
        #     # for ob_vertice in ob_vertices:
        #     #     pos = ob_vertice
        #     #     debug.draw_line(pos, pos, 0.1, carla.Color(255,0,0,0), 0)
        #     debug.draw_box(ob_box, ob_rot, 0.2, carla.Color(0,255,0,0),0)
        ob_box, ob_rot = get_ob_box(world,obstacle)
        # ob_vertices = ob_box.get_world_vertices(carla.Transform())        # 得到障碍物的顶点
        # for ob_vertice in ob_vertices:
        #     pos = ob_vertice
        #     debug.draw_line(pos, pos, 0.1, carla.Color(255,0,0,0), 0)
        debug.draw_box(ob_box, ob_rot, 0.2, carla.Color(0,255,0,0),0)
    print("[INFO] Init debugger")
    
    """ 其他设置 """
    agent = DriverlessAgent(ego_car, vehicles_list, FPS)   # 自动驾驶服务创建
    destination = carla.Location(x=target_pos[0],y=target_pos[1],z=target_pos[2])   
    agent.set_destination(agent.vehicle.get_location(), destination, clean=True)    # 设置目标点
    print("[INFO] Set destination")

    settings = world.get_settings()
    settings.fixed_delta_seconds = 1.0/FPS#0.02#0.05
    settings.synchronous_mode = True    # 同步模式
    world.apply_settings(settings)
    print("[INFO] World synchronous")

    """ 开始测试 """
    print("[INFO] Mission start")
    spectator = world.get_spectator()
    tot_target_reached = 0
    num_min_waypoints = 21
    while True:
        # 监视者
        transform = ego_car.get_transform()
        spectator.set_transform(carla.Transform(transform.location + carla.Location(z=40),
            carla.Rotation(pitch=-90,yaw=180)))
        world.tick()
        agent.update_information()
        # # Set new destination when target has been reached
        # if len(agent.get_local_planner().waypoints_queue) < num_min_waypoints:
        #     # agent.reroute(spawn_points)
        #     # tot_target_reached += 1
        #     # world.hud.notification("The target has been reached " +
        #     #                         str(tot_target_reached) + " times.", seconds=4.0)
        #     break
        if len(agent.get_local_planner().waypoints_queue) == 0:
            print("[INFO] Target reached, mission accomplished...")
            break
        speed_limit = ego_car.get_speed_limit()
        agent.get_local_planner().set_speed(speed_limit)
        control = agent.run_step()
        ego_car.apply_control(control)
    print("[INFO] Mission over")
    # time.sleep(60)

finally:
    """ 摧毁actor """
    print('destroying actors')
    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)
    for actor in actor_list:
        actor.destroy()
    print('done.')
    if ob_state == ObState.RANDOM:
        for vehicle in vehicles_list:
            vehicle.destroy()

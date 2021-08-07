#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np
import carla
import os

from enum import Enum

def get_ob_box(world, ob):
    world_snapshot = world.get_snapshot()
    for actor_snapshot in world_snapshot:
        if actor_snapshot.id == ob.id:
            # ob_box = carla.BoundingBox(actor_snapshot.get_transform().location,actor_snapshot.bounding_box)
            ob_box = carla.BoundingBox(actor_snapshot.get_transform().location,ob.bounding_box.extent)
            ob_rot = actor_snapshot.get_transform().rotation
            break
    return ob_box, ob_rot

def check_collision(ego_pos, ego_box, ob):
    extent = ego_box.extent
    return True

def cal_dist(p1,p2):
    return math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)

# 计算点到曲线的距离，返回距离和投影点
def cal_dist_arr(arr,p):
    # 计算第i个目标点到所有源点的距离列表
    p = p.reshape(2,1)
    num = np.size(arr,1) 
    dist_mat = arr - np.tile(p,(1,num))
    dist = np.sqrt((dist_mat[[0],:]**2+dist_mat[[1],:]**2))
    # 取最近的距离作为两点配对后的误差，并记录源点下标
    min_dist = dist.min()
    min_ind = dist.argmin()
    pro_point = np.array([arr[0][min_ind],arr[1][min_ind]])
    return min_dist, pro_point

def to_point(x,y):
    return np.array([x, y])

def mkdir(path):
    # # 去除首位空格
    # path=path.strip()
    # # 去除尾部 \ 符号
    # path=path.rstrip("\\")
 
    # 判断路径是否存在
    # 存在     True
    # 不存在   False
    isExists=os.path.exists(path)
 
    # 判断结果
    if not isExists:
        # 如果不存在则创建目录
        # 创建目录操作函数
        os.makedirs(path) 
        print("Create folder successfully")
    else:
        print("Folder already exists")
 



class Command(Enum):
    """
    RoadOption represents the possible topological configurations
    when moving from a segment of lane to other.
    """
    VOID = -1
    LEFT = 1
    RIGHT = 2
    STRAIGHT = 3
    LANEFOLLOW = 4
    CHANGELANELEFT = 5
    CHANGELANERIGHT = 6

class RoadOption(Enum):
    """
    RoadOption represents the possible topological configurations
    when moving from a segment of lane to other.
    """
    VOID = -1
    LEFT = 1
    RIGHT = 2
    STRAIGHT = 3
    LANEFOLLOW = 4
    CHANGELANELEFT = 5
    CHANGELANERIGHT = 6



#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
车辆box
"""

from Utils.tool import cal_dist

class VehicleBox():
    def __init__(self, pos, velocity, radius):
        self.pos = pos
        self.velocity = velocity
        self.radius = radius

    def cal_dist(self, point):
        return cal_dist(self.pos, point)
        



        
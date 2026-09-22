#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np


class Scenario(object):
    def __init__(self):
        self.boundaries = []
        self.obstacles = []

        self.boundaries_pcl = []
        self.obstacles_pcl = []

        self.pcl_parser = PclParser()

    def add_boundary(self, name, parameters):
        self.boundaries.append([name, parameters])

    def add_obstacle(self, name, parameters):
        self.obstacles.append([name, parameters])

    def boundaries_pcl_parse(self):
        self.boundaries_pcl = []
        for b in self.boundaries:
            if len(b) < 2:
                continue
            name = b[0]
            parameters = b[1]
            self.boundaries_pcl.append(
                self.pcl_parser.parser(name, parameters)
            )

    def obstacles_pcl_parse(self):
        self.obstacles_pcl = []
        for o in self.obstacles:
            if len(o) < 2:
                continue
            name = o[0]
            parameters = o[1]
            self.obstacles_pcl.append(
                self.pcl_parser.parser(name, parameters)
            )

    def parse_all(self):
        self.boundaries_pcl_parse()
        self.obstacles_pcl_parse()

    def get_full_pcl(self):
        self.parse_all()

        pcl_array = []

        for b in self.boundaries_pcl:
            pcl_array += b

        for o in self.obstacles_pcl:
            pcl_array += o

        return pcl_array


class PclParser(object):
    def __init__(self, resolution=0.1):
        self.resolution = resolution

    def parser(self, name, parameters):
        if name == "wall":
            if len(parameters) != 3:
                return []
            return self.wall(parameters[0], parameters[1], parameters[2])

        if name == "cylinder":
            if len(parameters) != 3:
                return []
            return self.cylinder(parameters[0], parameters[1], parameters[2])

        if name == "square_cylinder":
            if len(parameters) != 3:
                return []
            return self.square_cylinder(
                parameters[0],
                parameters[1],
                parameters[2]
            )

        if name == "rectangle_cylinder":
            if len(parameters) != 4:
                return []
            return self.rectangle_cylinder(
                parameters[0],
                parameters[1],
                parameters[2],
                parameters[3]
            )

        return []

    def cylinder(self, radius, height, position):
        points = []

        radius = float(radius)
        height = float(height)
        position = np.array(position, dtype=float)

        num_points = int((2.0 * math.pi * radius) / self.resolution) + 1
        z_num = int(height / self.resolution) + 1

        for i in range(num_points):
            angle = (2.0 * math.pi * float(i)) / float(max(num_points - 1, 1))
            x = position[0] + radius * math.cos(angle)
            y = position[1] + radius * math.sin(angle)

            for j in range(z_num):
                z = position[2] + j * self.resolution
                points.append([x, y, z])

        return points

    def square_cylinder(self, side_length, height, position):
        points = []

        side_length = float(side_length)
        height = float(height)
        position = np.array(position, dtype=float)

        num_points = int((4.0 * side_length) / self.resolution) + 1
        z_num = int(height / self.resolution) + 1

        for i in range(num_points):
            s = i * self.resolution

            if s <= side_length:
                x = position[0] + s
                y = position[1]
            elif s <= 2.0 * side_length:
                x = position[0] + side_length
                y = position[1] + (s - side_length)
            elif s <= 3.0 * side_length:
                x = position[0] + (3.0 * side_length - s)
                y = position[1] + side_length
            else:
                x = position[0]
                y = position[1] + (4.0 * side_length - s)

            for j in range(z_num):
                z = position[2] + j * self.resolution
                points.append([x, y, z])

        return points

    def rectangle_cylinder(self, side_length_x, side_length_y, height, position):
        points = []

        side_length_x = float(side_length_x)
        side_length_y = float(side_length_y)
        height = float(height)
        position = np.array(position, dtype=float)

        perimeter = 2.0 * side_length_x + 2.0 * side_length_y
        num_points = int(perimeter / self.resolution) + 1
        z_num = int(height / self.resolution) + 1

        for i in range(num_points):
            s = i * self.resolution

            if s <= side_length_x:
                x = position[0] + s
                y = position[1]
            elif s <= side_length_x + side_length_y:
                x = position[0] + side_length_x
                y = position[1] + (s - side_length_x)
            elif s <= 2.0 * side_length_x + side_length_y:
                x = position[0] + (2.0 * side_length_x + side_length_y - s)
                y = position[1] + side_length_y
            else:
                x = position[0]
                y = position[1] + (perimeter - s)

            for j in range(z_num):
                z = position[2] + j * self.resolution
                points.append([x, y, z])

        return points

    def wall(self, p1, p2, height):
        points = []

        p1 = np.array(p1, dtype=float)
        p2 = np.array(p2, dtype=float)
        height = float(height)

        length = np.linalg.norm(p2 - p1)

        if length < 1e-6:
            return points

        num_points = int(length / self.resolution) + 1
        step_vector = (p2 - p1) / float(num_points)

        z_num = int(height / self.resolution) + 1

        for i in range(num_points + 1):
            base = p1 + i * step_vector

            for j in range(z_num):
                x = base[0]
                y = base[1]
                z = p1[2] + j * self.resolution
                points.append([x, y, z])

        return points
# '''
# MIT License

# Copyright (c) [year] Wang Yue (ywang_86@stu.xidian.edu.cn), Xidian University, Shaanxi, Xian, China 

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# '''
                
# import numpy as np
# import random

# class Scenario:
#     def __init__(self):
#         self.boundaries = []
#         self.boundaries_pcl = None
#         self.boundaries_parsed = False


#         self.obstacles = []
#         self.obstacles_pcl = None
#         self.obstacles_pcl_parsed = False

#         self.pcl_parser = PclParser()

#         self.DS = None
#         self.PFRs = None
#         self.ORFs = None
#         self.GRF = None

#     def add_boundary(self, name, parameters):
#         self.boundaries.append([name, parameters])
#         self.boundaries_parsed = False

#     def add_obstacle(self, name, parameters):
#         self.obstacles.append([name, parameters])
#         self.obstacles_pcl_parsed = False

#     def delete_all_obstacles(self):
#         self.obstacles = []
#         self.obstacles_pcl_parsed = False

#     def boundaries_pcl_parse(self):
#         if self.boundaries_parsed == False:
#             self.boundaries_pcl = []
#             for b in self.boundaries:
#                 self.boundaries_pcl.append(self.pcl_parser.parser(b[0], b[1]))               
#             self.boundaries_parsed = True

#     def obstacles_pcl_parse(self):
#         if self.obstacles_pcl_parsed == False:
#             self.obstacles_pcl = []
#             for o in self.obstacles:
#                 self.obstacles_pcl.append(self.pcl_parser.parser(o[0], o[1]))
#             self.obstacles_pcl_parsed = True

#     def shape_mutate(self, index):
#         obs_type = ['cylinder', 'square_cylinder']
#         selected = random.sample(obs_type,1)[0]
#         self.obstacles[index][0] = selected

#     def position_xy_mutate(self, index):
#         position_now = self.obstacles[index][1][2]
#         x_bias = random.choice([-1, 1]) * random.uniform(0.5, 1)
#         y_bias = random.choice([-1, 1]) * random.uniform(0.5, 1)
#         position_now[0] = position_now[0] + x_bias
#         position_now[1] = position_now[1] + y_bias
#         self.obstacles[index][1][2] = position_now

#     def position_xy_mutate_slightly(self, index):
#         position_now = self.obstacles[index][1][2]
#         x_bias = random.choice([-1, 1]) * random.uniform(0.1, 0.3)
#         y_bias = random.choice([-1, 1]) * random.uniform(0.1, 0.3)
#         position_now[0] = position_now[0] + x_bias
#         position_now[1] = position_now[1] + y_bias
#         self.obstacles[index][1][2] = position_now

#     def check_xy_conflict(self, conflict_list):
#         conflict_flag = False
#         for o in self.obstacles:
#             position_now = o[1][2]
#             for p in conflict_list:
#                 dist = (position_now[0] - p[0])**2 + (position_now[1] - p[1])**2
#                 if dist < 1**2:
#                     x_bias = random.uniform(-1, 1)
#                     y_bias = random.uniform(-1, 1)
#                     position_now[0] = position_now[0] + x_bias
#                     position_now[1] = position_now[1] + y_bias
#                     o[1][2] = position_now
#                     conflict_flag = True
#         return conflict_flag

#     def parse_all(self):
#         self.boundaries_pcl_parse()
#         self.obstacles_pcl_parse()

#     def get_full_pcl(self):
#         self.parse_all()
#         pcl_array = []
#         for b in self.boundaries_pcl:
#             pcl_array = pcl_array + b
#         for o in self.obstacles_pcl:
#             pcl_array = pcl_array + o

#         return pcl_array
        
#     def get_occ_in_2d(self):
#         self.boundaries_pcl_parse()
#         occ_list = []
#         for bs in self.boundaries_pcl:
#             for b in bs:
#                 occ_list.append([b[0],b[1]])

#         return occ_list



# class PclParser:
#     def __init__(self, resolution = 0.1):
#         self.resolution = resolution

#     def parser(self, name, parameters):
#         if name == 'cylinder':
#             if len(parameters) != 3:
#                 return []
#             else:
#                 return self.cylinder(parameters[0], parameters[1], parameters[2])

#         if name == 'square_cylinder':
#             if len(parameters) != 3:
#                 return []
#             else:
#                 return self.square_cylinder(parameters[0], parameters[1], parameters[2])

#         if name == 'wall':
#             if len(parameters) != 3:
#                 return []
#             else:
#                 return self.wall(parameters[0], parameters[1], parameters[2])


#     def cylinder(self, radius, height, position):
#         # 计算圆柱体的点数
#         num_points = int((2 * np.pi * radius) / self.resolution) + 1

#         # 生成圆柱体的点云数据
#         points = []
#         for i in range(num_points):
#             angle = (2 * np.pi * i) / (num_points - 1)
#             for j in range(int(height / self.resolution) + 1):
#                 z = position[2] + j * self.resolution
#                 x = position[0] + radius * np.cos(angle)
#                 y = position[1] + radius * np.sin(angle)
#                 points.append([x, y, z])
        
#         return points

#     def square_cylinder(self, side_length, height, position):
#         # 计算正方形柱体的点数
#         num_points = int((4 * side_length) / self.resolution) + 1

#         # 生成正方形柱体的点云数据
#         points = []
#         for i in range(num_points):
#             if i < side_length / self.resolution:
#                 x = position[0] + i * self.resolution
#                 y = position[1]
#             elif i < (2 * side_length) / self.resolution:
#                 x = position[0] + side_length
#                 y = position[1] + (i - side_length / self.resolution) * self.resolution
#             elif i < (3 * side_length) / self.resolution:
#                 x = position[0] + (3 * side_length - i * self.resolution)
#                 y = position[1] + side_length
#             else:
#                 x = position[0]
#                 y = position[1] + (4 * side_length - i * self.resolution)

#             for j in range(int(height / self.resolution) + 1):
#                 z = position[2] + j * self.resolution
#                 points.append([x, y, z])

#         return points

#     def rectangle_cylinder(self, side_length_x, side_length_y, height, position):
#         # 计算正方形柱体的点数
#         num_points = int((2 * side_length_x + 2 * side_length_y) / self.resolution) + 1

#         # 生成正方形柱体的点云数据
#         points = []
#         for i in range(num_points):
#             if i < side_length_x / self.resolution:
#                 x = position[0] + i * self.resolution
#                 y = position[1]
#             elif i < (side_length_x + side_length_y) / self.resolution:
#                 x = position[0] + side_length_x
#                 y = position[1] + (i - side_length_x / self.resolution) * self.resolution
#             elif i < (2 * side_length_x + side_length_y) / self.resolution:
#                 x = position[0] + (2 * side_length_x + side_length_y - i * self.resolution)
#                 y = position[1] + side_length_y
#             else:
#                 x = position[0]
#                 y = position[1] + (2 * side_length_x + 2 * side_length_y - i * self.resolution)

#             for j in range(int(height / self.resolution) + 1):
#                 z = position[2] + j * self.resolution
#                 points.append([x, y, z])

#         return points

#     def wall(self, p1, p2, height):
#         # 计算正方形柱体的点数
#         position1 = np.array(p1)
#         position2 = np.array(p2)
#         side_length = np.linalg.norm(position2 - position1)
#         num_points = int(side_length / self.resolution) + 1
#         step_vector = (position2 - position1) / num_points

#         # 生成正方形柱体的点云数据
#         points = []
#         for i in range(num_points):
#             x = position1[0] + i * step_vector[0]
#             y = position1[1] + i * step_vector[1]
#             z = position1[2] + i * step_vector[2]

#             for j in range(int(height / self.resolution) + 1):
#                 z = position1[2] + j * self.resolution
#                 points.append([x, y, z])

#         return points

#     def floor(self, p1, p2, height):
#         # 计算正方形柱体的点数
#         position1 = np.array(p1)
#         position2 = np.array(p2)
#         x1, y1 = position1
#         x2, y2 = position2
#         x_min = min(x1, x2)
#         x_max = max(x1, x2)
#         y_min = min(y1, y2)
#         y_max = max(y1, y2)

#         # 计算矩形中所有点的坐标
#         points = []
#         x = x_min
#         while x <= x_max:
#             y = y_min
#             while y <= y_max:
#                 points.append((x, y, height))
#                 y += self.resolution
#             x += self.resolution

#         return points

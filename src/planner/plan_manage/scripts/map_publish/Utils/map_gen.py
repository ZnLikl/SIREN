
'''
MIT License

Copyright (c) [year] Wang Yue (ywang_86@stu.xidian.edu.cn), Xidian University, Shaanxi, Xian, China 

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''
                
import pcl
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import rospy
import threading
import json
import sys
sys.path.append("../")
from Utils import pcl_parser
import time


class MapGenerator:
    def __init__(self, resolution, size, height, z_global, topic="map_generator/global_cloud", ns = None):
        self.node_name = "cylinder_point_cloud_generator"
        if not rospy.core.is_initialized():
            if ns != None:
                rospy.init_node(ns + '_' + self.node_name)
            else:
                rospy.init_node(self.node_name)
        if ns != None:
            topic = ns + "/" + topic  # 添加命名空间前缀


        self.pub = rospy.Publisher(topic, PointCloud2, queue_size=10)
        self.resolution = resolution
        self.pcl_parser = pcl_parser.PclParser(resolution = resolution)
        self.size = size
        self.height = height
        self.z_global = z_global
        self.global_map = None
        self.publish_thread = None
        self.publish_flag = False

        self.lock = threading.Lock()

    def generate_cylinder(self, radius, height, position):
        points = self.pcl_parser.cylinder(radius, height, position)
        self.add_to_ros_msg(points)


    def generate_square_cylinder(self, side_length, height, position):
        points = self.pcl_parser.square_cylinder(side_length, height, position)
        self.add_to_ros_msg(points)


    def generate_rectangle_cylinder(self, side_length_x, side_length_y, height, position):
        # 计算正方形柱体的点数
        points = self.pcl_parser.rectangle_cylinder(side_length_x, side_length_y, height, position)
        self.add_to_ros_msg(points)
        

    def generate_wall(self, p1, p2, height):
        points = self.pcl_parser.wall(p1, p2, height)
        self.add_to_ros_msg(points)


    def generate_floor(self, p1, p2, height):
        points = self.pcl_parser.floor(p1, p2, height)
        self.add_to_ros_msg(points)

    def add_to_ros_msg(self, points):
        # 将点云数据转换为sensor_msgs/PointCloud2消息
        cloud = pcl.PointCloud()
        cloud.from_list(points)

        # 创建PointCloud2消息
        msg = PointCloud2()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.height = 1
        msg.width = len(points)
        msg.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * len(points)
        msg.is_dense = True
        msg.data = np.asarray(cloud, dtype=np.float32).tostring()

        with self.lock:
            if self.global_map is None:
                self.global_map = msg
            else:
                self.global_map = self.merge_point_clouds(self.global_map, msg)

    def clean_map(self):
        self.global_map = None

    def merge_point_clouds(self, cloud1, cloud2):
        merged_cloud = PointCloud2()
        merged_cloud.header.stamp = rospy.Time.now()
        merged_cloud.header.frame_id = "map"
        merged_cloud.height = 1
        merged_cloud.width = cloud1.width + cloud2.width
        merged_cloud.fields = cloud1.fields
        merged_cloud.is_bigendian = cloud1.is_bigendian
        merged_cloud.point_step = cloud1.point_step
        merged_cloud.row_step = merged_cloud.point_step * merged_cloud.width
        merged_cloud.is_dense = True
        merged_cloud.data = cloud1.data + cloud2.data
        return merged_cloud

    def generate_map(self, obstacle_type, xy_positions, obs_size = None):
        if obs_size == None:
            obs_size = self.size
        for position in xy_positions:
            position = [position[0], position[1], self.z_global]
            if obstacle_type == "cylinder":
                self.generate_cylinder(obs_size, self.height, position)
            elif obstacle_type == "square_cylinder":
                self.generate_square_cylinder(obs_size, self.height, position)
            else:
                raise ValueError("Invalid obstacle_type: {}. Use 'cylinder' or 'square_cylinder'.".format(obstacle_type))

    def save_map(self, name):
        if not isinstance(self.global_map, PointCloud2):
            raise TypeError('Map data must be a PointCloud2 object')

        if not isinstance(name, type("string")):
            raise TypeError('Need a file name')

        pc = pc2.read_points(self.global_map, skip_nans=True)
        pc_array = np.array(list(pc))

        np.save(name + ".npy", pc_array)

    def read_map(self, name):
        if not isinstance(name, type("string")):
            raise TypeError('Need a file name')

        points = np.load(name + ".npy")

        cloud = pcl.PointCloud()
        cloud.from_list(points)

        msg = PointCloud2()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.height = 1
        msg.width = len(points)
        msg.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * len(points)
        msg.is_dense = True
        msg.data = np.asarray(cloud, dtype=np.float32).tostring()

        self.global_map = msg

        
    def map_pub(self):
        self.publish_flag = True
        self.publish_thread = threading.Thread(target=self.publish_loop)
        self.publish_thread.start()

    def publish_loop(self):
        #rate = rospy.Rate(1)  # 发布频率为1Hz
        while self.publish_flag and not rospy.is_shutdown():
            with self.lock:
                if self.global_map is not None:
                    self.global_map.header.stamp = rospy.Time.now()
                    self.pub.publish(self.global_map)
            #rate.sleep()
            time.sleep(1)

    def map_pub_stop(self):
        self.publish_flag = False
        if self.publish_thread != None:
            self.publish_thread.join()

    def __del__(self):
        self.map_pub_stop()


if __name__ == "__main__":
    resolution = 0.1
    height = 3.2
    size = 0.2
    z = -0.2

    map_generator = MapGenerator(resolution, size, height, z)

    obstacle_type = "cylinder"
    xy_positions = [[0.0, 0.0], [2.0, 0.0]]
    map_generator.generate_map(obstacle_type, xy_positions)

    obstacle_type = "square_cylinder"
    size = 0.3
    xy_positions = [[4.0, 0.0]]
    map_generator.size = size
    map_generator.generate_map(obstacle_type, xy_positions)
    map_generator.map_pub()
    rospy.sleep(5)
    map_generator.map_pub_stop()


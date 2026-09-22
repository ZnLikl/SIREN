#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import os
import sys
import rospy

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
MAP_PUBLISH_DIR = os.path.abspath(os.path.join(CURRENT_DIR, ".."))
UTILS_DIR = os.path.join(MAP_PUBLISH_DIR, "Utils")

if MAP_PUBLISH_DIR not in sys.path:
    sys.path.insert(0, MAP_PUBLISH_DIR)

if UTILS_DIR not in sys.path:
    sys.path.insert(0, UTILS_DIR)

from Utils import scenario_loader


class ScenarioExecutor(object):
    def __init__(self):
        rospy.init_node("scenario_executor")

        default_json = os.path.join(MAP_PUBLISH_DIR, "1.json")

        self.scenario_json = rospy.get_param("~scenario_json", default_json)
        self.map_topic = rospy.get_param("~map_topic", "/map_generator/global_cloud")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.publish_rate = rospy.get_param("~publish_rate", 1.0)

        self.points = []

        self.pub = rospy.Publisher(
            self.map_topic,
            PointCloud2,
            queue_size=1,
            latch=True
        )

    def normalize_points(self, raw_points):
        points = []

        for p in raw_points:
            if p is None:
                continue
            if len(p) < 3:
                continue

            points.append((
                float(p[0]),
                float(p[1]),
                float(p[2])
            ))

        return points

    def load_map(self):
        loader = scenario_loader.Loader()
        scenario = loader.load(self.scenario_json)

        if scenario is None:
            rospy.logerr("Failed to load scenario json: %s", self.scenario_json)
            return False

        raw_points = scenario.get_full_pcl()
        self.points = self.normalize_points(raw_points)

        if len(self.points) == 0:
            rospy.logerr("Scenario map has no points: %s", self.scenario_json)
            return False

        rospy.logwarn(
            "Custom map loaded: json=%s, points=%d, topic=%s, frame=%s",
            self.scenario_json,
            len(self.points),
            self.map_topic,
            self.frame_id
        )

        return True

    def make_cloud_msg(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame_id
        return pc2.create_cloud_xyz32(header, self.points)

    def spin(self):
        if not self.load_map():
            return

        rate = rospy.Rate(float(self.publish_rate))

        while not rospy.is_shutdown():
            self.pub.publish(self.make_cloud_msg())
            rate.sleep()


if __name__ == "__main__":
    node = ScenarioExecutor()
    node.spin()
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
                
# # import sys
# # sys.path.append("../")
# # from Utils import map_gen
# # from Utils import pcl_parser
# # from Utils import node_manager2 as node_manager
# # import time
# # from Utils import scenario_loader

# # class Executor:
# #     def __init__(self, planner = 'ego_planner', launch_file = 'run_in_sim.launch', ns = None, show_in_terminal = True):
# #         self.launch_node = node_manager.ROSLauncher(planner, launch_file, ns = ns, show_in_terminal = show_in_terminal)
# #         self.map_pub = map_gen.MapGenerator(0.1, height = 2.2, size = 0.2, z_global = -0.2, ns = ns)
# #         self.scenario = None

# #     def set_scenario(self, scenario):
# #         assert isinstance(scenario, pcl_parser.Scenario), "Error: must be a scenario type"
# #         self.scenario = scenario
# #         self.map_pub.clean_map()
# #         points = self.scenario.get_full_pcl()
# #         self.map_pub.add_to_ros_msg(points)

# #     def start(self):
# #         self.map_pub.map_pub()
# #         self.launch_node.start_launch()

# #     def stop(self):
# #         self.launch_node.stop_launch()
# #         self.map_pub.map_pub_stop()


# # if __name__ == '__main__':
# #     import sys
# #     loader = scenario_loader.Loader()
# #     scenario = loader.load(sys.argv[1])
# #     executor = Executor()
# #     executor.set_scenario(scenario)
# #     for i in range(1):
# #         executor.start()
# #         time.sleep(15)
# #         executor.stop()
    

# #!/usr/bin/env python
# # -*- coding: utf-8 -*-

# import os
# import sys
# import rospy

# from std_msgs.msg import Header
# from sensor_msgs.msg import PointCloud2
# import sensor_msgs.point_cloud2 as pc2


# CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
# MAP_PUBLISH_DIR = os.path.abspath(os.path.join(CURRENT_DIR, ".."))
# UTILS_DIR = os.path.join(MAP_PUBLISH_DIR, "Utils")

# if MAP_PUBLISH_DIR not in sys.path:
#     sys.path.insert(0, MAP_PUBLISH_DIR)

# if UTILS_DIR not in sys.path:
#     sys.path.insert(0, UTILS_DIR)

# from Utils import scenario_loader


# class ScenarioExecutor(object):
#     """
#     EGO-Swarm 专用地图发布节点。

#     作用：
#         1. 读取 scenario json
#         2. 生成点云
#         3. 发布到 /map_generator/global_cloud

#     注意：
#         这个版本不再启动 planner。
#         这个版本不再使用 node_manager。
#         这个版本不再调用 ROSLauncher。
#     """

#     def __init__(self):
#         rospy.init_node("scenario_executor")

#         default_json = os.path.join(MAP_PUBLISH_DIR, "1.json")

#         self.scenario_json = rospy.get_param(
#             "~scenario_json",
#             default_json
#         )

#         self.map_topic = rospy.get_param(
#             "~map_topic",
#             "/map_generator/global_cloud"
#         )

#         self.frame_id = rospy.get_param(
#             "~frame_id",
#             "world"
#         )

#         self.publish_rate = rospy.get_param(
#             "~publish_rate",
#             1.0
#         )

#         self.points = []

#         self.pub = rospy.Publisher(
#             self.map_topic,
#             PointCloud2,
#             queue_size=1,
#             latch=True
#         )

#     def normalize_points(self, raw_points):
#         points = []

#         for p in raw_points:
#             if p is None:
#                 continue

#             if len(p) < 3:
#                 continue

#             x = float(p[0])
#             y = float(p[1])
#             z = float(p[2])

#             points.append((x, y, z))

#         return points

#     def load_map(self):
#         loader = scenario_loader.Loader()
#         scenario = loader.load(self.scenario_json)

#         if scenario is None:
#             rospy.logerr("Failed to load scenario json: %s", self.scenario_json)
#             return False

#         raw_points = scenario.get_full_pcl()
#         self.points = self.normalize_points(raw_points)

#         if len(self.points) == 0:
#             rospy.logerr("Scenario map has no points: %s", self.scenario_json)
#             return False

#         rospy.logwarn(
#             "Custom map loaded: json=%s, points=%d, topic=%s, frame=%s",
#             self.scenario_json,
#             len(self.points),
#             self.map_topic,
#             self.frame_id
#         )

#         return True

#     def make_cloud_msg(self):
#         header = Header()
#         header.stamp = rospy.Time.now()
#         header.frame_id = self.frame_id

#         cloud_msg = pc2.create_cloud_xyz32(header, self.points)

#         return cloud_msg

#     def spin(self):
#         if not self.load_map():
#             return

#         rate = rospy.Rate(float(self.publish_rate))

#         while not rospy.is_shutdown():
#             cloud_msg = self.make_cloud_msg()
#             self.pub.publish(cloud_msg)
#             rate.sleep()


# if __name__ == "__main__":
#     node = ScenarioExecutor()
#     node.spin()
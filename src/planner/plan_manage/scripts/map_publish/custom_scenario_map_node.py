#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
UTILS_DIR = os.path.join(CURRENT_DIR, "Utils")

if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

if UTILS_DIR not in sys.path:
    sys.path.insert(0, UTILS_DIR)

from Utils import scenario_loader


def normalize_points(points):
    """
    保证点云格式是 [(x, y, z), ...]
    """
    normalized = []

    for p in points:
        if len(p) < 3:
            continue

        normalized.append((
            float(p[0]),
            float(p[1]),
            float(p[2])
        ))

    return normalized


def main():
    rospy.init_node("custom_scenario_map_node")

    scenario_json = rospy.get_param(
        "~scenario_json",
        os.path.join(CURRENT_DIR, "1.json")
    )

    map_topic = rospy.get_param(
        "~map_topic",
        "/map_generator/global_cloud"
    )

    frame_id = rospy.get_param(
        "~frame_id",
        "world"
    )

    publish_rate = rospy.get_param(
        "~publish_rate",
        1.0
    )

    loader = scenario_loader.Loader()
    scenario = loader.load(scenario_json)

    if scenario is None:
        rospy.logerr("Failed to load scenario json: %s", scenario_json)
        return

    raw_points = scenario.get_full_pcl()
    points = normalize_points(raw_points)

    if len(points) == 0:
        rospy.logerr("Scenario map has no points: %s", scenario_json)
        return

    rospy.logwarn(
        "Custom scenario map loaded: json=%s, points=%d, topic=%s, frame=%s",
        scenario_json,
        len(points),
        map_topic,
        frame_id
    )

    pub = rospy.Publisher(
        map_topic,
        PointCloud2,
        queue_size=1,
        latch=True
    )

    rate = rospy.Rate(float(publish_rate))

    while not rospy.is_shutdown():
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id

        cloud_msg = pc2.create_cloud_xyz32(header, points)
        pub.publish(cloud_msg)

        rate.sleep()


if __name__ == "__main__":
    main()
#!/usr/bin/env python

import rospy

from perception_planner.srv import PerceivePlan, PerceivePlanResponse
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points_list, create_cloud
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

import numpy as np

class PerceptionPlanner:
    def __init__(self):
        rospy.init_node('perception_planner')
        rospy.loginfo('Perception Planner Node Started')

        service_name = '/perception_planner_srv'
        s = rospy.Service(service_name, PerceivePlan, self.handle_perception_planner)
        
        pcd_sub = rospy.Subscriber('/points2', PointCloud2, self.pcd_callback)

        filtered_pub = rospy.Publisher('/filtered_pcd', PointCloud2, queue_size=10)

        rospy.loginfo('Service %s is ready', service_name)

        object_center = (0, -0.17)

        rospy.spin()
    
    def handle_perception_planner(self, req):
        rospy.loginfo('Perception Planner Service Called')

        shape = req.shape

        pcd_msg = rospy.wait_for_message('/points2', PointCloud2)

        self.frame_id = pcd_msg.header.frame_id


        return PerceivePlanResponse(True)
    
    def filter_pcd(self, pcd, x_range, y_range, z_range=None):
        x_mask = (pcd[:, 0] >= x_range[0]) & (pcd[:, 0] <= x_range[1])
        y_mask = (pcd[:, 1] >= y_range[0]) & (pcd[:, 1] <= y_range[1])
        
        if z_range is not None:
            z_mask = (pcd[:, 2] >= z_range[0]) & (pcd[:, 2] <= z_range[1])
            mask = x_mask & y_mask & z_mask
        else:
            mask = x_mask & y_mask
        return pcd[mask]
    
    def pcd_callback(self, msg):
        rospy.loginfo('Point Cloud Received')

        pcd_arr = np.array(list(read_points_list(msg)))
        pcd_arr = pcd_arr[:, :3]

        pcd_arr_filtered = self.filter_pcd(pcd_arr, (-0.2, 0.2), (-0.4, 0))

        filtered_msg = self.make_pc2_msg(pcd_arr_filtered)
        self.pcd_pub.publish(filtered_msg)



    def get_shape_points(self, shape):
        pass

    def find_depth(self, x, y) :
        pass

    def generate_path_message(self, path_points):
        pass

    def make_pc2_msg(self, pcd_arr):
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                ]

        header = Header()
        header.frame_id = self.frame_id
        header.stamp = rospy.Time.now()

        pc2_msg = create_cloud(header, fields, pcd_arr)
        return pc2_msg
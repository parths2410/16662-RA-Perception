#!/usr/bin/env python

import rospy

from perception_planner.srv import PerceivePlan, PerceivePlanResponse
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points_list

import numpy as np

class PerceptionPlanner:
    def __init__(self):
        rospy.init_node('perception_planner')
        rospy.loginfo('Perception Planner Node Started')

        service_name = '/perception_planner_srv'
        s = rospy.Service(service_name, PerceivePlan, self.handle_perception_planner)
        
        pcd_sub = rospy.Subscriber('/points2', PointCloud2, self.pcd_callback)

        rospy.loginfo('Service %s is ready', service_name)

        object_center = (0, -0.17)

        rospy.spin()
    
    def handle_perception_planner(self, req):
        rospy.loginfo('Perception Planner Service Called')

        shape = req.shape

        pcd_msg = rospy.wait_for_message('/points2', PointCloud2)

        # Extract 5x5 region - X (-0.2 to 0.2) and Y (0 to -0.4)
        self.pcd = np.array(read_points_list(pcd_msg, field_names=("x", "y", "z"), skip_nans=True))


        return PerceivePlanResponse(True)
    
    def filter_pcd(self, pcd, x_range, y_range, z_range=None):
        if x_range is not None:
            pcd = pcd[(pcd[:,0] >= x_range[0]) & (pcd[:,0] <= x_range[1])]
        if y_range is not None:
            pcd = pcd[(pcd[:,1] >= y_range[0]) & (pcd[:,1] <= y_range[1])]
        if z_range is not None:
            pcd = pcd[(pcd[:,2] >= z_range[0]) & (pcd[:,2] <= z_range[1])]
        
        return pcd
    
    def pcd_callback(self, msg):
        rospy.loginfo('Point Cloud Received')

    def get_shape_points(self, shape):
        pass

    def find_depth(self, x, y) :
        pass

    def generate_path_message(self, path_points):
        pass
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


        #sashank
        #center(offset)
        self.c_x = 2
        self.c_y = 2
        

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


    #helper functions for generating points
    def generate_rectangle_points(self, l, b):


        # Calculate corner points
        top_right = (l/2 + self.c_x, b/2 + self.c_y)
        top_left = (-l/2 + self.c_x, b/2 + self.c_y)
        bottom_left = (-l/2 + self.c_x, -b/2 + self.c_y)
        bottom_right = (l/2 + self.c_x, -b/2 + self.c_y)
        
        # Generate points along each edge, separated by 2 cm
        points = []
        
        # point generation separated by 2cm
        # Top edge
        points += [(x, top_right[1]) for x in np.arange(top_right[0], top_left[0], -2e-2)]
        # Left edge
        points += [(top_left[0], y) for y in np.arange(top_left[1], bottom_left[1], -2e-2)]
        # Bottom edge
        points += [(x, bottom_left[1]) for x in np.arange(bottom_left[0], bottom_right[0], 2e-2)]
        # Right edge
        points += [(bottom_right[0], y) for y in np.arange(bottom_right[1], top_right[1], 2e-2)]
        
        return points

    def generate_circle_points(self, r):
        # Determine the number of points needed: using circumference as 2*pi*r
        circumference = 2 * np.pi * r
        num_points = int(circumference / 2e-2)
        
        # Generate points along the circumference
        theta = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
        points = [(r * np.cos(t) + self.c_x, r * np.sin(t) + self.c_y) for t in theta]
        
        return points
        
    def get_shape_points(self, shape):
        self.points = self.generate_rectangle_points(10, 5)



    def find_depth(self, x, y):
        #error handling for points not set/point cloud not set
        if not hasattr(self, 'points') or not hasattr(self, 'pcd'):
            rospy.loginfo('Points or point cloud not set.')
            return []

        # Threshold for considering points in the point cloud near the points of interest
        x_threshold = 0.02  # 2 cm
        y_threshold = 0.02  # 2 cm

        depths = []
        for point in self.points:
            x_interest, y_interest = point

            # Filter the point cloud for points close to the point of interest
            close_points = self.filter_pcd(
                self.pcd,
                x_range=(x_interest - x_threshold, x_interest + x_threshold),
                y_range=(y_interest - y_threshold, y_interest + y_threshold)
            )

            # If no points are close, skip depth calculation
            if len(close_points) == 0:
                rospy.loginfo('No points found close to (%f, %f)', x_interest, y_interest)
                depths.append(None)  # Indicate no depth found with None
                continue

            # Calculate the average depth (z value) of the close points
            avg_depth = np.mean(close_points[:, 2])
            self.depths.append(avg_depth)

        

    def generate_path_message(self, path_points):
        pass
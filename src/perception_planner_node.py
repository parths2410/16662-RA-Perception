#!/usr/bin/env python

import rospy

from perception_planner.srv import PerceivePlan, PerceivePlanResponse
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points_list, create_cloud
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from nav_msgs.msg import Path, PoseStamped

import numpy as np

class ShapeGenerator:
    def __init__(self, c_x, c_y, step_size=2e-2):
        self.c_x = c_x
        self.c_y = c_y
        self.step_size = step_size
    
    def generate_rectangle_points(self, l, b):
        # Calculate corner points
        top_right = (l/2 + self.c_x, b/2 + self.c_y)
        top_left = (-l/2 + self.c_x, b/2 + self.c_y)
        bottom_left = (-l/2 + self.c_x, -b/2 + self.c_y)
        bottom_right = (l/2 + self.c_x, -b/2 + self.c_y)
        
        # Generate points along each edge, separated by 2 mm
        points = []

        # Top edge
        points += [(x, top_right[1]) for x in np.arange(top_right[0], top_left[0], -self.step_size)]
        # Left edge
        points += [(top_left[0], y) for y in np.arange(top_left[1], bottom_left[1], -self.step_size)]
        # Bottom edge
        points += [(x, bottom_left[1]) for x in np.arange(bottom_left[0], bottom_right[0], self.step_size)]
        # Right edge
        points += [(bottom_right[0], y) for y in np.arange(bottom_right[1], top_right[1], self.step_size)]
        
        return points
    
    def generate_circle_points(self, r):
        # Determine the number of points needed, approximating circumference as 2*pi*r
        circumference = 2 * np.pi * r
        num_points = int(circumference / self.step_size)
        
        # Generate points along the circumference
        theta = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
        points = [(r * np.cos(t) + self.c_x, r * np.sin(t) + self.c_y) for t in theta]
        
        return points
        
class PerceptionPlanner:
    def __init__(self):
        rospy.init_node('perception_planner')
        rospy.loginfo('Perception Planner Node Started')

        service_name = '/perception_planner_srv'
        s = rospy.Service(service_name, PerceivePlan, self.handle_perception_planner)
        
        # pcd_sub = rospy.Subscriber('/points2', PointCloud2, self.pcd_callback)

        self.filtered_pub = rospy.Publisher('/filtered_pcd', PointCloud2, queue_size=10)

        rospy.loginfo('Service %s is ready', service_name)

        object_center = (0, -0.17)
        self.shape_gen = ShapeGenerator(*object_center, step_size=2e-2)

        rospy.spin()
    
    def handle_perception_planner(self, req):
        rospy.loginfo('Perception Planner Service Called')

        shape = req.shape
        print("Shape received ", shape)

        print("Waiting for Pointcloud")
        pcd_msg = rospy.wait_for_message('/points2', PointCloud2)
        self.frame_id = pcd_msg.header.frame_id
        print("Pointcloud received. In frame - ", self.frame_id)

        print("Numpying pcd")
        pcd_np = np.array(list(read_points_list(pcd_msg)))[:, :3]
        print("PCD Shape - ", pcd_np.shape)

        print("Filtering")
        pcd_np_filtered = self.filter_pcd(pcd_np, (-0.2, 0.2), (-0.4, 0))
        self.pcd = pcd_np_filtered
        print("PCD Filtered Shape - ", pcd_np_filtered.shape)
        
        print("Publishing filtered")
        filtered_msg = self.generate_pc2_msg(pcd_np_filtered)
        self.filtered_pub.publish(filtered_msg)
        
        path_2d = self.get_shape_points(shape)
        path_3d = self.project_to_pcd(path_2d)

        path_msg = self.generate_path_message(path_3d)

        return PerceivePlanResponse(path_msg)
    
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
 
        filtered_msg = self.generate_pc2_msg(pcd_arr_filtered)
        self.pcd_pub.publish(filtered_msg)

    def get_shape_points(self, shape):
        if shape == "rectangle":
            l, b = 0.24, 0.15
            points = self.shape_gen.generate_rectangle_points(l, b)
        elif shape == "circle":
            r = 0.12
            points = self.shape_gen.generate_circle_points(r)
        elif shape == "square":
            l, b = 0.24, 0.24
            points = self.shape_gen.generate_rectangle_points(l, b)

        return points

    def find_depth(self, x, y) :
        thresh = 0.001  # 1mm
        pcd = self.pcd

        mask = (pcd[:, 0] >= x - thresh) & (pcd[:, 0] <= x + thresh) & (pcd[:, 1] >= y - thresh) & (pcd[:, 1] <= y + thresh)
        depth = pcd[mask][:, 2]
        return np.mean(depth)

    def project_to_pcd(self, path_2d):
        path_3d = []
        for x, y in path_2d:
            depth = self.find_depth(x, y)
            path_3d.append((x, y, depth))
        return np.array(path_3d)

    def generate_path_message(self, path_points):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = rospy.Time.now()
        for x, y, z in path_points:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            path.poses.append(pose)

        return path


    def generate_pc2_msg(self, pcd_arr):
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                ]

        header = Header()
        header.frame_id = self.frame_id
        header.stamp = rospy.Time.now()

        pc2_msg = create_cloud(header, fields, pcd_arr)
        return pc2_msg

if __name__=="__main__":
    p = PerceptionPlanner()
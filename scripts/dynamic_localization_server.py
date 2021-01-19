#! /usr/bin/env python

import rospy
import actionlib
import numpy as np

from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import OccupancyGrid
from multi_robot_nav.msg import DynamicLocalizationResult, DynamicLocalizationFeedback, DynamicLocalizationAction
from multi_robot_nav.msg import LocalizationData

from nav_msgs.srv import GetMap

class DynamicLocalizationServer:
    
    def __init__(self, name):
        self.action_name = name
        self._as = actionlib.SimpleActionServer(self.action_name, DynamicLocalizationAction, execute_cb=self.exec_cb, auto_start=False)
        self.rate = rospy.Rate(10)
        self.dynamic_pose = None
        self.local_map = None
        self.global_map = None
        self.feedback_msg = DynamicLocalizationFeedback()

        self._as.start()

    def get_maps(self):
        self.local_map = rospy.wait_for_message('move_base/local_costmap/costmap', OccupancyGrid)
        
        rospy.wait_for_service('static_map')
        static_map = rospy.ServiceProxy('static_map', GetMap)
        self.global_map = static_map().map


    def exec_cb(self, goal):
        self.dynamic_pose = rospy.wait_for_message('amcl_pose', PoseWithCovariance)
        self.get_maps()

        self.feedback_msg.message = 'Acquired local and global maps'
        self._as.publish_feedback(self.feedback_msg)

        print("Extract points from local map")
        self.local_points = np.array(self.local_map.data).reshape((self.local_map.width,self.local_map.height))
        self.local_position = np.array([self.local_map.origin_x,self.local_map.origin_y])
	
        print("Filter by global map")
        self.global_points = np.array(self.global_map.data).reshape((self.global_map.width,self.global_map.height))
        self.global_position = np.array([self.global_map.origin_x,self.global_map.origin_y])
        self.global_local_points_index = (self.local_position-self.global_position)/0.05
        print("cutting the points we need")
        self.global_points = self.global_points[global_local_points_index[0]:self.local_map.width,global_local_points_index[1],self.local_map.height]
        mask=(self.global_points<90) & (self.local_points>90) #mask to capture the points, have false and True in
        rows,cols=np.where(mask)
        self.suspicus_points= self.global_points[mask] #the value of the points in global
        self.coords=list(zip(rows,cols))
        print(len(self.coords))

        # Clustering

        
        # Fill in data
        result = LocalizationData()
        result.pose = self.dynamic_pose
        
        # Publish and return result
        pub = rospy.Publisher('dynamic_localization_data', LocalizationData, queue_size=1)
        pub.publish(result)
        self._as.set_succeeded(result)
        


if __name__ == '__main__':
    rospy.init_node('dynamic_localization_server')
    server = DynamicLocalizationServer(rospy.get_name())
    rospy.spin()

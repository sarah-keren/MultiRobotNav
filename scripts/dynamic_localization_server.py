#! /usr/bin/env python

import rospy
import actionlib

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
        # Extract points from local map


        # Filter by global map

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

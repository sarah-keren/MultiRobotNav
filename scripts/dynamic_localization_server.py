#! /usr/bin/env python

import rospy
import actionlib
import numpy as np
from scipy.spatial.distance import cdist

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance
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
        rospy.loginfo('Dynamic localization has started')
        self.feedback_msg.message = 'Dynamic localization has started'
        self._as.publish_feedback(self.feedback_msg)

        self.dynamic_pose = rospy.wait_for_message('amcl_pose', PoseWithCovarianceStamped)
        self.get_maps()

        self.feedback_msg.message = 'Acquired local and global maps'
        self._as.publish_feedback(self.feedback_msg)

        print("Extract points from local map")
        self.local_points = np.transpose(np.array(self.local_map.data).reshape(
                                (self.local_map.info.width, self.local_map.info.height)))
        self.local_position = np.array([self.local_map.info.origin.position.x,                      
                                        self.local_map.info.origin.position.y])
	
        print("Filter by global map")
        self.global_points = np.array(self.global_map.data).reshape(
                                (self.global_map.info.width, self.global_map.info.height))
        self.global_position = np.array([self.global_map.info.origin.position.x,
                                         self.global_map.info.origin.position.y])
        self.global_local_points_index = (self.local_position - self.global_position) / 0.05
        global_to_local_x = int(self.global_local_points_index[0])
        global_to_local_y = int(self.global_local_points_index[1])
        
        self.summed_global_map = np.zeros_like(self.local_points)

        radius=2
        print("cutting the points we need")

        for i,j in [(-radius+x,-radius+y)for x in range(2*radius) for y in range (2*radius)]:
            self.global_points_crop = self.global_points[global_to_local_x+i: global_to_local_x+i+self.local_map.info.width,
                                                         global_to_local_y+j: global_to_local_y+j+self.local_map.info.height]
            self.summed_global_map += self.global_points_crop

        mask = (self.local_points>99.0) & (self.summed_global_map > -1.0) & (self.summed_global_map < 90.0)  # mask to capture the points, have false and True in
        rows,cols = np.where(mask)
        self.suspicus_points = self.summed_global_map[mask]  # the value of the points in global

        self.coords = np.array(zip(rows,cols))
        rospy.loginfo('coords len')
        rospy.loginfo(len(self.coords))

        # Clustering

        dist_mat = cdist(self.coords, self.coords, metric='euclidean')
        dist_mat[dist_mat > 3] = 0.0  # Drop distances bigger than the robot's radius / map's resolution
        cluster_row = np.argmax(np.count_nonzero(dist_mat, axis=0))
        biggest_cluster_mat = self.coords[np.nonzero(dist_mat[cluster_row])]
        biggest_cluster_mat = np.append(biggest_cluster_mat, [self.coords[cluster_row]], axis=0)
        rospy.loginfo('Biggest cluster')
        print(biggest_cluster_mat)

        cluster_center = np.mean(biggest_cluster_mat, axis=0) * 0.05
        static_point = self.local_position + cluster_center
        rospy.loginfo('Static point')
        print(static_point)

        # Fill in default data
        result = DynamicLocalizationResult()
        result.data.pose = self.dynamic_pose.pose

        static_pose = PoseWithCovariance()
        static_pose.pose.position.x = static_point[0]
        static_pose.pose.position.y = static_point[1]
        result.data.static_pose = static_pose
        
        # Publish and return result
        pub = rospy.Publisher('/dynamic_localization_data', LocalizationData, queue_size=1)
        pub.publish(result.data)
        
        self._as.set_succeeded(result)
        


if __name__ == '__main__':
    rospy.init_node('dynamic_localization_server')
    server = DynamicLocalizationServer(rospy.get_name())
    rospy.spin()

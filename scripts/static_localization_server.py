#! /usr/bin/env python

import rospy

import actionlib

from multi_robot_nav.msg import StaticLocalizationResult, StaticLocalizationFeedback, StaticLocalizationAction, LocalizationData
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty

class StaticLocalizationServer:
    
    def __init__(self, name):
        self.action_name = name
        self._as = actionlib.SimpleActionServer(self.action_name, StaticLocalizationAction, execute_cb=self.exec_cb, auto_start=False)
        self.pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.static_pose = PoseWithCovarianceStamped()

        self._as.start()
    
    def make_pose_stamped(self, msg):

        self.static_pose = PoseWithCovarianceStamped()
        self.static_pose.pose = msg.static_pose
        self.static_pose.pose.pose.orientation.w = 1.0
        self.static_pose.header.stamp = rospy.Time.now()
        self.static_pose.header.frame_id = '/map'

    def exec_cb(self, goal):
        rospy.loginfo('Started Static Server')
        msg = rospy.wait_for_message('/dynamic_localization_data', LocalizationData)
        self.make_pose_stamped(msg)
        
        self.pub.publish(self.static_pose)
        rospy.loginfo('Published static pose')

        rospy.sleep(2)

        rospy.wait_for_service('request_nomotion_update')
        update_amcl = rospy.ServiceProxy('request_nomotion_update', Empty)
        update_amcl()
        rospy.loginfo('Updated amcl')

        result = StaticLocalizationResult()
        result.pose = self.static_pose.pose
        self._as.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('static_localization_server')
    server = StaticLocalizationServer(rospy.get_name())
    rospy.spin()

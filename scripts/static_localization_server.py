#! /usr/bin/env python

import rospy

import actionlib

from multi_robot_nav.msg import StaticLocalizationResult, StaticLocalizationFeedback, StaticLocalizationAction

class StaticLocalizationServer:
    
    def __init__(self, name):
        self.action_name = name
        self._as = actionlib.SimpleActionServer(self.action_name, StaticLocalizationAction, execute_cb=self.exec_cb, auto_start=False)
        self.rate = rospy.Rate(10)
        self._as.start()

    def exec_cb(self, goal):
        pass



if __name__ == '__main__':
    rospy.init_node('static_localization_server')
    server = StaticLocalizationServer(rospy.get_name())
    rospy.spin()

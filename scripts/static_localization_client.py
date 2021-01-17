#! /usr/bin/env python

import rospy
import actionlib

from multi_robot_nav.msg import StaticLocalizationGoal, StaticLocalizationAction

class StaticLocalizationClient:

    def __init__(self, name):
        self.action_name = name
        self.client = actionlib.SimpleActionClient(self.action_name, StaticLocalizationAction)

    def run_client(self):
        self.client.wait_for_server()

        goal = StaticLocalizationGoal()

        self.client.send_goal(goal)

        self.client.wait_for_result()

        return self.client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('static_localization_client')
        client = StaticLocalizationClient(rospy.get_name())
        result = client.run_client()

    except rospy.ROSInterruptException:
        rospy.loginfo('static_localization_client has finished')

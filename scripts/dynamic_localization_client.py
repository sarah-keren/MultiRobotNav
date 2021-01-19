#! /usr/bin/env python

import rospy
import actionlib

from multi_robot_nav.msg import DynamicLocalizationGoal, DynamicLocalizationAction

class DynamicLocalizationClient:

    def __init__(self, name):
        self.action_name = name
        self.client = actionlib.SimpleActionClient(self.action_name, DynamicLocalizationAction)

    def run_client(self):
	print ("waiting for server")
        self.client.wait_for_server()
	print("sending message")
        goal = DynamicLocalizationGoal()

        self.client.send_goal(goal)

        self.client.wait_for_result()

        return self.client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('dynamic_localization_client')
        client = DynamicLocalizationClient(rospy.get_name())
        result = client.run_client()

    except rospy.ROSInterruptException:
        rospy.loginfo('dynamic_localization_client has finished')

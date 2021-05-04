#! /usr/bin/env python

import rospy
import numpy as np
import random
import actionlib

from geometry_msgs.msg import PoseStamped,PoseArray
from nav_msgs.srv import GetPlan
from rospy.numpy_msg import numpy_msg

class PathMetrics:
    
    def __init__(self, ns='/', k=100):
        self.ns = ns
        self.k = k
        rospy.loginfo('Started Metric calculation, with sample size {0} and in ns {1}'.format(k, ns))

        pf_topic = self.ns + 'particlecloud'
        msg = rospy.wait_for_message(pf_topic, PoseArray)
        possible_poses_array = msg.poses
        rospy.loginfo('Number of possible start locations {}'.format(len(possible_poses_array)))

        array_pose = [(item.position.x,item.position.y) for item in possible_poses_array]
        self.array_sample = random.sample(array_pose, self.k)
	
    def getVariance(self,goal_location):
        goal = PoseStamped()
        goal.header.seq = 0
        goal.header.frame_id = self.ns + 'map'
        goal.header.stamp = rospy.Time(0)
        goal.pose.position.x = goal_location[0] 
        goal.pose.position.y = goal_location[1]  
        array_lens=np.zeros(self.k)
        
        start = PoseStamped()
        start.header.seq = 0
        start.header.frame_id = self.ns + 'map'
        start.header.stamp = rospy.Time(0)
        start.pose.position.x = 0.5
        start.pose.position.y = -2
        rospy.loginfo('Size of path from real start point')
        rospy.loginfo(self.getPlanSize(start,goal))
        
        for i, item in enumerate(self.array_sample):
            start = PoseStamped()
            start.header.seq = 0
            start.header.frame_id = self.ns + 'map'
            start.header.stamp = rospy.Time(0)
            start.pose.position.x = item[0]
            start.pose.position.y = item[1]
            array_lens[i] = self.getPlanSize(start,goal)

        var=np.var(array_lens)
        mean=np.mean(array_lens)

        rospy.loginfo('Result of Path size metric:')
        rospy.loginfo('Variance: {}'.format(var))
        rospy.loginfo('Mean: {}'.format(mean))

        return var,mean
	
    def getPlanSize(self, start, goal, show=False):
        get_plan_srv = self.ns + 'move_base/make_plan'
        get_plan = rospy.ServiceProxy(get_plan_srv, GetPlan)
        req = GetPlan()
        req.start = start
        req.goal = goal
        req.tolerance = .5
        resp = get_plan(req.start, req.goal, req.tolerance)

        if show:
            rospy.logdebug(resp.plan)

        return len(resp.plan.poses)


if __name__ == '__main__':
    rospy.init_node('path_length_metric')
    path_metrics = PathMetrics(ns='/tb3_0/', k=100)
    
    path_metrics.getVariance((1,2))

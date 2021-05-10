#! /usr/bin/env python

import rospy
import numpy as np
import random
import actionlib

from geometry_msgs.msg import PoseStamped,PoseArray
from nav_msgs.srv import GetPlan, GetPlanResponse
from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

class PathMetrics:
    
    def __init__(self, ns='/', k=100):
        self.ns = ns
        self.k = k
        self.global_origin = None
        self.global_map = None
        rospy.loginfo('Started Metric calculation, with sample size {0} and in ns {1}'.format(k, ns))

        pf_topic = self.ns + 'particlecloud'
        msg = rospy.wait_for_message(pf_topic, PoseArray)
        possible_poses_array = msg.poses
        rospy.loginfo('Number of possible start locations {}'.format(len(possible_poses_array)))

        array_pose = [(item.position.x,item.position.y) for item in possible_poses_array]
        self.array_sample = random.sample(array_pose, self.k)

        self.get_map()
        self.heat_map = np.zeros((self.global_map.info.width, self.global_map.info.height))
        # self.numpize_response()
	
    def get_metrics_to_goal(self,goal_location):
        goal = PoseStamped()
        goal.header.seq = 0
        goal.header.frame_id = self.ns + 'map'
        goal.header.stamp = rospy.Time(0)
        goal.pose.position.x = goal_location[0] 
        goal.pose.position.y = goal_location[1]  
        size_array=np.zeros(self.k)
        
        # start = PoseStamped()
        # start.header.seq = 0
        # start.header.frame_id = self.ns + 'map'
        # start.header.stamp = rospy.Time(0)
        # start.pose.position.x = 0.5
        # start.pose.position.y = -2

        # real_size = self.single_plan_analysis(start,goal)
        # rospy.loginfo('Size of path from real start point: {}'.format(real_size))
        
        
        for i, item in enumerate(self.array_sample):
            start = PoseStamped()
            start.header.seq = 0
            start.header.frame_id = self.ns + 'map'
            start.header.stamp = rospy.Time(0)
            start.pose.position.x = item[0]
            start.pose.position.y = item[1]
            size_array[i] = self.single_plan_analysis(start,goal)
        
        self.get_size_variance_metric(size_array)
        self.get_heatmap_analysis()
	
    def single_plan_analysis(self, start, goal, show=False):
        get_plan_srv = self.ns + 'move_base/make_plan'
        get_plan = rospy.ServiceProxy(get_plan_srv, GetPlan)
        req = GetPlan()
        req.start = start
        req.goal = goal
        req.tolerance = .5
        resp = get_plan(req.start, req.goal, req.tolerance)

        steps_array = resp.plan.poses

        if show:
            rospy.logdebug(resp.plan)
        
        size_metric = len(steps_array)
        
        self.fill_single_path_heatmap(steps_array)

        return size_metric
    
    def get_size_variance_metric(self, size_array):
        var=np.var(size_array)
        mean=np.mean(size_array)

        rospy.loginfo('Result of Path size metric:')
        rospy.loginfo('Variance: {}'.format(var))
        rospy.loginfo('Mean: {}'.format(mean))
    
    def fill_single_path_heatmap(self, steps_array):
        xy_array = np.empty((len(steps_array), 2))

        for i, pose in enumerate(steps_array):
            position = pose.pose.position
            xy_array[i, 0] = position.x
            xy_array[i, 1] = position.y

        grid_poses = xy_array - self.global_origin
        grid_indices = (grid_poses / 0.05).astype(np.int)

        self.heat_map[grid_indices] += 1
    
    def get_heatmap_analysis(self):
        var = np.var(self.heat_map)

        rospy.loginfo('Result of Path Heatmap metric:')
        rospy.loginfo('Variance: {}'.format(var))
    
    def get_map(self):
        self.local_map = rospy.wait_for_message(self.ns + 'move_base/local_costmap/costmap', OccupancyGrid)
        
        rospy.wait_for_service(self.ns + 'static_map')
        static_map = rospy.ServiceProxy(self.ns + 'static_map', GetMap)
        self.global_map = static_map().map

        self.global_origin = np.array([self.global_map.info.origin.position.x,
                                       self.global_map.info.origin.position.y])

    # def numpize_response(self):
        # GetPlan._response_class = rospy.numpy_msg.numpy_msg(GetPlanResponse)
        # GetPlanResponse = rospy.numpy_msg.numpy_msg(GetPlanResponse)

if __name__ == '__main__':
    rospy.init_node('path_length_metric')
    path_metrics = PathMetrics(ns='/tb3_0/', k=100)

    path_metrics.get_metrics_to_goal((1,2))

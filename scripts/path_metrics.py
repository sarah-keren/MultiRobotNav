#! /usr/bin/env python

import rospy
import numpy as np
import random
import actionlib
import sys

from geometry_msgs.msg import PoseStamped,PoseArray
from std_msgs.msg import Header
from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import OccupancyGrid, MapMetaData
from nav_msgs.srv import GetMap, GetPlan, GetPlanResponse

class PathMetrics:
    
    def __init__(self, ns='/', k=100,real_start=None,global_origin=None,resolution=None):
        """
        initilize metric settings
        """
        self.ns = ns
        self.k = k
        self.global_origin = global_origin
        self.resolution = resolution
        self.global_map = None
        self.global_meta_data = None
        self.real_start = real_start
        rospy.loginfo('Started Metric calculation, with sample size {0} and in ns {1}'.format(k, ns))

        pf_topic = self.ns + 'particlecloud'
        msg = rospy.wait_for_message(pf_topic, PoseArray)
        possible_poses_array = msg.poses
        rospy.loginfo('Number of possible start locations {}'.format(len(possible_poses_array)))

        array_pose = [(item.position.x,item.position.y) for item in possible_poses_array]
        self.array_sample = random.sample(array_pose, self.k)
        self.start_sample = random.sample(array_pose, 10)

        self.get_map()
        self.heat_map = np.zeros_like(self.global_map)
        self.counter_along_path = []
        self.distance_replan_paths = []
        self.steps_replan_array = []
        self.notNeed_to_replan_array=[]
        # self.numpize_response()
	
    def get_metrics_to_goal(self,goal_location):
        """
        return all 4 metric results.
        """
        goal = PoseStamped()
        goal.header.seq = 0
        goal.header.frame_id = self.ns + 'map'
        goal.header.stamp = rospy.Time(0)
        goal.pose.position.x = goal_location[0] 
        goal.pose.position.y = goal_location[1]  
        size_array = np.zeros(self.k)        
        
        for i, item in enumerate(self.array_sample):
            start = PoseStamped()
            start.header.seq = 0
            start.header.frame_id = self.ns + 'map'
            start.header.stamp = rospy.Time(0)
            start.pose.position.x = item[0]
            start.pose.position.y = item[1]
            size_array[i] = self.single_plan_analysis(start,goal)
        
        result0 = None
        if self.real_start is not None:
            real_start = PoseStamped()
            real_start.header.seq = 0
            real_start.header.frame_id = self.ns + 'map'
            real_start.header.stamp = rospy.Time(0)
            real_start.pose.position.x = self.real_start[0] 
            real_start.pose.position.y = self.real_start[1]

            get_plan_srv = self.ns + 'move_base/make_plan'
            get_plan = rospy.ServiceProxy(get_plan_srv, GetPlan)
            
            req = GetPlan()
            req.start = start
            req.goal = goal
            req.tolerance = .5
            resp = get_plan(req.start, req.goal, req.tolerance)
            result0 = len(resp.plan.poses)

        result1 = self.get_size_variance_metric(size_array)
        result2 = self.get_heatmap_analysis()
        result3 = self.get_localization_along_path_analysis()
        result4 = self.get_how_many_replans()
        return result0, result1, result2, result3, result4
	
    def single_plan_analysis(self, start, goal, show=False):
        """
        create all testing for a single path path
        """
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
        
        grid_indices = self.reduce_poses_resolution(steps_array,len(self.heat_map[0]),len(self.heat_map))
        if len(grid_indices) > 0:
            self.distance_replan_paths.append(self.will_do_replan(\
            self.plan_by_grid(grid_indices),steps_array,goal))

        x_index = grid_indices[:, 0]
        y_index = grid_indices[:, 1]
        
        self.heat_map[y_index, x_index] += 1

        self.counter_along_path.append(self.get_localization_along_path(grid_indices))

        return size_metric

    def plan_by_grid(self,grid_indices):
        """
        get the plan corrisponding to the grid path.
        """
        previous_location = grid_indices[0]
        plan_grid=[]
        
        for i,point in enumerate(grid_indices):
            if i==0: continue
            plan_grid.append((previous_location[0]-point[0],previous_location[1]-point[1]))
            previous_location = point

        return plan_grid

    def get_size_variance_metric(self, size_array):
        """
        final cacluation of metric 1 variation of size
        """
        var=np.var(size_array)
        mean=np.mean(size_array)

        rospy.loginfo('Result of Path size metric:')
        rospy.loginfo('Variance: {}'.format(var))
        rospy.loginfo('Mean: {}'.format(mean))
        return var,mean
    
    def reduce_poses_resolution(self, steps_array,width,height):
        """
        take a path and reduce it to grid
        """
        xy_array = np.empty((len(steps_array), 2))
        index_to_get=[]
        for i, pose in enumerate(steps_array):
            position = pose.pose.position
            xy_array[i, 0] = position.x
            xy_array[i, 1] = position.y
            if self.global_origin[0] < position.x < (self.global_origin[0] + width * self.resolution) and\
                self.global_origin[1] < position.y < (self.global_origin[1] + height * self.resolution):
                index_to_get.append(i)
        
        final_array = np.empty((len(index_to_get), 2))
        for i,pose in enumerate(index_to_get):
            final_array[i,0] = xy_array[pose,0]
            final_array[i,1] = xy_array[pose,1]

        grid_poses = final_array - self.global_origin
        grid_indices = (grid_poses / self.resolution).astype(np.int)
        return grid_indices
        
    def get_heatmap_analysis(self):
        """
        final calcuation of metric 2, calc heatmap variation not counting 0
        """
        #pub = rospy.Publisher(self.ns + 'heatmap', OccupancyGrid, queue_size=1)
        
        #oc_grid = OccupancyGrid()
        #oc_grid.data = self.heat_map.flatten()
        #oc_grid.info = self.global_meta_data

        #while(pub.get_num_connections() < 1):
        #    rospy.sleep(1)

        #pub.publish(oc_grid)

        heat_map_array_nonZero=self.heat_map.flatten()
        heat_map_array_nonZero=heat_map_array_nonZero[np.nonzero(heat_map_array_nonZero)]

        mean = np.mean(heat_map_array_nonZero) #np.mean(self.heat_map)
        var = np.var(heat_map_array_nonZero) #np.var(self.heat_map)
        non_zero_count = np.count_nonzero(self.heat_map)
        non_zero_percent = non_zero_count / float(self.heat_map.shape[0]*self.heat_map.shape[1])

        rospy.loginfo('Result of Path Heatmap metric:')
        rospy.loginfo('Variance: {}'.format(var))
        rospy.loginfo('Mean: {}'.format(mean))
        rospy.loginfo('Non-zero %: {}'.format(non_zero_percent))
        return var, mean, non_zero_count, non_zero_percent
    
    def get_map(self):
        """
        getting the map data and it's settings
        """
        self.local_map = rospy.wait_for_message(self.ns + 'move_base/local_costmap/costmap', OccupancyGrid)
        
        rospy.wait_for_service(self.ns + 'static_map')
        static_map_srv = rospy.ServiceProxy(self.ns + 'static_map', GetMap)
        static_map = static_map_srv().map
        self.global_map = np.array(static_map.data).reshape(
                                (static_map.info.width, static_map.info.height))

        #self.global_origin = np.array([static_map.info.origin.position.x,
        #                               static_map.info.origin.position.y])
        self.global_meta_data = static_map.info
    
    def get_localization_along_path(self, grid_indices):
        """
        single calcuation of metric 3, looks on the grid path and check on radius couting walls
        """
        counter = 0

        steps = 10
        radius = 10
        obstacle_threshold = 0.5

        for i in range(0, len(grid_indices), steps):
            x, y = grid_indices[i]
            map_in_radius = np.array(self.global_map[x - radius : x + radius, y - radius: y + radius])
            counter += np.count_nonzero(map_in_radius > obstacle_threshold)
        
        return counter
    
    def get_localization_along_path_analysis(self):
        """
        final cacluation of metric 3
        """
        mean = np.mean(self.counter_along_path)

        rospy.loginfo('Result of Localization Along Path metric:')
        rospy.loginfo('Mean: {}'.format(mean))
        return mean
        

    def will_do_replan(self,fake_grid_plan,step_array,goal):
        """
        calcuation per single path how many replans
        """        
        from math import sqrt
        
        radius = 1
        obstacle_threshold = 0.5
        multi_start_results = []
        steps_till_replan_array = []
        success_percent=[]

        for false_start in self.start_sample:
            previous_point = ((np.array(false_start) - np.array(self.global_origin)) / self.resolution).astype(np.int)
            index_in_array=0
            for i, point in enumerate(fake_grid_plan):
                x, y = (previous_point[0] + point[0], previous_point[1] + point[1])
                map_in_radius = np.array(self.global_map[x - radius : x + radius, y - radius: y + radius])
                if np.count_nonzero(map_in_radius > obstacle_threshold)>0: #we hit a wall
                    break
                previous_point = (x,y)
                index_in_array += 1

            steps_till_replan_array.append(index_in_array)    
            success_percent.append(len(fake_grid_plan) == index_in_array)

            pose = step_array[index_in_array].pose.position    
            goal_pose=goal.pose.position

            distance_goal = sqrt((goal_pose.x - pose.x)**2 + (goal_pose.y - pose.y)**2)
            multi_start_results.append(distance_goal)

        self.steps_replan_array.append(np.mean(steps_till_replan_array))
        self.notNeed_to_replan_array.append(len([x for x in success_percent if x])/10.0)
        
        return np.mean(multi_start_results)

    def get_how_many_replans(self):
        """
        final calculation of how many on average it will do replans
        """
        #non_zeros = np.count_nonzero(np.array(self.counter_replan_paths)>0.5)
        percent = np.mean(self.notNeed_to_replan_array) #(float(non_zeros) / float(len(self.counter_replan_paths))) * 100
        mean_steps = np.mean(self.steps_replan_array)
        mean = np.mean(self.distance_replan_paths)

        rospy.loginfo('Result of replans Along fake Paths metric:')
        rospy.loginfo('precent: {}%'.format(percent))
        rospy.loginfo('mean step: {}'.format(mean_steps))
        rospy.loginfo('mean distance: {}'.format(mean))

        return percent,mean_steps,mean


if __name__ == '__main__':
    rospy.init_node('path_length_metric')
    print("first robot")
    real=[float(x) for x in sys.argv[1].split(',')]
    global_origin = [float(x) for x in sys.argv[2].split(',')]
    res = float(sys.argv[3])
    path_metrics = PathMetrics(real_start=real,global_origin=global_origin,resolution=res,k=100)
    path_metrics.get_metrics_to_goal((1,2))

#! /usr/bin/env python
import roslaunch
import rospy
import rospkg
import numpy as np
from path_metrics import *
import random
import time
import pandas as pd
from os.path import exists
import sys
import subprocess

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance
from nav_msgs.msg import Odometry, Path
from tf.transformations import quaternion_from_euler
import yaml
from yaml import SafeLoader

def running_single():
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    
    rospack = rospkg.RosPack()
    pkg_dir = rospack.get_path('multi_robot_nav')

    launch = roslaunch.parent.ROSLaunchParent(uuid, [pkg_dir + "/launch/tests/test_runner.launch"])
    return launch
  
def create_test_launch(real, fake, world_path=None, map_path=None):

    if world_path is None:
        world_path="$(find turtlebot3_gazebo)/worlds/turtlebot3_world.world"
        map_path="$(find multi_robot_nav)/config/map.yaml"
    rospack = rospkg.RosPack()
    pkg_dir = rospack.get_path('multi_robot_nav')

    name = pkg_dir + '/launch/tests/test_runner.launch'
    with open(name,'w') as file:
        file.write(\
        "<launch>\n\t\
            <include file=\"$(find multi_robot_nav)/launch/tests/test_world.launch\">\n\t\
                <arg name=\"real_x\" value=\""+str(real[0])+"\"/>\n\t\
                <arg name=\"real_y\" value=\""+str(real[1])+"\"/>\n\t\
                <arg name=\"fake_x\" value=\""+str(fake[0])+"\"/>\n\t\
                <arg name=\"fake_y\" value=\""+str(fake[1])+"\"/>\n\t\
                <arg name=\"world_location\" value=\""+str(world_path)+"\"/>\n\t\
                <arg name=\"map_file\" value=\""+str(map_path)+"\"/>\n\t\
            </include>\n\
        </launch>")

def set_real_position(position):
    pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
    static_pose = PoseWithCovariance()
    static_pose.pose.position.x = position[0]
    static_pose.pose.position.y = position[1]

    new_pose = PoseWithCovarianceStamped()
    new_pose.pose = static_pose
    new_pose.pose.pose.orientation.w = 1.0
    new_pose.header.stamp = rospy.Time.now()
    new_pose.header.frame_id = '/map'
    
    pub.publish(new_pose)

def get_robot_location():
    odom_msg = rospy.wait_for_message('odom', Odometry)
    position = odom_msg.pose.pose.position
    return position.x,position.y

def goalTargetMaking(goal):
    goalTarget = MoveBaseGoal()
    goalTarget.target_pose.header.frame_id = "map"
    goalTarget.target_pose.header.stamp = rospy.Time.now()
    goalTarget.target_pose.pose.position.x = goal[0]
    goalTarget.target_pose.pose.position.y = goal[1]
    goalTarget.target_pose.pose.orientation.w = 1.0
    return goalTarget

def running_on_map(expirement=None,start=None,end=None):
        
    row={}
    create_test_launch(real, fake if expirement==1 else real, world_path, map_path)

    core=subprocess.Popen(['roscore'])
    print("starting Core")
    time.sleep(20)

    rospy.init_node('move_base_sequence')

    ending_string="without_oracle" if expirement == 1 else "oracle"
            
    print("running launch")
    launch = running_single()
    launch.start()
    time.sleep(10)
        
    print("making goal")
       
    goalMsg=goalTargetMaking(goal)
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server(rospy.Duration(10))
        
    if experiment == 0:
        print("setting position")
    else:
        path_metrics = PathMetrics(ns='/', k=100,real_start=real)
        metric0, metric1, metric2, metric3 = path_metrics.get_metrics_to_goal(goal)
        row['metric-0'] = metric0
        row['metric-1'] = metric1[0]
        row['metric-2'] = metric2[0]
        row['metric-3'] = metric3
        
    print("Waiting to start experiment.")
    time.sleep(10)
    start_time = time.time()
    print("sending to goal")
    client.send_goal(goalMsg)
    #if(experiment==0):
    #    try:
    #        goal_msg = rospy.wait_for_message('move_base/DWAPlannerROS/global_plan',Path, 200)
    #    except:
    #        row['calculation_time_' + ending_string] = "timeout"
            
    calculation_time = time.time()
    print("waiting to reach")
    client.wait_for_result(rospy.Duration(300)) #5 mins
    status = client.get_state()

    end_time = time.time()
    location = get_robot_location()

    row['move_base_status_finish_' + ending_string] = str(True if status==4 else False)
    row['final_location_' + ending_string] = location
    row['how_far_from_goal_' + ending_string] = np.linalg.norm(np.array(location) - np.array(goal))
    #row['did_it_really_arrived_' + ending_string] = str(True if np.linalg.norm(np.array(location) - np.array(goal)) < 0.6 else False)
    row['execute_time_' + ending_string]  = end_time - calculation_time
    row['calculation_time_' + ending_string]  = calculation_time - start_time 
            
    launch.shutdown()
    print("killing Launch")
    time.sleep(10)
    core.terminate()
    print('finished '+ ending_string)
    time.sleep(20)
    return row
    
if __name__ == '__main__':
    return running_on_map(expirement=int(sys.argv[1]),tuple(sys.argv[2]),tuple(sys.argv[3]))
# uuid=roslaunch.rlutil.get_or_generate_uuid(None, False)
# roslaunch.configure_logging(uuid)
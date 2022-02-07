#! /usr/bin/env python
import roslaunch
import rospy
import rospkg
import numpy as np
from path_metrics import *
import time
import pandas as pd
from os.path import exists
from os import mkdir
import sys
import subprocess

import yaml
from yaml import SafeLoader


import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance
from nav_msgs.msg import Odometry, Path
from tf.transformations import quaternion_from_euler


def read_pgm(address):
    """
    read the map for metric
    """
    address = address.split('.')[0]+'.pgm'
    pgmf = open(address,'rb')
    version=pgmf.readline()
    assert version == b'P5\n' or version == b'P6\n'
    comment = pgmf.readline()
    (width, height) = [int(i) for i in pgmf.readline().split()]
    depth = int(pgmf.readline())
    assert depth <= 255

    seen=set()
    raster = []
    for y in range(height):
        row = []
        for x in range(width):
            row.append(ord(pgmf.read(1)))
        seen.update(set(row))
        #print(row)
        raster.append(row)

    address = address.split('.')[0]+'.yaml'
    yaml_data = {}
    origin_data=(0,0)
    with open(address) as yaml_file:
        yaml_data = yaml.load(yaml_file,Loader=SafeLoader)
        origin_data = yaml_data['origin'][0:2]
        #print(origin_data)
    map_option={'width': width, 'height': height, 'seen': seen, 'resolution': float(yaml_data['resolution'])
        , 'origin': [float(origin_data[0]), float(origin_data[1])]}

    return raster, map_option

def running_single():
    """
    running the launch file
    """
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    
    rospack = rospkg.RosPack()
    pkg_dir = rospack.get_path('multi_robot_nav')

    launch = roslaunch.parent.ROSLaunchParent(uuid, [pkg_dir + "/launch/tests/test_robot_run.launch"])
    return launch
  
def create_test_launch(real, fake, map_path=None):
    """
    create launch for move base and amcl
    """
    #print((real,fake,map_path))
    if map_path is None:
        #world_path="$(find turtlebot3_gazebo)/worlds/turtlebot3_world.world"
        map_path="$(find multi_robot_nav)/config/map.yaml"
    rospack = rospkg.RosPack()
    pkg_dir = rospack.get_path('multi_robot_nav')

    name = pkg_dir + '/launch/tests/test_robot_run.launch'
    with open(name,'w') as file:
        file.write(
        "<launch>\n\t\
            <include file=\"$(find multi_robot_nav)/launch/tests/test_move_base_real.launch\">\n\t\
                <arg name=\"real_x\" value=\"" + str(real[0]) + "\"/>\n\t\
                <arg name=\"real_y\" value=\"" + str(real[1]) + "\"/>\n\t\
                <arg name=\"fake_x\" value=\"" + str(fake[0]) + "\"/>\n\t\
                <arg name=\"fake_y\" value=\"" + str(fake[1]) + "\"/>\n\t\
                <arg name=\"map_file\" value=\"" + str(map_path) + "\"/>\n\t\
            </include>\n\
        </launch>")

def set_real_position(position):
    """
    set the robot position
    """
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
    """
    find the robot using odometry it doesnt work good if the floor is not clean or have Embosse
    """
    odom_msg = rospy.wait_for_message('odom', Odometry)
    position = odom_msg.pose.pose.position
    return position.x,position.y

def goalTargetMaking(goal):
    """
    create move base goal class
    """
    goalTarget = MoveBaseGoal()
    goalTarget.target_pose.header.frame_id = "map"
    goalTarget.target_pose.header.stamp = rospy.Time.now()
    goalTarget.target_pose.pose.position.x = goal[0]
    goalTarget.target_pose.pose.position.y = goal[1]
    goalTarget.target_pose.pose.orientation.w = 1.0
    return goalTarget

def get_fake_point(real,base_radius=1):
    """
    create fake point with radius
    """
    radius = random.random() * base_radius
    theta = (2 * np.pi) * random.random()
    fake_x, fake_y = radius * np.cos(theta) + real[0], radius * np.sin(theta) + real[1]
    return fake_x,fake_y

def running_expirement(expirement=None,start=None,fake=None,end=None, map_path=None,map_options=None):
    """
    running and expermint and get the results and the metrics
    """
    row={}
    ending_string="without_oracle" if expirement == 1 else "oracle"
    #print((expirement,start,fake,end,map_path))
    create_test_launch(start, fake if expirement==1 else start, map_path)

    rospy.init_node('move_base_sequence')
            
    print("running launch")
    launch = running_single()
    launch.start()
    time.sleep(10)
        
    print("making goal")
       
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server(rospy.Duration(10))
        
    if expirement == 0:
        print("setting position")
        set_real_position(start)
    else:
        path_metrics = PathMetrics(ns='/', k=150,real_start=start,\
                            global_origin=map_options['origin'],resolution=map_options['resolution'])
        metric0, metric1, metric2, metric3, metric4 = path_metrics.get_metrics_to_goal(end)
        row['metric-0'] = metric0
        row['metric-1'] = metric1[0]
        row['metric-2'] = metric2[0]
        row['metric-3'] = metric3
        row['dead_reakoing_percent'] = metric4[0]
        row['dead_reakoing_mean_step'] = metric4[1]
        row['dead_reakoing_mean_dist'] = metric4[2]
        
    print("Waiting to start experiment.")
    time.sleep(10)
    start_time = time.time()
    print("sending to goal")
    client.send_goal(goalTargetMaking(end))
    #for i in range(30):
    #    print(client.get_state())
    #    time.sleep(0.5)
    try:
        goal_msg = rospy.wait_for_message('move_base/DWAPlannerROS/global_plan',Path, 20)
    except:
        row['calculation_time_' + ending_string] = "timeout"
            
    calculation_time = time.time()
    print("waiting to reach")
    client.wait_for_result(rospy.Duration(600)) #10 mins
    status = client.get_state()

    end_time = time.time()
    location = get_robot_location()

    row['move_base_status_finish_' + ending_string] = str(True if status == 3 else False)
    row['move_base_status_aborted_' + ending_string] = str(True if status == 4 else False)
    row['final_location_' + ending_string] = location
    row['how_far_from_goal_' + ending_string] = np.linalg.norm(np.array(location) - np.array(end))
    #row['did_it_really_arrived_' + ending_string] = str(True if np.linalg.norm(np.array(location) - np.array(goal)) < 0.6 else False)
    row['execute_time_' + ending_string]  = end_time - calculation_time
    row['calculation_time_' + ending_string]  = calculation_time - start_time 
            
    launch.shutdown()
    print("killing Launch - finished " + ending_string)
    time.sleep(10)
    #core.terminate()
    #time.sleep(15)
    return row

def running_on_map(world_name='turtlebot3_world',map_path=None,experiment=0,real=None,fake=None,goal=None,radius=1):
    """ 
    create expriment row, running the robot and when it finish save the results to csv.
    """
    if real is not None: real = tuple([float(x) for x in real.split(',')])
    if fake is not None: fake = tuple([float(x) for x in fake.split(',')])
    else: fake = get_fake_point(real, radius)
    if goal is not None: goal = tuple([float(x) for x in goal.split(',')])
    #print(fake)
    map_array,map_options = read_pgm(map_path)
    #print((map_options['width'],map_options['height']))
    #print(np.unique(np.array(map_array)))
    if not exists("Results/"): mkdir("Results")
    results = pd.read_csv("Results/" + world_name + '_results' + str(experiment + 1) + '.csv')\
         if exists("Results/" + world_name + '_results' + str(experiment + 1) + '.csv')\
         else pd.DataFrame()
 
    row={}
    #real, fake, goal = get_points(map_array, map_options)
        
    print((real,fake,goal))
    row['real_location'] = real
    row['fake_location'] = fake
    row['goal_location'] = goal  

    row.update(running_expirement(experiment,real,fake,goal, map_path,map_options))

    results = results.append(row, ignore_index=True)
    results.to_csv("Results/" + world_name + '_results' + str(experiment + 1) + '.csv', index=False)

if __name__ == '__main__':
    #print(sys.argv)
    if len(sys.argv)<5:
        print("need all the data points, give me in that order: experiment(0/1 = orcale/fake location), map_location, real point (without spaces for example 4.5,3 ), goal point (like the real point), and radius of fake (optinal)")
    else:
        running_on_map(experiment=int(sys.argv[1]),world_name="real_robot",\
            map_path=sys.argv[2],real=sys.argv[3],goal=sys.argv[4],radius=float(sys.argv[5]))
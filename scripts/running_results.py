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

def read_pgm(address):
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

def point_clear(map_array,map_options):
    #pixel world
    half_width = (map_options['width'] / 2)
    half_height = (map_options['height'] / 2)
    center = (half_width, half_height)
    radius = 2
    found_one_black = True
    #print((rnd_x,rnd_y))
    while found_one_black:
        found_one_black = False
        rnd_x = random.randint(-half_width+radius, half_width - radius)
        rnd_y = random.randint(-half_height+radius, half_height - radius)
        randomPicked=(int(center[0]+rnd_x),int(center[1]+rnd_y))
        if map_array[randomPicked[1]][randomPicked[0]] <= 208: #min(map_options['seen']): #clear and seen
            found_one_black = True
            continue

        for i in range(-radius, radius):
            for j in range(-radius, radius):
                if map_array[int(randomPicked[1]+i)][int(randomPicked[0]+j)]  <100:#==  min(map_options['seen']):
                    found_one_black = True

    #print(randomPicked)
    #print(map_array[randomPicked[0]][randomPicked[1]])
    return randomPicked[0] * map_options['resolution'] + map_options['origin'][0],\
           randomPicked[1] * map_options['resolution'] + map_options['origin'][1]

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

def get_points(map_array, map_options):
    points_off_map = False
    while not points_off_map:
        real_x, real_y = point_clear(map_array, map_options)
        goal_x, goal_y = point_clear(map_array, map_options)
        radius = random.random() + 1
        theta = (2 * np.pi) * random.random()
        fake_x, fake_y = radius * np.cos(theta) + real_x, radius * np.sin(theta) + real_y
        
        fake_x_in = map_options['origin'][0] < fake_x < (map_options['origin'][0] + map_options['width'] * map_options['resolution'])
        fake_y_in = map_options['origin'][1] < fake_y < (map_options['origin'][1] + map_options['height'] * map_options['resolution'])
        real_x_in = map_options['origin'][0] < real_x < (map_options['origin'][0] + map_options['width'] * map_options['resolution'])
        real_y_in = map_options['origin'][1] < real_y < (map_options['origin'][1] + map_options['height'] * map_options['resolution'])
        goal_x_in = map_options['origin'][0] < goal_x < (map_options['origin'][0] + map_options['width'] * map_options['resolution'])
        goal_y_in = map_options['origin'][1] < goal_y < (map_options['origin'][1] + map_options['height'] * map_options['resolution'])

        points_off_map = fake_x_in and fake_y_in and real_x_in and real_y_in and goal_x_in and goal_y_in

    return (real_x, real_y), (fake_x, fake_y), (goal_x, goal_y)

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



def running_expirement(expirement=None,start=None,fake=None,end=None, world_path=None, map_path=None):
        
    row={}
    create_test_launch(start, fake if expirement==1 else start, world_path, map_path)

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
       
    goalMsg=goalTargetMaking(end)
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server(rospy.Duration(10))
        
    if expirement == 0:
        print("setting position")
    else:
        path_metrics = PathMetrics(ns='/', k=150,real_start=start)
        metric0, metric1, metric2, metric3 = path_metrics.get_metrics_to_goal(end)
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
    row['how_far_from_goal_' + ending_string] = np.linalg.norm(np.array(location) - np.array(end))
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

def running_on_map(world_name='turtlebot3_world',world_path=None,map_path=None):

    rospack = rospkg.RosPack()
    pkg_dir = rospack.get_path('multi_robot_nav')
    if map_path is None:
        map_array,map_options = read_pgm(pkg_dir + "/maps/map.pgm")
    else:
        map_array,map_options = read_pgm(map_path)
    print((map_options['width'],map_options['height']))
    print(np.unique(np.array(map_array)))
    results = pd.read_csv(world_name + '_results.csv') if exists(world_name+'_results.csv') else pd.DataFrame()
 
    row={}
    real, fake, goal = get_points(map_array, map_options)
        
    print((real,fake,goal))
    row['real_location'] = real
    row['fake_location'] = fake
    row['goal_location'] = goal  


    for i in range(2):
        row.update(running_expirement(i,real,fake,goal, world_path, map_path))
    results = results.append(row, ignore_index=True)
    results.to_csv(world_name + '_results.csv', index=False)

if __name__ == '__main__':
    if len(sys.argv)<2:
        running_on_map()
    else:
        running_on_map(sys.argv[1],sys.argv[2],sys.argv[3])
# create_test_launch((1.5,3),(2.1,4))
# uuid=roslaunch.rlutil.get_or_generate_uuid(None, False)
# roslaunch.configure_logging(uuid)
# running_single(uuid)
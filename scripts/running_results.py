#! /usr/bin/env python
import roslaunch
import rospy
import rospkg
import numpy as np
from path_metrics import *
import random

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance
from tf.transformations import quaternion_from_euler

def read_pgm(address):
    pgmf = open(address,'rb')
    assert pgmf.readline() == 'P5\n'
    comment=pgmf.readline()
    (width, height) = [int(i) for i in pgmf.readline().split()]
    depth = int(pgmf.readline())
    assert depth <= 255

    raster = []
    for y in range(height):
        row = []
        for y in range(width):
            row.append(ord(pgmf.read(1)))
        raster.append(row)
    return raster,width,height

def running_single():
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    
    rospack = rospkg.RosPack()
    pkg_dir = rospack.get_path('multi_robot_nav')

    launch = roslaunch.parent.ROSLaunchParent(uuid, [pkg_dir + "/launch/tests/test_runner.launch"])
    launch.start()
    return launch
    # cli_args = [pkg_dir + '/launch/localization_world.launch',\
    # 'fake_x_1:='+str(fake_1[0]),'fake_y_1:='+str(fake_1[1]), 'real_x_1:='+str(real_1[0]),'real_y_1:='+str(real_1[1]),\
    # 'fake_x_2:='+str(fake_2[0]),'fake_y_2:='+str(fake_2[1]), 'real_x_2:='+str(real_2[0]),'real_y_2:='+str(real_2[1]),]

    # roslaunch_args = cli_args[1:]
    # # /home/user/multiNav_ws/src/multi_robot_nav/launch/localization_world.launch
    # # 'multi_robot_nav','localization_world.launch'

    # roslaunch_args = cli_args[1:]
    # class tupleList(tuple):
    #     def __init__(self,launch):
    #         #print("%s"%self)
    #         #print("%s"%tuple(launch))
    #         self.launch=launch
    #         tuple.__init__(launch)
    
    #     def __format__(self,spec):
    #         print("here i am")
    #         return ""
    #     def __repr__(self):
    #         print("hereeee")
    #         return ""
    #     def __str__(self):
    #         print("I being called!")
    #         return str(self.launch[0])
    #         #return str(self.launch)

    # tupleItem=[roslaunch.rlutil.resolve_launch_arguments(cli_args)[0],roslaunch_args]
    # #print(tupleItem)
    # print("%s"%tupleItem)
    #launch = roslaunch.parent.ROSLaunchParent(uuid,[tupleItem])
    #launch = roslaunch.parent.ROSLaunchParent(uuid, ["pkg_dir + /launch/localization_world.launch"])
    

    #    rospy.loginfo("started")

    #    rospy.sleep(30)
    # 3 seconds later
    # launch.shutdown()

  
def create_test_launch(real,fake):
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
            </include>\n\
        </launch>")

def point_clear(map_array,width,height):
    rnd_x = random.randint(0,width-1)
    rnd_y = random.randint(0,height-1)
    #print((rnd_x,rnd_y))
    while map_array[rnd_x][rnd_y] != 254:
        rnd_x = random.randint(0,width-1)
        rnd_y = random.randint(0,height-1)
    #    print((rnd_x,rnd_y))
    return rnd_x*0.05-10,rnd_y*0.05-10

def get_points(map_array,width,height):
    real_x,real_y=point_clear(map_array,width,height)
    goal_x,goal_y=point_clear(map_array,width,height)
    radius = random.random()*2+1
    theta = (2*np.pi)*random.random()
    fake_x, fake_y = radius * np.cos(theta) + real_x, radius * np.sin(theta) + real_y
    return (real_x,real_y), (fake_x,fake_y), (goal_x,goal_y)

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

def goalTargetMaking(goal):
    goalTarget = MoveBaseGoal()
    goalTarget.target_pose.header.frame_id = "map"
    goalTarget.target_pose.header.stamp = rospy.Time.now()
    goalTarget.target_pose.pose.position.x = goal[0]
    goalTarget.target_pose.pose.position.y = goal[1]
    goalTarget.target_pose.pose.orientation.w = 1.0
    return goalTarget

def running_on_map():
    rospy.init_node('move_base_sequence')

    rospack = rospkg.RosPack()
    pkg_dir = rospack.get_path('multi_robot_nav')

    map_array,width,height=read_pgm(pkg_dir + "/maps/map.pgm")
    print((width,height))
    print(np.unique(np.array(map_array)))


    for i in range(2):
        
        real,fake,goal=get_points(map_array,width,height)
        
        print((real,fake,goal))
        
        create_test_launch(real,fake)

        
        for experiment in range(2):
            
            print("running launch")
            launch=running_single()
            print("making goal")
            goalTarget = goalTargetMaking(goal)

            client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
            client.wait_for_server(rospy.Duration(5))
            if experiment == 1:
                print("setting position")
                set_real_position(goal)
            else:
                path_metrics = PathMetrics(ns='/', k=100)
                print(path_metrics.get_metrics_to_goal(goal))
            print("sending to goal")
            client.send_goal(goalTarget)
            print("waiting to reach")
            client.wait_for_result(rospy.Duration(150)) #2.5 mins
            launch.shutdown()



running_on_map()
# create_test_launch((1.5,3),(2.1,4))
# uuid=roslaunch.rlutil.get_or_generate_uuid(None, False)
# roslaunch.configure_logging(uuid)
# running_single(uuid)
#! /usr/bin/env python
import roslaunch
import rospy
import rospkg
import numpy as np
from path_metrics import *
import time
import pandas as pd
from os.path import exists
import os
import sys
import subprocess

import threading
import Queue

import yaml
from yaml import SafeLoader


import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, PoseArray
from nav_msgs.msg import Odometry, Path
from tf.transformations import quaternion_from_euler
from nav_msgs.srv import GetMap, GetPlan


def read_pgm(address):
    if '..' in address:
        address = address[0:address.index(
            'scripts')]+address[address.index('..')+3:]
    address = address.split('.')[0]+'.pgm'
    pgmf = open(address, 'rb')
    version = pgmf.readline()
    assert version == b'P5\n' or version == b'P6\n'
    comment = pgmf.readline()
    (width, height) = [int(i) for i in pgmf.readline().split()]
    depth = int(pgmf.readline())
    assert depth <= 255

    seen = set()
    raster = []
    for y in range(height):
        row = []
        for x in range(width):
            row.append(ord(pgmf.read(1)))
        seen.update(set(row))
        # print(row)
        raster.append(row)

    address = address.split('.')[0]+'.yaml'
    yaml_data = {}
    origin_data = (0, 0)
    with open(address) as yaml_file:
        yaml_data = yaml.load(yaml_file, Loader=SafeLoader)
        origin_data = yaml_data['origin'][0:2]
        # print(origin_data)
    map_option = {'width': width, 'height': height, 'seen': seen, 'resolution': float(
        yaml_data['resolution']), 'origin': [float(origin_data[0]), float(origin_data[1])]}

    return raster, map_option


def running_single():
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    rospack = rospkg.RosPack()
    pkg_dir = rospack.get_path('multi_robot_nav')

    launch = roslaunch.parent.ROSLaunchParent(
        uuid, [pkg_dir + "/launch/tests/test_runner.launch"])
    return launch


def create_test_launch(real, fake, world_path=None, map_path=None):

    if world_path is None:
        world_path = "$(find turtlebot3_gazebo)/worlds/turtlebot3_world.world"
        map_path = "$(find multi_robot_nav)/config/map.yaml"
    rospack = rospkg.RosPack()
    pkg_dir = rospack.get_path('multi_robot_nav')

    name = pkg_dir + '/launch/tests/test_runner.launch'
    with open(name, 'w') as file:
        file.write(
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
    pub = rospy.Publisher(
        'initialpose', PoseWithCovarianceStamped, queue_size=1)
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
    return position.x, position.y


def get_plan(start_tuple, end_tuple):

    startPoseStamp = PoseStamped()
    startPoseStamp.header.seq = 0
    startPoseStamp.header.frame_id = '/map'
    startPoseStamp.header.stamp = rospy.Time(0)
    startPoseStamp.pose.position.x = start_tuple[0]
    startPoseStamp.pose.position.y = start_tuple[1]

    endPoseStamp = PoseStamped()
    endPoseStamp.header.seq = 0
    endPoseStamp.header.frame_id = '/map'
    endPoseStamp.header.stamp = rospy.Time(0)
    endPoseStamp.pose.position.x = end_tuple[0]
    endPoseStamp.pose.position.y = end_tuple[1]

    get_plan_srv = '/move_base/make_plan'
    get_plan = rospy.ServiceProxy(get_plan_srv, GetPlan)

    req = GetPlan()
    req.start = startPoseStamp
    req.goal = endPoseStamp
    req.tolerance = .5
    resp = get_plan(req.start, req.goal, req.tolerance)
    # print(resp)
    return resp.plan.poses


def goalTargetMaking(goal):
    goalTarget = MoveBaseGoal()
    goalTarget.target_pose.header.frame_id = "map"
    goalTarget.target_pose.header.stamp = rospy.Time.now()
    goalTarget.target_pose.pose.position.x = goal[0]
    goalTarget.target_pose.pose.position.y = goal[1]
    goalTarget.target_pose.pose.orientation.w = 1.0
    return goalTarget


def get_location_of_particle_system():
    pf_topic = 'particlecloud'
    msg = rospy.wait_for_message(pf_topic, PoseArray)
    possible_poses_array = msg.poses

    array_pose = [(item.position.x, item.position.y)
                  for item in possible_poses_array]
    array_sample = random.sample(array_pose, 20)
    return array_sample


def from_path_str_to_position_list(text):
    try_text = text[1:-1]
    try_text = try_text.split("position:")[1:]
    for i in range(len(try_text)):
        try_text[i] = try_text[i].split('orientation:')[0]
        try_text[i] = (float(try_text[i].split('x:')[1].split('y:')[0]), float(
            try_text[i].split('y:')[1].split('z:')[0]))
    return try_text


def running_expirement(expirement=None, start=None, fake=None, end=None, world_name=None, world_path=None, map_path=None, map_options=None, dead_rekoing=False):

    row = {}
    ending_string = "without_oracle" if expirement == 1 else "oracle"
    create_test_launch(start, fake if expirement ==
                       1 else start, world_path, map_path)

    core = subprocess.Popen(['roscore'])
    print("starting Core")
    time.sleep(10)

    rospy.init_node('move_base_sequence')

    print("running launch")
    launch = running_single()
    launch.start()
    time.sleep(5)

    print("making goal")

    counter_replan = [None]

    def thread_counting_replan(counter_replan):
        counter = 0
        while True:
            try:
                rospy.wait_for_message(
                    '/move_base/DWAPlannerROS/global_plan', Path, 15)
                print("got replan")
                counter += .5
            except:
                break
        counter_replan[0] = counter
        return counter

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server(rospy.Duration(5))
    que = Queue.Queue()
    counting_thread = threading.Thread(target=lambda q, arg1: q.put(
        thread_counting_replan(arg1)), args=(que, counter_replan))

    row = {'real_location': start, 'goal_location': end}

    if expirement == 0:
        print("setting position")
        set_real_position(start)
    else:
        path_metrics = PathMetrics(ns='/', k=150, real_start=start,
                                   global_origin=map_options['origin'], resolution=map_options['resolution'])
        metric0, metric1, metric2, metric3, metric4 = path_metrics.get_metrics_to_goal(
            end)
        row['metric-0'] = metric0
        row['metric-1'] = metric1[0]
        row['metric-2'] = metric2[0]
        row['metric-3'] = metric3
        row['dead_reakoing_percent'] = metric4[0]
        row['dead_reakoing_mean_step'] = metric4[1]
        row['dead_reakoing_mean_dist'] = metric4[2]
        if dead_rekoing:
            launch.shutdown()
            print("killing Launch")
            time.sleep(10)
            core.terminate()
            print('finished ' + ending_string)
            time.sleep(15)
            return row

    started_pose = rospy.wait_for_message(
        'amcl_pose', PoseWithCovarianceStamped)

    cov_matrix = np.array(started_pose.pose.covariance).reshape(6, 6)
    print(cov_matrix)
    row['init_covariance'] = [cov_matrix[0][0], cov_matrix[0][1],
                              cov_matrix[1][0], cov_matrix[1][1], cov_matrix[5][5]]
    row['started_plan'] = from_path_str_to_position_list(
        str(get_plan(start if expirement == 0 else fake, end)).replace('\n', ' '))
    print("Waiting to start experiment.")
    counting_thread.start()

    time.sleep(2)
    start_time = time.time()
    print("sending to goal")
    client.send_goal(goalTargetMaking(end))
    # for i in range(30):
    #    print(client.get_state())
    #    time.sleep(0.5)
    try:
        goal_msg = rospy.wait_for_message(
            'move_base/DWAPlannerROS/global_plan', Path, 20)
    except:
        row['calculation_time_' + ending_string] = "timeout"

    calculation_time = time.time()
    print("waiting to reach")
    client.wait_for_result(rospy.Duration(600))  # 10 mins
    status = client.get_state()

    end_time = time.time()
    location = get_robot_location()

    row['move_base_status_finish_' +
        ending_string] = str(True if status == 3 else False)
    row['move_base_status_aborted_' +
        ending_string] = str(True if status == 4 else False)
    row['final_location_' + ending_string] = location
    row['how_far_from_goal_' +
        ending_string] = np.linalg.norm(np.array(location) - np.array(end))
    #row['did_it_really_arrived_' + ending_string] = str(True if np.linalg.norm(np.array(location) - np.array(goal)) < 0.6 else False)
    row['execute_time_' + ending_string] = end_time - calculation_time
    row['calculation_time_' + ending_string] = calculation_time - start_time
    finish_pose = rospy.wait_for_message(
        'amcl_pose', PoseWithCovarianceStamped)
    cov_matrix = np.array(finish_pose.pose.covariance).reshape(6, 6)
    print(cov_matrix)
    row['finish_covariance' + ending_string] = [cov_matrix[0][0],
                                                cov_matrix[0][1], cov_matrix[1][0], cov_matrix[1][1], cov_matrix[5][5]]
    counter_replan = que.get()
    print(counter_replan)
    row['number_of_replans'] = counter_replan

    launch.shutdown()
    print("killing Launch")
    time.sleep(10)
    core.terminate()
    print('finished ' + ending_string)
    time.sleep(15)
    return row


def running_on_map(world_name='turtlebot3_world', world_path=None, map_path=None, experiment=0, real=None, fake=None, goal=None, dead_rekoing=False):

    whereIAm = os.getcwd()
    if not exists(map_path) and exists(whereIAm+'/../'+map_path):
        map_path = whereIAm+'/../'+map_path
    if not exists(world_path) and exists(whereIAm+'/../'+world_path):
        world_path = whereIAm+'/../'+world_path

    real = tuple([float(x) for x in real[1:-1].split(',')])
    fake = tuple([float(x) for x in fake[1:-1].split(',')])
    goal = tuple([float(x) for x in goal[1:-1].split(',')])

    map_array, map_options = read_pgm(map_path)

    results = pd.read_csv("Results/" + world_name + '_newMetrics_' + str(experiment + 1) + '.csv')\
        if exists("Results/" + world_name + '_newMetrics_' + str(experiment + 1) + '.csv')\
        else pd.DataFrame()

    row = {}

    print((real, fake, goal))
    row['real_location'] = real
    row['fake_location'] = fake
    row['goal_location'] = goal

    row_expirement = running_expirement(
        experiment, real, fake, goal, world_name, world_path, map_path, map_options, dead_rekoing)
    row.update(row_expirement)

    results = results.append(row, ignore_index=True)
    results.to_csv("Results/" + world_name + '_newMetrics_' +
                   str(experiment + 1) + '.csv', index=False)


if __name__ == '__main__':
    if len(sys.argv) < 5:
        print()
    else:
        running_on_map(experiment=int(sys.argv[1]), world_name=sys.argv[2], world_path=sys.argv[3],
                       map_path=sys.argv[4], real=sys.argv[5], fake=sys.argv[6], goal=sys.argv[7], dead_rekoing=False)

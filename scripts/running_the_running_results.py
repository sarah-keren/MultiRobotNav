#! /usr/bin/env python
import subprocess
import time
import sys

world_dict = {
    #'corridor' : ('$(find chirs-worlds)/worlds/corridor.world','$(find chirs-worlds)/worlds/corridor.world'),
    'creech': ("/home/user/multiNav_ws/src/chris-worlds/worlds/creech_map.world","/home/user/multiNav_ws/src/chris-worlds/maps/creech_map.world"),
    'maze': ("/home/user/multiNav_ws/src/chris-worlds/worlds/maze.world", "/home/user/multiNav_ws/src/chris-worlds/maps/chris_maze.yaml"),
    'willow': ("worlds/willowgarage.world", "/home/user/multiNav_ws/src/chris-worlds/maps/willow_map.yaml"),
    'lab': ("/home/user/multiNav_ws/src/chris-worlds/worlds/lab_world_050.world", "/home/user/multiNav_ws/src/chris-worlds/maps/lab_world_050.yaml"),
    'house': ("/home/user/multiNav_ws/src/multi_robot_nav/worlds/turtlebot3_house.world", "/home/user/multiNav_ws/src/multi_robot_nav/maps/house_map.yaml")}
def running():
    if len(sys.argv)>1:
        name=sys.argv[1]
        if name not in world_dict:
            name = 'creech'
        print("running on map: "+name)
        time.sleep(2)
    else: name='creech'
    for i in range(20):
        print("running the running number: "+str(i))
        
        p = subprocess.Popen(['python', 'running_results.py',name,world_dict[name][0],world_dict[name][1]])
        time.sleep(200*2)
        p.terminate()
       
running()

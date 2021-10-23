#! /usr/bin/env python
import subprocess
import time
import sys
import pandas as pd
from pandas import DataFrame
import yaml
from yaml import SafeLoader
import random
import numpy as np
import os


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



world_dict = {
    #'corridor' : ('$(find chirs-worlds)/worlds/corridor.world','$(find chirs-worlds)/worlds/corridor.world'),
    'creech': ("/home/user/multiNav_ws/src/chris-worlds/worlds/creech_map.world","/home/user/multiNav_ws/src/chris-worlds/maps/creech_map.world"),
    'maze': ("/home/user/multiNav_ws/src/chris-worlds/worlds/maze.world", "/home/user/multiNav_ws/src/chris-worlds/maps/chris_maze.yaml"),
    'office': ("worlds/willowgarage.world", "/home/user/multiNav_ws/src/chris-worlds/maps/willow_map.yaml"),
    'lab': ("/home/user/multiNav_ws/src/chris-worlds/worlds/lab_world_050.world", "/home/user/multiNav_ws/src/chris-worlds/maps/lab_world_050.yaml"),
    'house': ("/home/user/multiNav_ws/src/multi_robot_nav/worlds/turtlebot3_house.world", "/home/user/multiNav_ws/src/multi_robot_nav/maps/house_map.yaml")}

def create_dataPoints():
    dataPoints = DataFrame()
    for i in range(100):
        real, fake, goal = get_points(map_array, map_options)
        dataPoints=dataPoints.append({'real':real,'fake':fake,'goal':goal},ignore_index=True)
    dataPoints.to_csv('Results/'+name+'_data_points.csv',index=False)

def update_picked(name,experiment):
    dataPoints = pd.read_csv('Results/'+name+'_data_points.csv')
    dataPoints['picked'] = False
    if not os.path.exists("Results/" + name + '_results' + str(experiment + 1) + '.csv'): return dataPoints
    results = pd.read_csv("Results/" + name + '_results' + str(experiment + 1) + '.csv')
    found=0
    for index,row in results.iterrows():
        for index2,row2 in dataPoints.iterrows():
            if row2['real'] == row['real_location'] and row2['goal'] == row['goal_location']:
                dataPoints.loc[index2,'picked']=True
                found += 1
    dataPoints.to_csv('Results/'+name+'_data_points.csv',index=False)
    print(str(found)+" updated")
    return dataPoints
    
def running():
    if len(sys.argv)>1:
        name=sys.argv[1]
        if name not in world_dict:
            name = 'creech'
        print("running on map: "+name)
        time.sleep(2)
    else: name='creech'
    experiment=0
    data = pd.read_csv('Results/'+name+'_data_points.csv')
    map_array,map_options = read_pgm(world_dict[name][1])
    data=update_picked(name,experiment)
    for index, row in data.sample(100).iterrows():
        print("running the running number: "+str(index))
        if 'picked' in data.columns and row['picked']: continue
        print(str(row['real']),str(row['fake']),str(row['goal']))
        p = subprocess.Popen(['python', 'running_results.py',str(experiment),name,world_dict[name][0],world_dict[name][1],str(row['real']),str(row['fake']),str(row['goal'])])
        p.wait()
        #p.terminate()
        print("killing core if there")
        killcore=subprocess.Popen(['killall', '-9' ,'rosmaster'])
        time.sleep(10)
        data.loc[index,'picked']=True
        data.to_csv('Results/'+name+'_data_points.csv',index=False)
running()

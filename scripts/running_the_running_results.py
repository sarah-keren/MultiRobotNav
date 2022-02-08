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

#relative to this code
world_dict = {
    #'corridor' : ('$(find chirs-worlds)/worlds/corridor.world','$(find chirs-worlds)/worlds/corridor.world'),
    #'creech': ("multi_robot_nav/chris_world/creech_map.world","chris_maps/creech_map.world"),
    'maze': ("../chris_world/maze.world", "../chris_maps/chris_maze.yaml"),
    'office': ("worlds/willowgarage.world", "../chris_maps/Office.yaml"),
    'lab': ("../chris_world/lab_world_complete.world", "../maps/lab_submaps/lab_world_complete.yaml"),
    'lab_boxLess': ("../maps/lab_submaps/lab_world_boxesLess.world", "../maps/lab_submaps/lab_world_boxesLess.yaml"),
    'lab_frontLess': ("../maps/lab_submaps/lab_world_frontLess.world", "../maps/lab_submaps/lab_world_frontLess.yaml"),
    'lab_sideLess': ("../maps/lab_submaps/lab_world_sideLess.world", "../maps/lab_submaps/lab_world_sideLess.yaml"),
    'lab_side2Less': ("../maps/lab_submaps/lab_world_side2Less.world", "../maps/lab_submaps/lab_world_side2Less.yaml"),
    'lab_wallLess': ("../maps/lab_submaps/lab_world_wallLess.world", "../maps/lab_submaps/lab_world_wallLess.yaml"),
    'lab_partBoxes': ("../maps/lab_submaps/lab_world_part_boxes.world", "../maps/lab_submaps/lab_world_part_boxes.yaml"),
    'lab_part2Boxes': ("../maps/lab_submaps/lab_world_part2_boxes.world", "../maps/lab_submaps/lab_world_part2_boxes.yaml"),
    'house': ("../worlds/turtlebot3_house.world", "../maps/house_map.yaml")}

def create_dataPoints(name,map_array,map_options):
    if 'lab' in name: return
    if os.path.exists('Results/' + name + '_data_points.csv'): return
    dataPoints = DataFrame()
    for i in range(100):
        real, fake, goal = get_points(map_array, map_options)
        dataPoints=dataPoints.append({'real':real,'fake':fake,'goal':goal},ignore_index=True)
    dataPoints.to_csv('Results/' + name + '_data_points.csv',index=False)
    print("finish creating new data points")

def from_path_str_to_position_list(text):
    try_text=text[1:-1]
    try_text=try_text.split("position:")[1:]
    for i in range(len(try_text)):
        try_text[i] = try_text[i].split('orientation:')[0]
        try_text[i] = (float(try_text[i].split('x:')[1].split('y:')[0]), float(try_text[i].split('y:')[1].split('z:')[0]))
    return try_text

def update_picked(dataPoints,name,experiment):
    #for i in range(10):
    dataPoints['picked'] = False
    if not os.path.exists("Results/" + name + '_newMetrics_'+str(experiment+1)+'.csv'): return dataPoints
    results = pd.read_csv("Results/" + name + '_newMetrics_'+str(experiment+1)+'.csv')
    found=0
    print(len(results))
    for index,row in results.iterrows():
        #if 'started_plan' in row.columns and type(row['started_plan'])==str and 'position' in row['started_plan']:
        #    results.iloc[index,'started_plan'] = from_path_str_to_position_list(row['started_plan'])
        for index2,row2 in dataPoints.iterrows():
        #    if row2['real_location'] == row['real_location'] and row2['goal_location'] == row['goal_location']:
        #        for i in range(10):
                    if row2['fake'] == row['fake_location']:
                        dataPoints.loc[index2,'picked'] = True
                        found += 1
    #results.to_csv("Results/" + name + '_newMetrics_'+str(experiment+1)+'.csv',index=False)
    dataPoints.to_csv('Results/' + name + '_data_points.csv',index=False)
    print(str(found)+" updated")
    return dataPoints
    
def running():
    if len(sys.argv)>1:
        name=sys.argv[1]
        if name not in world_dict:
            name = 'lab'
        print("running on map: "+name)
        time.sleep(2)
    else: name='lab'
    if len(sys.argv)>=3: experiment=int(sys.argv[2])
    else: experiment=1
    #map_array, map_options = read_pgm(os.getcwd()+'/'+world_dict[name][1])
    #create_dataPoints(name,map_array,map_options)

    data = pd.read_csv('Results/'+name+'_data_points.csv')
    
    data = update_picked(data, name, experiment)
    #for index, row in data.sample(100).iterrows():
    for index, row in data.sample(100).iterrows():
            real,goal,fake=row['real'],row['goal'],row['fake']
            print((real,goal,fake))
            print("running the running number: "+str(index))
            if 'picked' in data.columns and row['picked']: continue

            p = subprocess.Popen(['python', 'running_results.py',str(experiment),name,\
                (os.getcwd()+'/' if name!='office' else '') + world_dict[name][0],\
                os.getcwd()+'/'+world_dict[name][1],str(real),str(fake),str(goal)])
            p.wait()
            #p.terminate()
            print("killing core if there")
            subprocess.Popen(['killall', '-9' ,'rosmaster']) #kill the roscore if exists
            time.sleep(4)
            data.loc[index,'picked']=True
            data.to_csv('Results/'+name+'_data_points.csv',index=False)
running()

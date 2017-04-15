#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 10 22:03:47 2017

@author: uri7910

"""

import numpy as np
import svg
import time
import env
from PedestrianClass import Pedestrian
from euclid import Circle, Point2, Vector2, LineSegment2
from IPython.display import clear_output, display, HTML

crowd = []
xsize = 8
ysize = 8
amp = 80
offset = 30
ped_ra = 13
ped_buffersize = 0.5
ped_spdlimit = 1
ped_outcount = 0
ped_outflow = 0
walls = [(Point2(0,0), Point2(xsize, 0)), 
         (Point2(0,0), Point2(0, ysize)), 
         (Point2(xsize/2, ysize/2), Point2(xsize, ysize/2)), 
         (Point2(xsize/2, ysize/2), Point2(xsize/2, ysize))
        ]

pos_rx = 0.5
pos_ry = 2.5


num_flowA = 180
num_flowB = 120

ped_outcount = 0
ped_outflow = 0

state_pos_x = np.zeros([num_pedestrains, 1])
state_pos_y = np.zeros([num_pedestrains, 1])
state_vel_x = np.zeros([num_pedestrains, 1])
state_vel_y = np.zeros([num_pedestrains, 1])

from env import num_pedestrains
import math

flowA_init_wide = 3.6
flowA_x_start = 0
flowA_y_start = 4.2

flowB_init_wide = 3.6
flowB_x_start = 0
flowB_y_start = 0

radius_hh = 0.2

dest_x = 8
numblue = 0
numred = 0

init_flowA_pos_list = []
flowA_x_index = 0
num_ped_each_row = int(math.floor(flowA_init_wide/radius_hh))
for flowA_index in range(num_flowA):
    flowA_y_index = 1 + flowA_index % num_ped_each_row
    if flowA_index % num_ped_each_row ==0:
        flowA_x_index += 1

    init_pos_x = flowA_x_start-radius_hh*flowA_x_index 
    init_pos_y = flowA_y_start+radius_hh*flowA_y_index
    init_flowA_pos_list.append(np.array( [init_pos_x, init_pos_y] ) )
    ped = Pedestrian(np.array([init_pos_x]), np.array([init_pos_y]), c='red')
    crowd.append(ped)
    
    
init_flowB_pos_list = []
flowB_y_index = 0
num_ped_each_row = int(math.floor(flowB_init_wide/radius_hh))
for flowB_index in range(num_flowB):
    flowB_x_index = 1 + flowB_index % num_ped_each_row
    if flowB_index % num_ped_each_row ==0:
        flowB_y_index += 1
        
    init_pos_x = flowB_x_start+radius_hh*flowB_x_index
    init_pos_y = flowB_y_start-radius_hh*flowB_y_index    
    init_flowB_pos_list.append(np.array( [init_pos_x, init_pos_y] ))    
    ped = Pedestrian(np.array([init_pos_x]), np.array([init_pos_y]), c='blue')
    crowd.append(ped)    

   
    
    
def add_ped(num1, num2):
    for i in range(num1):
        rand = np.random.random([3])
        posX = rand[0] * 3 +1
        posY = -rand[1] * ped_buffersize
        #VelX = rand[2]
        ped = Pedestrian(np.array([posX]), np.array([posY]), c='blue')
        crowd.append(ped)
        
    for i in range(num2):
        rand = np.random.random([3])
        posX = -rand[0] * ped_buffersize
        posY = rand[1] * 2 + 5
        ped = Pedestrian(np.array([posX]), np.array([posY]), c='red')
        crowd.append(ped)

        

for human_index in range(num_pedestrains):
    if state_pos_x[human_index] > dest_x:
        ped.reset(human_index)

        
        
        
to_remove = []

for ped in crowd:
    if ped.x > xsize:
        to_remove.append(ped)
        if ped.c == 'blue':
            numblue += 1
        else:
            numred += 1
ped_outcount += numblue + numred
if step % 10 ==0:
    ped_outflow = ped_outcount
    ped_outcount = 0

if to_remove is not None:
    for ped in to_remove:
        crowd.remove(ped)
    add_ped(numblue, numred)
        
posX=[]
posY=[]
velX=[]
velY=[]

for ped in crowd:
    posX.append(ped.x)
    posY.append(ped.y)
    # speed limit
    spd = np.sqrt(ped.vx**2 + ped.vy**2)
    if spd > ped_spdlimit:
        ped.vx = (ped_spdlimit/spd)*ped.vx
        ped.vy = (ped_spdlimit/spd)*ped.vy
    velX.append(ped.vx)
    velY.append(ped.vy)
pos_x = np.reshape(posX, [-1,1])
pos_y = np.reshape(posY, [-1,1])
vel_x = np.reshape(velX, [-1,1])
vel_y = np.reshape(velY, [-1,1])

#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Apr  8 11:14:18 2017

environment setup

@author: uri7910
"""
import numpy as np

interact_region = 1.5  #when distance > interact_region, no social force
num_pedestrains = 300
#parameters of human-human interaction force
A_hhi = 2.6
B_hhi = 0.31
k_body_hhi = 20
radius_hh = 0.4 #radius of two humans
#parameters of human-wall interaction force
A_hwi = 15
B_hwi = 0.1
radius_h = 0.5 * radius_hh #radius of one human
k_body_hwi = 20
#parameters of human-robot interaction force
A_hri = 10
B_hri = 1
radius_r = 0.5 #radius of robot
radius_hr = radius_h + radius_r
#parameters of self-driven force
accel_time = 0.5
desired_speed = 2 + 0.3 * np.random.randn(num_pedestrains, 1)


def human_human_force(pos_x, pos_y):
    """Calculate the interaction force between human and human.

    Note: make sure the shape of pos_x and pos_y is N x 1 not N.

    Args:
      pos_x: A vector(N x 1) which represents the postion of human in x axis.
      pos_y: A vector(N x 1) which represents the postion of human in y axis.
      
    Return:
      f_hhi_x: An array(N x N) which represents the human-human interaction force in x axis. 
               For example, the element in the ith colum and jth row represents force pointing from human j to human i.
      f_hhi_y: An array(N x N) which represents the human-human interaction force in y axis. 

    """    
    pos_x_tile_col = np.tile(pos_x, (1, np.shape(pos_x)[0]))
    pos_x_tile_row = np.transpose(pos_x_tile_col)
    pos_y_tile_col = np.tile(pos_y, (1, np.shape(pos_y)[0]))
    pos_y_tile_row = np.transpose(pos_y_tile_col)
    distance_x = pos_x_tile_row - pos_x_tile_col
    distance_y = pos_y_tile_row - pos_y_tile_col
    distance = np.sqrt(np.square(distance_x) + np.square(distance_y))
    distance_add_idetity = distance + np.identity(np.shape(distance)[0]) #add identity to avoid divide by 0
    distance_x_norm = np.divide(distance_x, distance_add_idetity)
    distance_y_norm = np.divide(distance_y, distance_add_idetity)    
    
    f_interact = np.multiply( np.exp( np.divide((radius_hh - distance), B_hhi) ), A_hhi )
    f_body = np.multiply( np.maximum( (radius_hh - distance), np.zeros( np.shape(distance) ) ), k_body_hhi )
    f_hhi = f_interact + f_body    
    f_hhi_x = np.multiply( f_hhi, distance_x_norm ) 
    f_hhi_y = np.multiply( f_hhi, distance_y_norm )
    
    return f_hhi_x, f_hhi_y

def human_wall_force(pos_x, pos_y):
    """Calculate the interaction force between human and wall.

    Note: make sure the shape of pos_x and pos_y is N x 1 not N.

    Args:
      pos_x: A vector(N x 1) which represents the postion of human in x axis.
      pos_y: A vector(N x 1) which represents the postion of human in y axis.
      
    Return:
      f_hw_x: An vector(N x 1) which represents the human-wall interaction force in x axis. 
      f_hw_y: An vector(N x 1) which represents the human-wall interaction force in y axis.

    """    
    num_humans = np.shape(pos_x)[0]
    f_hw_x = np.zeros( (num_humans, 1) )
    f_hw_y = np.zeros( (num_humans, 1) )
    for human_index in range(num_humans):
        if pos_y[human_index] > 6:
            distance_hw = np.abs( 8 - pos_y[human_index] )
            f_hw = np.multiply( np.exp( np.divide( (radius_h - distance_hw),B_hwi ) ), A_hwi ) + np.multiply( np.maximum( (radius_h - distance_hw), 0 ) ,k_body_hwi )
            f_index_hw_y = np.multiply(f_hw, -1)
            f_index_hw_x = np.zeros(1)
        elif pos_x[human_index] >= 4 and pos_y[human_index] < 6:
            distance_hw = np.abs( pos_y[human_index] - 4 )
            f_hw = np.multiply( np.exp( np.divide( (radius_h - distance_hw),B_hwi ) ), A_hwi ) + np.multiply( np.maximum( (radius_h - distance_hw), 0 ) ,k_body_hwi )
            f_index_hw_y = np.multiply(f_hw, 1)
            f_index_hw_x = np.zeros(1)
        elif pos_x[human_index] < 4 and pos_y[human_index] < 4:
            distance_hw = np.abs( pos_x[human_index] - 4 )
            f_hw = np.multiply( np.exp( np.divide( (radius_h - distance_hw),B_hwi ) ), A_hwi ) + np.multiply( np.maximum( (radius_h - distance_hw), 0 ) ,k_body_hwi )
            f_index_hw_y = np.zeros(1)
            f_index_hw_x = np.multiply(f_hw, -1)
        elif pos_x[human_index] < 4 and pos_y[human_index] < 6 and pos_y[human_index] > 4 and np.sqrt(np.square(pos_x[human_index]-4) + np.square(pos_y[human_index]-4)) < 2:
            distance_hw_x = pos_x[human_index] - 4
            distance_hw_y = pos_y[human_index] - 4
            distance_hw = np.sqrt(np.square(distance_hw_x) + np.square(distance_hw_y))
            f_hw = np.multiply( np.exp( np.divide( (radius_h - distance_hw),B_hwi ) ), A_hwi ) + np.multiply( np.maximum( (radius_h - distance_hw), 0 ) ,k_body_hwi )
            distance_hw_norm_x = np.divide( distance_hw_x, distance_hw )
            distance_hw_norm_y = np.divide( distance_hw_y, distance_hw )
            f_index_hw_y = np.multiply( f_hw, distance_hw_norm_y )
            f_index_hw_x = np.multiply( f_hw, distance_hw_norm_x )
        else:
            f_index_hw_y = np.zeros(1)
            f_index_hw_x = np.zeros(1)
    
        f_hw_x[human_index] = f_index_hw_x
        f_hw_y[human_index] = f_index_hw_y    

    return f_hw_x, f_hw_y   

def human_robot_force(pos_x, pos_y, pos_rx, pos_ry):
    """Calculate the interaction force between human and robot.

    Note: make sure the shape of pos_x and pos_y is N x 1 not N.

    Args:
      pos_x: A vector(N x 1) which represents the postion of human in x axis.
      pos_y: A vector(N x 1) which represents the postion of human in y axis.
      pos_rx: scale which represents the postion of robot in x axis.
      pos_ry: scale which represents the postion of robot in y axis.
      
    Return:
      f_hr_x: An vector(N x 1) which represents the human-robot interaction force in x axis. 
      f_hr_y: An vector(N x 1) which represents the human-robot interaction force in y axis.

    """   
    num_humans = np.shape(pos_x)[0]
    f_hr_x = np.zeros( (num_humans, 1) )
    f_hr_y = np.zeros( (num_humans, 1) )
    for human_index in range(num_humans):
        distance_hr_x = pos_x[human_index] - pos_rx
        distance_hr_y = pos_y[human_index] - pos_ry
        distance_hr = np.sqrt(np.square( distance_hr_x ) + np.square( distance_hr_y ))
        if distance_hr < interact_region:
            f_hr = np.multiply( np.exp( np.divide((radius_hr - distance_hr), B_hri) ), A_hri )
            distance_hr_norm_x = np.divide( distance_hr_x, distance_hr )
            distance_hr_norm_y = np.divide( distance_hr_y, distance_hr )
            f_index_hr_x = np.multiply( f_hr, distance_hr_norm_x )
            f_index_hr_y = np.multiply( f_hr, distance_hr_norm_y )
        else:
            f_index_hr_x = np.zeros(1)
            f_index_hr_y = np.zeros(1)
        
        f_hr_x[human_index] = f_index_hr_x
        f_hr_y[human_index] = f_index_hr_y
        
    return f_hr_x, f_hr_y

def self_driven_force(pos_x, pos_y, vel_x, vel_y):
    """Calculate the interaction self-driven force.

    Note: make sure the shape of pos_x and pos_y is N x 1 not N.

    Args:
      pos_x: A vector(N x 1) which represents the postion of human in x axis.
      pos_y: A vector(N x 1) which represents the postion of human in y axis.
      vel_x: A vector(N x 1) which represents the velocity of human in x axis.
      vel_y: A vector(N x 1) which represents the velocity of human in y axis.
      
    Return:
      f_sd_x: An vector(N x 1) which represents the self-driven force in x axis. 
      f_sd_y: An vector(N x 1) which represents the self-driven force in y axis.

    """   
    num_humans = np.shape(pos_x)[0]
    f_sd_x = np.zeros( (num_humans, 1) )
    f_sd_y = np.zeros( (num_humans, 1) ) 
    
    for human_index in range(num_humans):
        distance_dest_x6 = 6 - pos_x[human_index]
        distance_dest_y6 = 6 - pos_y[human_index]
        distance_dest_66 = np.sqrt(np.square( distance_dest_x6 ) + np.square( distance_dest_y6 )) #distance to destination (6, 6)
        if pos_x[human_index] <= 4 and pos_y[human_index] <= 4:  #destination (4, 8)
            distance_dest_x = 4 - pos_x[human_index]
            distance_dest_y = 8 - pos_y[human_index]
            distance_dest = np.sqrt(np.square( distance_dest_x ) + np.square( distance_dest_y ))
        elif pos_x[human_index] < 6 and pos_y[human_index] > 4 and distance_dest_66 > 2: #destination (6, 6)
            distance_dest_x = 6 - pos_x[human_index]
            distance_dest_y = 6 - pos_y[human_index]
            distance_dest = np.sqrt(np.square( distance_dest_x ) + np.square( distance_dest_y ))
        else: #destination (50, 6)
            distance_dest_x = 50 - pos_x[human_index]
            distance_dest_y = 6 - pos_y[human_index]
            distance_dest = np.sqrt(np.square( distance_dest_x ) + np.square( distance_dest_y ))
        
        distance_dest_norm_x = np.divide( distance_dest_x, distance_dest ) 
        distance_dest_norm_y = np.divide( distance_dest_y, distance_dest )
        f_sd_x[human_index] = np.divide( np.multiply( desired_speed[human_index], distance_dest_norm_x ) - vel_x[human_index], accel_time ) 
        f_sd_y[human_index] = np.divide( np.multiply( desired_speed[human_index], distance_dest_norm_y ) - vel_y[human_index], accel_time ) 
    
    return f_sd_x, f_sd_y    



#f_hhi_matrix_x, f_hhi_matrix_y = human_human_force(pos_x, pos_y)    
#f_hhi_x = np.expand_dims( np.sum(f_hhi_matrix_x, axis=0), 1)
#f_hhi_y = np.expand_dims( np.sum(f_hhi_matrix_y, axis=0), 1)

#f_hwi_x, f_hwi_y = human_wall_force(pos_x, pos_y)
#f_hri_x, f_hri_y = human_robot_force(pos_x, pos_y, pos_rx, pos_ry)
#f_sd_x, f_sd_y = self_driven_force(pos_x, pos_y, vel_x, vel_y)

#f_comb_x = f_sd_x + f_hri_x + f_hwi_x + f_hhi_x
#f_comb_y = f_sd_y + f_hri_y + f_hwi_y + f_hhi_y
    
#flag_A = np.zeros( (num_flowA, 1) )  # flag for destination change
#flag_B = np.zeros( (num_flowB, 1) )     
#    
#def self_driven_force(pos_x, pos_y, index_flowA, index_flowB):
#    """Calculate the interaction self-driven force.
#
#    Note: make sure the shape of pos_x and pos_y is N x 1 not N.
#
#    Args:
#      pos_x: A vector(N x 1) which represents the postion of human in x axis.
#      pos_y: A vector(N x 1) which represents the postion of human in y axis.
#      index_flowA: A vector(M x 1) which represents the index of human in vector(N x 1) belongs to flow A
#      index_flowB: A vector(N-M x 1) which represents the index of human in vector(N x 1) belongs to flow B
#      
#    Return:
#      f_sd_x: An vector(N x 1) which represents the self-driven force in x axis. 
#      f_sd_y: An vector(N x 1) which represents the self-driven force in y axis.
#
#    """   
#    num_humans = np.shape(pos_x)[0]
#    num_flowA = np.shape(index_flowA)[0]
#    num_flowB = np.shape(index_flowB)[0]
#    f_sd_x = np.zeros( (num_humans, 1) )
#    f_sd_y = np.zeros( (num_humans, 1) ) 
#    
#    for flowA_iter in range(num_flowA):
#        hum_in_flowA = index_flowA[flowA_iter]
#        distance_dest_x = 6 - pos_x[hum_in_flowA]
#        distance_dest_y = 6 - pos_y[hum_in_flowA]
#        distance_dest = np.sqrt(np.square( distance_dest_x ) + np.square( distance_dest_y ))
#        if distance_dest < 2:
#            
#    
#    return f_sd_x, f_sd_y
    
#pos_x = np.array([[1], [2], [3], [4]])  
#pos_y = np.array([[3], [5], [6], [7]]) 
  




#d_B = np.divide((radius_hh - distance), B_hhi)
#d_exp = np.exp( np.divide((radius_hh - distance), B_hhi) ) 
#d_A = np.multiply( np.exp( np.divide((radius_hh - distance), B_hhi) ), A_hhi ) 


#body_g_input = radius_hh - distance
#body_g = np.maximum( (radius_hh - distance), np.zeros( np.shape(distance) ) )
#body_output = np.multiply( np.maximum( (radius_hh - distance), np.zeros( np.shape(distance) ) ), k_body_hhi )


#f_hhi = f_interact + f_body


#f_check = np.sqrt(np.square(f_hhi_x) + np.square(f_hhi_y))




    

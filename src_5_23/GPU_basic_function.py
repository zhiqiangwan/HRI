#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue May  2 09:29:02 2017

@author: pc7
"""
import math
import pycuda.driver as cuda
import pycuda.autoinit
from pycuda.compiler import SourceModule

def best_device_select():#Select the best installed GPU in case more than one are installed
   device_best = 0
   dev_compute_capability = cuda.Device(0).compute_capability()[0]
   for i in xrange(cuda.Device.count()):
      if dev_compute_capability<cuda.Device(i).compute_capability()[0]:
         dev_compute_capability = cuda.Device(i).compute_capability()[0]
         device_best = i
   return device_best      
def fet_parameters(device_best): #Fetch the parameters of the installed GPU
    global mem_size, gd_x, gd_y, gd_z, th_pr_bl    
    mem_size = float(cuda.Device(device_best).total_memory())
    gd_x = float(cuda.Device(device_best).get_attribute(cuda.device_attribute.MAX_GRID_DIM_X))
    gd_y = float(cuda.Device(device_best).get_attribute(cuda.device_attribute.MAX_GRID_DIM_Y))
    gd_z = float(cuda.Device(device_best).get_attribute(cuda.device_attribute.MAX_GRID_DIM_Z))
    th_pr_bl = float(cuda.Device(device_best).get_attribute(cuda.device_attribute.MAX_THREADS_PER_BLOCK))
    print "Max Threads per Block: "+str(th_pr_bl)
    return gd_x, gd_y, gd_z, th_pr_bl

def thread_block_grid(num_pedestrains,gd_x, gd_y, gd_z):
        
    block_x = 16            
    block_y = 16     
    block_z = 4
      
    grd_x = int(math.ceil(float(num_pedestrains)/(block_x*block_y*block_z)))
            
    if grd_x<=1:
       grid_xx = 1            
    elif (grd_x>1) and (grd_x<gd_x):
       grid_xx=grd_x
    else:
       grid_xx = gd_x

    grd_y = int(math.ceil(float(grd_x)/float(grid_xx)))
            
    if grd_y<=1:
       grid_yy = 1            
    elif (grd_y>1) and (grd_y<gd_y):
       grid_yy = grd_y
    else:
       grid_yy = gd_y
            
    grd_z = int(math.ceil(grd_y/grid_yy))
            
    if grd_z<=1:
       grid_zz = 1            
    elif (grd_z>1) and (grd_z<gd_z):
       grid_zz = grd_z
    else:
       grid_zz = gd_z
            
    grid_xx = int(grid_xx)
    grid_yy = int(grid_yy)
    grid_zz = int(grid_zz)
    return grid_xx, grid_yy, grid_zz        

#standard library imports
import time
import math
import os
import pickle

#related third party imports
import numpy as np
import scipy.stats as stats
import cv2
import tensorflow as tf
import matplotlib.pyplot as plt
import time


#local application/library specific imports
from CrowdDQNAgent import DeepQNetwork, experience_buffer


indx = 0
crowd = []
xsize = 8
ysize = 8
amp = 80
offset = 30
ped_ra = 13
ped_buffersize = 2
ped_spdlimit = 4
ped_outcount = 0
ped_outflow = []

flow_win_size = 4



num_flowA = 180#120#150#
num_flowB = 120#180#150#

flowA_init_wide = 3.6
flowA_x_start = 0
flowA_y_start = 4

flowB_init_wide = 3.6
flowB_x_start = 0
flowB_y_start = 0

init_space_ped = 0.4

ped_accel_rate_limit = 1000
interact_region = 1.5  #when distance > interact_region, no social force
num_pedestrains = 300
#parameters of human-human interaction force
A_hhi = 2.6
B_hhi = 0.31
#k_body_hhi = 2000
k_fric_hhi = 2000.0
radius_hh = 0.4 #radius of two humans
D_0 = 0.31
D_1 = 0.45
kay = 2.0
kappa = 1000
const_lambda = 0.25
A1aa = 2.6
A_rb = 1.0
#parameters of human-wall interaction force
A_hwi = 15.0
B_hwi = 0.1
radius_h = 0.5 * radius_hh #radius of one human
k_body_hwi = 1000 #20
#parameters of human-robot interaction force
A_hri = 5.0
B_hri = 0.5
radius_r = 0.5 #radius of robot
radius_hr = radius_h + radius_r
#parameters of self-driven force
accel_time = 0.5
np.random.seed(123)
desired_speed = 2 + 0.1 * np.random.randn(num_pedestrains, 1)
#lower_vel = 1.0
#upper_vel = 4.0
#mu_vel = 2
#sigma_vel = 0.3
#desired_speed = stats.truncnorm.rvs((lower_vel - mu_vel) / sigma_vel, (upper_vel - mu_vel) / sigma_vel, loc=mu_vel, scale=sigma_vel, size=(num_pedestrains, 1) )
desired_speed = desired_speed.astype(np.float32)
step_len = 0.005#0.01#
#robot action {action_key : robot velocity}
#action_dic = {'0': 'UP', 
#              '1': 'DOWN', 
#              '2': 'LEFT', 
#              '3': 'RIGHT',
#              '4': 'STOP'}
action_dic = {'0': 'UP', 
              '1': 'DOWN', 
              '2': 'LEFT', 
              '3': 'RIGHT'}
radius_robot_in_env = 0.2


pos_rx = 0.5
pos_ry = 2.5
vel_rx = 0
vel_ry = 0
init_pos_rx = 2.0#1.50
phi = 0.0

rx_min = 0.0
rx_max = 3.8
ry_min = 0.0
ry_max = 3.8

# train parameters
batch_size = 32 #How many experiences to use for each training step.
update_freq = 4 #How often to perform a training step.
y = .99 #Discount factor on the target Q-values
startE = 1 #Starting chance of random action
endE = 0.1 #Final chance of random action
anneling_steps = 100000. #How many steps of training to reduce startE to endE.
num_episodes = 5000 #How many episodes of game environment to train network with.

pre_train_steps = 100000 # 10000 #How many steps of random actions before training begins.
max_epLength = 400#1000 #The max allowed length of our episode.
load_model = True#False #  Whether to load a saved model.
tau = 0.001 #Rate to update target network toward primary network
ob_len = 200 #size of the image of the environment
action_size = len(action_dic)
num_sim_steps = 30000

#'../models/7_10_flowA_B_180_120_random_robot_position_rang_0_3_8'-----'DDQNmodel800000.cptk'
# Check point
if load_model:
    model_path = '../models/7_10_flowA_B_180_120_random_robot_position_rang_0_3_8'#'../models/7_7_flowA_B_180_120_random_robot_position'#'../models/6-26_flowA_B_180_120_random_robot_position'#'
else:
    model_path = '../models/'

# DQN
# some useful functions
def updateTargetGraph(tfVars,tau):
    with tf.device('/gpu:1'): 
        total_vars = len(tfVars)
        op_holder = []
        for idx,var in enumerate(tfVars[0:total_vars/2]):
            op_holder.append(tfVars[idx+total_vars/2].assign((var.value()*tau) + ((1-tau)*tfVars[idx+total_vars/2].value())))
    return op_holder

def updateTarget(op_holder,sess):
    for op in op_holder:
        sess.run(op)

with tf.device('/gpu:1'):                
    tf.reset_default_graph()
    
    mainQN = DeepQNetwork(ob_len, action_size)
    targetQN = DeepQNetwork(ob_len, action_size)
    saver = tf.train.Saver()
    trainables = tf.trainable_variables()
    targetOps = updateTargetGraph(trainables, tau)
    copyOps = updateTargetGraph(trainables, 1.0)
    
myBuffer = experience_buffer()

config = tf.ConfigProto(log_device_placement=True)
config.gpu_options.allow_growth = True
sess = tf.InteractiveSession(config=config)
#Set the rate of random action decrease. 
e = startE
stepDrop = (startE - endE)/anneling_steps
#create lists to contain total rewards and steps per episode
jList = []
rList = []
ped_outcount_list = []
total_steps = 0
ShowInterval = 10

if load_model == True:
    print 'Loading Model...'
    #ckpt = tf.train.get_checkpoint_state(model_path)
    saver.restore(sess,os.path.join(model_path, 'DDQNmodel800000.cptk'))

#if load_model == True:
#    print 'Loading Model...'
#    ckpt = tf.train.get_checkpoint_state(model_path)
#    saver.restore(sess,ckpt.model_checkpoint_path)
#else:
#    print 'initialize Model parameters...'
#    tf.global_variables_initializer().run()

os.environ["CUDA_DEVICE"] = "0"
import pycuda.driver as cuda
import pycuda.autoinit
from pycuda.compiler import SourceModule

import GPU_basic_function
from GPU_basic_function import *

indx = 0
device_best = 0
gd_x, gd_y, gd_z, th_pr_bl = fet_parameters(device_best)
grid_xx, grid_yy, grid_zz = thread_block_grid(num_pedestrains,gd_x, gd_y, gd_z) 
ped_x = np.zeros(((num_flowA+num_flowB),1),np.float32)
ped_y = np.zeros(((num_flowA+num_flowB),1),np.float32)
ped_vx = np.zeros(((num_flowA+num_flowB),1),np.float32)
ped_vy = np.zeros(((num_flowA+num_flowB),1),np.float32)
ped_c = np.zeros(((num_flowA+num_flowB),1),np.float32)
ped_cnt_flag1 = np.zeros(((num_flowA+num_flowB),1),np.float32)
ped_cnt_flag2 = np.zeros(((num_flowA+num_flowB),1),np.float32)
rndnum = np.zeros((2*(num_flowA+num_flowB),1),np.float32)
ped_x_gpu = cuda.mem_alloc(ped_x.nbytes)
ped_y_gpu = cuda.mem_alloc(ped_y.nbytes)
ped_vx_gpu = cuda.mem_alloc(ped_vx.nbytes)
ped_vy_gpu = cuda.mem_alloc(ped_vy.nbytes)
ped_x_gpu_up = cuda.mem_alloc(ped_x.nbytes)
ped_y_gpu_up = cuda.mem_alloc(ped_y.nbytes)
ped_vx_gpu_up = cuda.mem_alloc(ped_vx.nbytes)
ped_vy_gpu_up = cuda.mem_alloc(ped_vy.nbytes)
ped_c_gpu = cuda.mem_alloc(ped_c.nbytes)
rndnum_gpu = cuda.mem_alloc(rndnum.nbytes)
ped_cnt_flag1_gpu = cuda.mem_alloc(ped_cnt_flag1.nbytes)
ped_cnt_flag2_gpu = cuda.mem_alloc(ped_cnt_flag2.nbytes)
f_x_gpu = cuda.mem_alloc(ped_x.nbytes)
f_y_gpu = cuda.mem_alloc(ped_x.nbytes)
desired_speed_gpu = cuda.mem_alloc(desired_speed.nbytes)
cout1 = np.zeros((int(flow_win_size),1))
cout2 = np.zeros((int(flow_win_size),1))

sampl = int(1/step_len)
out_flow = []
cc1=[]   
cnt_flow = 0  
out_flow_cal = 0.0


def init_flowAB():
    global indx, phi, cout1, cnt_flow, out_flow_cal, pos_rx, pos_ry
    #init parameter
    phi = 0.0 #robot
    cout1 = np.zeros((int(flow_win_size),1))
    cnt_flow = 0  
    out_flow_cal = 0.0
    indx = 0
    
    #random initialize robot position , uniform (0.5~3.5)
    pos_rx = np.random.uniform(rx_min, rx_max)
    pos_ry = np.random.uniform(ry_min, ry_max)
#    pos_rx = 0.5
#    pos_ry = 2.5

    
    #init the position of flowA    
    flowA_x_index = 0
    num_ped_each_row = int(math.floor(flowA_init_wide/init_space_ped))
    for flowA_index in range(num_flowA):
        flowA_y_index = 1 + flowA_index % num_ped_each_row
        if flowA_index % num_ped_each_row ==0:
            flowA_x_index += 1
        init_pos_x = flowA_x_start-init_space_ped*flowA_x_index 
        init_pos_y = flowA_y_start+init_space_ped*flowA_y_index
        ped_x[indx] = init_pos_x
        ped_y[indx] = init_pos_y
        ped_vx[indx] = 2.0
        ped_vy[indx] = 0.0
        ped_c[indx] = 1.0#red
        ped_cnt_flag1[indx] = 0.0
        ped_cnt_flag2[indx] = 0.0
        indx = indx+1
    #init the position of flowB       

    flowB_y_index = 0
    for flowB_index in range(num_flowB):
        flowB_x_index = 1 + flowB_index % num_ped_each_row
        if flowB_index % num_ped_each_row ==0:
            flowB_y_index += 1
        init_pos_x = flowB_x_start+init_space_ped*flowB_x_index
        init_pos_y = flowB_y_start-init_space_ped*flowB_y_index    
        ped_x[indx] = init_pos_x
        ped_y[indx] = init_pos_y
        ped_vx[indx] = 0.0
        ped_vy[indx] = 2.0
        ped_c[indx] = 2.0#blue
        ped_cnt_flag1[indx] = 0.0
        ped_cnt_flag2[indx] = 0.0
        indx = indx+1


func1 = SourceModule("""

        #include <stdio.h>
        #include <math.h>
        __global__ void func1(float *ped_x_up, float *ped_y_up, float *ped_vx_up, float *ped_vy_up, float *rndnum, float *ped_x, float *ped_y, float *ped_vx, float *ped_vy, float xsize, float *ped_c, float *ped_cnt_flag1, float *ped_cnt_flag2, float ped_buffersize, float spd_lmt, float nu_hu,float radius_h, float k_body_hwi, float A_hwi, float B_hwi, float pos_rx, float pos_ry,float interact_region, float A_hri, float B_hri, float radius_hr, float robot_flag, float *desired_speed, float accel_time, float kay, float kappa, float d0, float d1, float const_lambda, float a1aa, float step_len)
        {
        int blockId = blockIdx.x+ blockIdx.y * gridDim.x+ gridDim.x * gridDim.y * blockIdx.z;
        int threadId = blockId * (blockDim.x * blockDim.y * blockDim.z)+ (threadIdx.z * (blockDim.x * blockDim.y))+ (threadIdx.y * blockDim.x)+ threadIdx.x;
        float spd;
        if (ped_x[threadId]>xsize){
               if(abs(ped_c[threadId]-2)<0.1){//blue flow
                  ped_x[threadId] = abs(rndnum[threadId*2]) * 3 +0.5; //0.5~3.5
                  ped_y[threadId] = (-1*rndnum[threadId*2+1] * ped_buffersize) - 3; //-3-buf~-3
                  ped_x_up[threadId] = abs(rndnum[threadId*2]) * 3 +0.5;
                  ped_y_up[threadId] = (-1*rndnum[threadId*2+1] * ped_buffersize) - 3;
                  ped_vx[threadId] = 0.0;
                  ped_vy[threadId] = 2.0;
                  ped_vx_up[threadId] = 0.0;
                  ped_vy_up[threadId] = 2.0;
                 
               }
               if(abs(ped_c[threadId]-1)<0.1){//red flow
                  ped_x[threadId] = (-1*rndnum[threadId*2] * ped_buffersize) - 3; // -3-buf~-3
                  ped_y[threadId] = rndnum[threadId*2+1] * 3 + 4.5; //4.5~7.5
                  ped_x_up[threadId] = (-1*rndnum[threadId*2] * ped_buffersize) - 3;
                  ped_y_up[threadId] = rndnum[threadId*2+1] * 3 + 4.5;
                  ped_vx[threadId] = 2.0;
                  ped_vy[threadId] = 0.0;
                  ped_vx_up[threadId] = 2.0;
                  ped_vy_up[threadId] = 0.0; 
               }
            
            } 

        //Pedestrian and wall force calculation
        
        float distance_hw, f_hw_y, f_hw_x, f_hw,distance_hw_x,distance_hw_y,distance_hw_norm_x,distance_hw_norm_y;
        f_hw_x = 0.0;
        f_hw_y = 0.0;
        f_hw=0.0;
        
        if (ped_y[threadId]>6){
            distance_hw = abs(8-ped_y[threadId]);
            f_hw = (exp(((radius_h - distance_hw)/B_hwi))* A_hwi) + max((radius_h - distance_hw),0.0)*k_body_hwi;
            f_hw_y = -1*f_hw;
            f_hw_x = 0;        
        }
        else if ((ped_x[threadId] >= 4 or ped_x[threadId] < 0) and (ped_y[threadId] < 6) and (ped_y[threadId] > 4)){
            distance_hw = abs( ped_y[threadId] - 4 );
            f_hw = ( exp(((radius_h - distance_hw)/B_hwi))* A_hwi ) + ( max((radius_h - distance_hw),0.0)*k_body_hwi);
            f_hw_y = f_hw;
            f_hw_x = 0.0;                
        }
        else if (ped_x[threadId] < 4 and ped_x[threadId] > 2 and ped_y[threadId] < 4){
            distance_hw = abs(ped_x[threadId]-4);
            f_hw = (exp(((radius_h - distance_hw)/B_hwi))*A_hwi) + (max((radius_h-distance_hw),0.0)*k_body_hwi);
            f_hw_y = 0.0;
            f_hw_x = f_hw*-1;            
        }
        else if (ped_x[threadId] < 2 and ped_x[threadId] > 0 and ped_y[threadId] < 4){
            distance_hw = abs(ped_x[threadId] );
            f_hw = (exp(((radius_h - distance_hw)/B_hwi))*A_hwi) + ( max((radius_h-distance_hw),0.0)*k_body_hwi);
            f_hw_y = 0;
            f_hw_x = f_hw;
        }
        else if ((ped_x[threadId] < 4) and (ped_y[threadId] < 6) and (ped_y[threadId] > 4) and ((sqrt(pow((ped_x[threadId]-4),2))+ pow((ped_y[threadId]-4),2))<2)){
            distance_hw_x = ped_x[threadId]-4;
            distance_hw_y = ped_y[threadId]-4;
            distance_hw = sqrt(pow(distance_hw_x,2)+pow(distance_hw_y,2));
            f_hw = (exp(((radius_h-distance_hw)/B_hwi))*A_hwi) + (max((radius_h-distance_hw),0.0)*k_body_hwi);
            distance_hw_norm_x = (distance_hw_x/distance_hw);
            distance_hw_norm_y = (distance_hw_y/distance_hw);
            f_hw_y = (f_hw*distance_hw_norm_y);
            f_hw_x = (f_hw*distance_hw_norm_x);
            }
        else{
            f_hw_y = 0;
            f_hw_x = 0;        
        }
        
       //human and robot interaction force calculation 
       float distance_hr_x,distance_hr_y,distance_hr,f_hr,distance_hr_norm_x,distance_hr_norm_y,f_index_hr_x,f_index_hr_y;
        f_index_hr_x = 0.0;
        f_index_hr_y = 0.0;
        if (robot_flag>0){
        distance_hr_x = ped_x[threadId] - pos_rx;
        distance_hr_y = ped_y[threadId] - pos_ry;
        distance_hr = sqrt(pow(distance_hr_x,2) + pow(distance_hr_y,2));
        if (distance_hr < interact_region){
           f_hr = exp(((radius_hr-distance_hr)/B_hri))*A_hri;
           distance_hr_norm_x = distance_hr_x/distance_hr;
           distance_hr_norm_y = distance_hr_y/distance_hr;
           f_index_hr_x = f_hr*distance_hr_norm_x;
           f_index_hr_y = f_hr*distance_hr_norm_y;
           
        }
        else{
           f_index_hr_x = 0.0;
           f_index_hr_y = 0.0;
        }    
        }
        
        //human to human force calculation

        float distance_dest_x6, distance_dest_y6, distance_dest_66, distance_dest,distance_dest_x,distance_dest_y,distance_dest_norm_x,distance_dest_norm_y,f_sd_x,f_sd_y;
        distance_dest_x6 = 6 - ped_x[threadId];
        distance_dest_y6 = 6 - ped_y[threadId];
        distance_dest_66 = sqrt(pow(distance_dest_x6,2)+pow(distance_dest_y6,2));
        f_sd_x = 0.0;
        f_sd_y = 0.0;

        if ((ped_x[threadId] <= 4) and (ped_y[threadId] <= 4)){
           distance_dest_x = 4 - ped_x[threadId];
           distance_dest_y = 8 - ped_y[threadId];
           distance_dest = sqrt(pow(distance_dest_x,2) + pow(distance_dest_y,2));
        }
        else if ((ped_x[threadId] < 6) and (ped_y[threadId] > 4) and (distance_dest_66 > 2)){
           distance_dest_x = 6 - ped_x[threadId];
           distance_dest_y = 6 - ped_y[threadId];
           distance_dest = sqrt(pow(distance_dest_x,2)+pow(distance_dest_y,2));
        }
        else{
           distance_dest_x = 50 - ped_x[threadId];
           distance_dest_y = 6 - ped_y[threadId];
           distance_dest = sqrt(pow(distance_dest_x,2)+pow(distance_dest_y,2));
        }
            
        distance_dest_norm_x = distance_dest_x/distance_dest; 
        distance_dest_norm_y = distance_dest_y/distance_dest;
        f_sd_x = ((desired_speed[threadId]*distance_dest_norm_x)-ped_vx[threadId])/accel_time;
        f_sd_y = ((desired_speed[threadId]*distance_dest_norm_y)-ped_vy[threadId])/accel_time;
        
        float f_hh_x,f_hh_y,dist_ped,n_ab_x,n_ab_y,t_ab_x,t_ab_y,gab;
        f_hh_x = 0.0;
        f_hh_y = 0.0;
        float radius_hh = 2*radius_h;

        float con1, con2, con3;
        for (int m=0;m<nu_hu;m=m+1){
            if(m!=threadId){
                 dist_ped = sqrt(pow((ped_x[threadId]-ped_x[m]),2) + pow((ped_y[threadId]-ped_y[m]),2));//distance between the pedestrians
                 if(dist_ped<interact_region ){
                     n_ab_x = (ped_x[threadId]-ped_x[m])/dist_ped;//unit vector x
                     n_ab_y = (ped_y[threadId]-ped_y[m])/dist_ped;//unit vector y
                     t_ab_x = -1*(ped_y[threadId]-ped_y[m])/dist_ped;
                     t_ab_y = (ped_x[threadId]-ped_x[m])/dist_ped;
                     if(dist_ped>radius_hh) gab=0;
                     else gab = radius_hh-dist_ped;
                     con1 = a1aa*exp((((-1*dist_ped)/d0))+(pow(d1/dist_ped,kay)));
                     con2 = const_lambda+((1-const_lambda)*((1-(n_ab_x*distance_dest_norm_x)-(n_ab_y*distance_dest_norm_y))/2));
                     con3 = kappa*gab*(((ped_vx[m]-ped_vx[threadId])*t_ab_x)+((ped_vy[m]-ped_vy[threadId])*t_ab_y));
                     f_hh_x = f_hh_x + con1*n_ab_x*con2+con3*t_ab_x;
                     f_hh_y = f_hh_y + con1*n_ab_y*con2+con3*t_ab_y;
                 }
            }
        
        }
        if (isnan(f_hw_x)|| isinf(f_hw_x)) f_hw_x = 0.0;
        if (isnan(f_hw_y)|| isinf(f_hw_y)) f_hw_y = 0.0;
        if (isnan(f_index_hr_x)|| isinf(f_index_hr_x)) f_index_hr_x = 0.0;
        if (isnan(f_index_hr_y)|| isinf(f_index_hr_y)) f_index_hr_y = 0.0;
        if (isnan(f_sd_x)|| isinf(f_sd_x)) f_sd_x = 0.0;
        if (isnan(f_sd_y)|| isinf(f_sd_y)) f_sd_y = 0.0;
        if (isnan(f_hh_x)|| isinf(f_hh_x)) f_hh_x = 0.0;
        if (isnan(f_hh_y)|| isinf(f_hh_y)) f_hh_y = 0.0;
        float f_x, f_y;
        f_x = f_hw_x+f_index_hr_x+f_sd_x+f_hh_x;
        f_y = f_hw_y+f_index_hr_y+f_sd_y+f_hh_y;
        float xl = 10.0;
        float yl = 10.0;
        if (abs(f_x)>xl) {if (f_x>0.0) f_x = xl;if (f_x<0.0) f_x = -1.0*xl;}
        if (abs(f_y)>yl) {if (f_y>0.0) f_y = yl;if (f_y<0.0) f_y = -1.0*yl;}        
       
        ped_x_up[threadId] = ped_x[threadId] + ( ped_vx[threadId] + 0.5*f_x*step_len ) * step_len;
        ped_y_up[threadId] = ped_y[threadId] + ( ped_vy[threadId] + 0.5*f_y*step_len ) * step_len;
        ped_vx_up[threadId] = ped_vx[threadId] + f_x * step_len;
        ped_vy_up[threadId] = ped_vy[threadId] + f_y * step_len; 
        spd = sqrt(pow(ped_vx_up[threadId],2)+pow(ped_vy_up[threadId],2));                
        if (spd>spd_lmt){
            ped_vx_up[threadId] = (spd_lmt/spd)*ped_vx_up[threadId];
            ped_vy_up[threadId] = (spd_lmt/spd)*ped_vy_up[threadId];        
        
        }
        
        if (ped_x_up[threadId]>xsize) ped_cnt_flag1[threadId]=1.0;
        else ped_cnt_flag1[threadId]=0.0;
        if ((ped_x_up[threadId]>4.0) and (ped_x_up[threadId]<8.0)) ped_cnt_flag2[threadId]=1.0;
        else ped_cnt_flag2[threadId]=0.0;

        if (ped_x_up[threadId] < -10.0) ped_x_up[threadId] = -10.0;
        if (ped_y_up[threadId] > 9.0) ped_y_up[threadId] = 9.0;
        else if (ped_y_up[threadId] < -10.0) ped_y_up[threadId] = -10.0;
        
        }
                                                
""")  

func1 = func1.get_function("func1")       
    

def move_crowd(robot_flag,render_flag,omega,numthstep, gener_flag):
    # check, remove and add
    global ped_outcount, vel_rx, init_pos_rx, pos_rx, pos_ry, cout1, cout2, phi, A_rb, cnt_flow,cc1,sampl,out_flow_cal 
    rndnum = np.random.random((num_flowA+num_flowB)*2)
    rndnum = rndnum.astype(np.float32)
    cuda.memcpy_htod(ped_x_gpu,ped_x)
    cuda.memcpy_htod(ped_y_gpu,ped_y)
    cuda.memcpy_htod(ped_vx_gpu,ped_vx)
    cuda.memcpy_htod(ped_vy_gpu,ped_vy)
    cuda.memcpy_htod(ped_c_gpu,ped_c)
    cuda.memcpy_htod(ped_cnt_flag1_gpu,ped_cnt_flag1)
    cuda.memcpy_htod(ped_cnt_flag2_gpu,ped_cnt_flag2)
    cuda.memcpy_htod(rndnum_gpu,rndnum)
    cuda.memcpy_htod(desired_speed_gpu,desired_speed)
    func1(ped_x_gpu_up,ped_y_gpu_up,ped_vx_gpu_up,ped_vy_gpu_up,rndnum_gpu, ped_x_gpu, ped_y_gpu, ped_vx_gpu, ped_vy_gpu, np.float32(xsize), ped_c_gpu, ped_cnt_flag1_gpu, ped_cnt_flag2_gpu, np.float32(ped_buffersize), np.float32(ped_spdlimit),np.float32(num_pedestrains), np.float32(radius_h), np.float32(k_body_hwi), np.float32(A_hwi),np.float32(B_hwi),np.float32(pos_rx), np.float32(pos_ry), np.float32(interact_region), np.float32(A_hri), np.float32(B_hri), np.float32(radius_hr), np.float32(robot_flag), desired_speed_gpu, np.float32(accel_time), np.float32(kay), np.float32(kappa), np.float32(D_0), np.float32(D_1), np.float32(const_lambda), np.float32(A1aa), np.float32(step_len), block=(16,16,4), grid=(grid_xx,grid_yy,grid_zz))
    cuda.memcpy_dtoh(ped_x,ped_x_gpu_up)
    cuda.memcpy_dtoh(ped_y,ped_y_gpu_up)              
    cuda.memcpy_dtoh(ped_vx,ped_vx_gpu_up)
    cuda.memcpy_dtoh(ped_vy,ped_vy_gpu_up)
    cuda.memcpy_dtoh(ped_cnt_flag1,ped_cnt_flag1_gpu)

    c1 = float(np.sum(ped_cnt_flag1))
    cout1=np.append(cout1,c1)
    #pos_rx = A_rb*math.sin(phi*step_len) + init_pos_rx #A_rb*math.acos(phi*step_len) + pos_rx
#    pos_rx = -A_rb*math.cos(phi*step_len) + init_pos_rx
#    phi += omega
    if ((numthstep>=sampl) and (divmod(numthstep,sampl)[1]==0)):
        out_flow_cal = np.sum(cout1[cnt_flow*sampl:(cnt_flow+1)*sampl])
        cnt_flow = cnt_flow+1
    
    if gener_flag:        
        if ((robot_flag<0.0) and (render_flag==True)):
            img1 = render_scene(False,True)
        if ((robot_flag>0.0) and (render_flag==True)):
            img1 = render_scene(True,True)
        if ((robot_flag<0.0) and (render_flag==False)):
            img1 = render_scene(False,False)
        if ((robot_flag>0.0) and (render_flag==False)):
            img1 = render_scene(True,False)
        return img1,out_flow_cal
        


def render_scene(robot_flag=False,shd_show=True):
    img=[]
    x = xsize*amp + 2*offset
    y = ysize*amp + 2*offset
    x_2 = int(x/2)
    y_2 = int(y/2)    
    img = 255*np.ones((int(x),int(y),3), np.uint8)
    cv2.rectangle(img,(x_2,y_2),(int(x),int(y)),(0,0,0),2)
    if robot_flag:
        r_x_loc = int(float(pos_rx*amp))+offset
        r_y_loc = int(float((ysize-pos_ry)*amp))+offset
        rb_pixels = int((radius_robot_in_env*amp))
        img[r_y_loc-rb_pixels:r_y_loc+rb_pixels,r_x_loc-rb_pixels:r_x_loc+rb_pixels,] = (0,0,0)
    for i in range(0,num_pedestrains):
        p_x = float(ped_x[i]*amp)
        p_y = float((ysize-ped_y[i])*amp)
        if math.isnan(p_x):
            p_x=0.0
        if math.isnan(p_y):
            p_y=0.0
        p_x =int(p_x)+offset
        p_y =int(p_y)+offset
        if(abs(ped_c[i]-1)<0.1):
            cv2.circle(img,(p_x,p_y), ped_ra, (0,0,255), -1)
        elif(abs(ped_c[i]-2)<0.1):
            cv2.circle(img,(p_x,p_y), ped_ra, (255,0,0), -1)
        else:
            cv2.circle(img,(p_x,p_y), ped_ra, (0,255,255), -1)  
    if shd_show:
        cv2.imshow('image',img)
        cv2.waitKey(1)
        
    img = cv2.resize(img, (ob_len, ob_len))
    img = cv2.cvtColor( img, cv2.COLOR_RGB2GRAY )
    img = img/255.0    
    return img

PLOT_RESULT = False
simulation_time = 400
numstep = sampl*simulation_time
num_experiments = 10

robot_step_len = 0.2
for episode in range(num_episodes):
    episodeBuffer = experience_buffer()
    # reset environment and get first observation
    simstep = 0
    env_step = 0
    #crowd = []
    init_flowAB()
    # I need to initialise robot position, velocity and flag
    
    s, _ = move_crowd(1.0,False,0.0,env_step, True)
    env_step += 1
    #s = image2array(img)

    d = False
    rAll = 0
   

    while simstep < num_sim_steps:
        simstep += 1
        total_steps += 1
        
        if load_model == False:
            if np.random.rand(1) < e or total_steps < pre_train_steps:
                a = np.random.randint(0, len(action_dic))
            else:
                a = sess.run(mainQN.predict,feed_dict={mainQN.observation:np.expand_dims(s, axis=0)})[0]
        else:
            a = sess.run(mainQN.predict,feed_dict={mainQN.observation:np.expand_dims(s, axis=0)})[0]
            
        omega_rx = 0.0

        move_direction = action_dic[str(a)]
        if move_direction == 'UP':
            pos_ry = pos_ry + robot_step_len
        elif move_direction == 'DOWN':
            pos_ry = pos_ry - robot_step_len
        elif move_direction == 'LEFT':
            pos_rx = pos_rx - robot_step_len
        elif move_direction == 'RIGHT':
            pos_rx = pos_rx + robot_step_len
        elif move_direction == 'STOP':
            pass
        else: raise ValueError('unacceptable moving direction')     
        
        if pos_rx > rx_max:
            pos_rx = rx_max
        elif pos_rx < rx_min:
            pos_rx = rx_min
            
        if pos_ry > 7.5:
            pos_ry = 7.5
        elif pos_ry < ry_min:
            pos_ry = ry_min
        
        for i in range(sampl):
            move_crowd(1.0,False,omega_rx,env_step, False)
            env_step += 1

        s1, r = move_crowd(1.0,False,omega_rx,env_step, True)
        env_step += 1
        
                            
        # save model
        if total_steps % 100000 == 0:
            saver.save(sess, os.path.join( model_path, 'DDQNmodel' + str(total_steps) + '.cptk') )
            print('Model Saved at total step:', total_steps)
        
        # save to experience buffer
        episodeBuffer.add(np.reshape(np.array([s,a,r,s1,d]),[1,5]))

        if total_steps > pre_train_steps:
            if e > endE:
                e -= stepDrop
            if total_steps % (update_freq) == 0:
                # get random batch of experience
                trainBatch = myBuffer.sample(batch_size)
                # Perform the Double-DQN update to the target Q-values
                Q1 = sess.run(mainQN.predict,feed_dict={mainQN.observation:np.reshape(np.vstack(trainBatch[:,3]), [batch_size, ob_len, ob_len])})
                Q2 = sess.run(targetQN.Qout,feed_dict={targetQN.observation:np.reshape(np.vstack(trainBatch[:,3]), [batch_size, ob_len, ob_len])})
                end_multiplier = -(trainBatch[:,4] - 1)
                doubleQ = Q2[range(batch_size),Q1]
                targetQ = trainBatch[:,2] + (y*doubleQ * end_multiplier)
                # update the network with our target values
                _ = sess.run(mainQN.updateModel, feed_dict={ mainQN.observation:np.reshape(np.vstack(trainBatch[:,0]), [batch_size, ob_len, ob_len]), 
                                                             mainQN.targetQ:targetQ, 
                                                             mainQN.actions:trainBatch[:,1]})
                # update the target network in the direction of the primary network
                updateTarget(targetOps, sess)
                #print 'update network'

        rAll += r
        s = s1

        if simstep > max_epLength:
            break
    print('Ep:', episode, 'TS:', total_steps, 'ES:', simstep, 'TR:', rAll)
    # END of one episode    

    # get all experience from this episode and discount their rewards
    myBuffer.add(episodeBuffer.buffer)
    #jList.append(simstep)
    rList.append(rAll)


with open('../models/workspace.pkl', 'w') as f:
    pickle.dump(rList, f)
#(6323-5973)/5973.0

with open(os.path.join(model_path, 'workspace.pkl'), 'r') as f:
   rList = pickle.load(f)

window_size = 5
average_rList = np.convolve(rList, np.ones((window_size,))/window_size, mode='valid')   
plt.plot(average_rList)
plt.ylabel('Accumulated rewards')
plt.xlabel('Epoch') 

with open(os.path.join(model_path, 'average_accumulated_flow.pkl'), 'r') as f:
    average_accumulated_flow = pickle.load(f)
#accum_flow_no_robot_multip_runs, accum_flow_rand_robot_multip_runs, accum_flow_DQN_robot_multip_runs 


fig = plt.figure(1, figsize=(6, 4))
ax = fig.add_subplot(111)
bp = ax.boxplot(average_accumulated_flow)
ax.set_xticklabels(['No robot', 'Random', 'DQN'])

with open(os.path.join(model_path, 'converge_result.pkl'), 'r') as f:
    converge_result = pickle.load(f)

for i in range(7):    
    fig = plt.figure(1, figsize=(6, 4))
    ax = fig.add_subplot(111)
    bp = ax.boxplot(converge_result[0][0+i*7:7+i*7])
    plt.show()
#[average_accum_outflow_list, pos_rx_stable_list, pos_ry_stable_list, converge_time_list]

# END of all episode

#plt.plot(rList)
#plt.ylabel('Accumulated outflow')
#plt.xlabel('Epoch') 


    
if PLOT_RESULT == True:
    
    accum_flow_no_robot_multip_runs = []
    for i in range(num_experiments):    
        print ("Without robot:")
        instane_flow_no_robot = []
        accum_flow_no_robot = []

        init_flowAB()    
        for step in range(numstep+1):
            if step % sampl ==0:
                _, _, = move_crowd(robot_flag = -1.0, render_flag = False, omega=0, numthstep=step, gener_flag=True)
            else:            
                move_crowd(robot_flag = -1.0, render_flag = False, omega=0, numthstep=step, gener_flag=False) #robot_flag < 0 , no robot
            accum_flow_no_robot.append(np.sum(cout1))
        print ("end of Without robot:")
        accum_flow_no_robot_multip_runs.append(accum_flow_no_robot[-1])
        
        for i in range(0,simulation_time):
            instane_flow_no_robot.append(np.sum(cout1[i*sampl:(i+1)*sampl]))

    plt.plot(instane_flow_no_robot)
    plt.show()
    print(accum_flow_no_robot[-1])
    sum(accum_flow_no_robot_multip_runs)/float(len(accum_flow_no_robot_multip_runs))
#    plt.show()


    accum_flow_stil_robot_multip_runs = []
    for i in range(num_experiments):  
        print ("Robot is stil:")
        instane_flow_stil_robot = []
        accum_flow_stil_robot = []      
        init_flowAB() 
        pos_rx = 2.0
        pos_ry = 4.0
        for step in range(numstep+1):
            if step % sampl ==0:
                _, _, = move_crowd(robot_flag = 1.0, render_flag = True, omega=0, numthstep=step, gener_flag=True)
            else:            
                move_crowd(robot_flag = 1.0, render_flag = False, omega=0, numthstep=step, gener_flag=False) #robot_flag < 0 , no robot
            accum_flow_stil_robot.append(np.sum(cout1))
        print ("end of Robot is stil:")
        accum_flow_stil_robot_multip_runs.append(accum_flow_stil_robot[-1])
        print(accum_flow_stil_robot[-1])
        
        
        for i in range(0,simulation_time):
            instane_flow_stil_robot.append(np.sum(cout1[i*sampl:(i+1)*sampl]))
        
    plt.plot(instane_flow_stil_robot)
    plt.show()
    print(accum_flow_stil_robot[-1])
    sum(accum_flow_stil_robot_multip_runs)/float(len(accum_flow_stil_robot_multip_runs))
#    plt.plot(accum_flow_stil_robot)
#    plt.show()


    accum_flow_rand_robot_multip_runs = []
    for i in range(num_experiments):  
        print ("Robot takes random action:")
        instane_flow_rand_robot = []
        accum_flow_rand_robot = []      
        init_flowAB() 
        pos_rx = 0.5
        pos_ry = 0.5
        for step in range(numstep+1):
            if step % sampl ==0:
                a = np.random.randint(0, len(action_dic))
    
                omega_rx = 0.0
        
                move_direction = action_dic[str(a)]
                if move_direction == 'UP':
                    pos_ry = pos_ry + robot_step_len
                elif move_direction == 'DOWN':
                    pos_ry = pos_ry - robot_step_len
                elif move_direction == 'LEFT':
                    pos_rx = pos_rx - robot_step_len
                elif move_direction == 'RIGHT':
                    pos_rx = pos_rx + robot_step_len
                elif move_direction == 'STOP':
                    pass                    
                else: raise ValueError('unacceptable moving direction') 

                if pos_rx > rx_max:
                    pos_rx = rx_max
                elif pos_rx < rx_min:
                    pos_rx = rx_min
                    
                if pos_ry > 7.5:
                    pos_ry = 7.5
                elif pos_ry < ry_min:
                    pos_ry = ry_min                
                    
                _, _, = move_crowd(robot_flag = 1.0, render_flag = False, omega=0, numthstep=step, gener_flag=True)    
            else:   
                move_crowd(robot_flag = 1.0, render_flag = False, omega=omega_rx, numthstep=step, gener_flag=False) #robot_flag < 0 , no robot
            accum_flow_rand_robot.append(np.sum(cout1))
    
        print ("end of Robot takes random action:")
        accum_flow_rand_robot_multip_runs.append(accum_flow_rand_robot[-1])
    
        for i in range(0,simulation_time):
            instane_flow_rand_robot.append(np.sum(cout1[i*sampl:(i+1)*sampl]))
        
    plt.plot(instane_flow_rand_robot)
    plt.show()
    print(accum_flow_rand_robot[-1])
    sum(accum_flow_rand_robot_multip_runs)/float(len(accum_flow_rand_robot_multip_runs))
#    plt.plot(accum_flow_rand_robot)
#    plt.show()

#
    init_pos_rx_list = [2.0] #[x*0.5 for x in range(1, 8)]
    init_pos_ry_list = [2.0] #[x*0.5 for x in range(1, 8)]
    pos_rx_stable_list = []
    pos_ry_stable_list = []
    converge_time_list = []
    average_accum_outflow_list = []
    for pos_rx_init in init_pos_rx_list:
        for pos_ry_init in init_pos_ry_list:

            accum_flow_DQN_robot_multip_runs = []
            pos_rx_stable_multip_runs = []
            pos_ry_stable_multip_runs = []
            converge_time_multip_runs = []
            for i in range(num_experiments):  
                print ("Robot with DQN:")
                instane_flow_DQN_robot = []
                accum_flow_DQN_robot = []      
                init_flowAB()
                omega_rx = 0.0
                
                pos_rx = pos_rx_init
                pos_ry = pos_ry_init
                pos_rx_list = []
                pos_ry_list = []
                
                start_time = time.time()
                for step in range(numstep+1):
                    if step == 0:
                        s,_=move_crowd(robot_flag = 1.0, render_flag = False, omega=omega_rx, numthstep=step, gener_flag=True)
                        a = sess.run(mainQN.predict,feed_dict={mainQN.observation:np.expand_dims(s, axis=0)})[0]
                        omega_rx = 0.0
                
                        move_direction = action_dic[str(a)]
                        if move_direction == 'UP':
                            pos_ry = pos_ry + robot_step_len
                        elif move_direction == 'DOWN':
                            pos_ry = pos_ry - robot_step_len
                        elif move_direction == 'LEFT':
                            pos_rx = pos_rx - robot_step_len
                        elif move_direction == 'RIGHT':
                            pos_rx = pos_rx + robot_step_len
                        elif move_direction == 'STOP':
                            pass                      
                        else: raise ValueError('unacceptable moving direction') 

                        if pos_rx > rx_max:
                            pos_rx = rx_max
                        elif pos_rx < rx_min:
                            pos_rx = rx_min
                            
                        if pos_ry > 7.5:
                            pos_ry = 7.5
                        elif pos_ry < ry_min:
                            pos_ry = ry_min                        
                            
                        pos_rx_list.append(pos_rx)
                        pos_ry_list.append(pos_ry)
                            
                    if step % sampl == 0 and step != 0:
                        s,_=move_crowd(robot_flag = 1.0, render_flag = False, omega=omega_rx, numthstep=step, gener_flag=True)
                        a = sess.run(mainQN.predict,feed_dict={mainQN.observation:np.expand_dims(s, axis=0)})[0]
                        
                        omega_rx = 0.0
                
                        move_direction = action_dic[str(a)]
                        if move_direction == 'UP':
                            pos_ry = pos_ry + robot_step_len
                        elif move_direction == 'DOWN':
                            pos_ry = pos_ry - robot_step_len
                        elif move_direction == 'LEFT':
                            pos_rx = pos_rx - robot_step_len
                        elif move_direction == 'RIGHT':
                            pos_rx = pos_rx + robot_step_len
                        elif move_direction == 'STOP':
                            pass                      
                        else: raise ValueError('unacceptable moving direction') 

                        if pos_rx > rx_max:
                            pos_rx = rx_max
                        elif pos_rx < rx_min:
                            pos_rx = rx_min
                            
                        if pos_ry > 7.5:
                            pos_ry = 7.5
                        elif pos_ry < ry_min:
                            pos_ry = ry_min
        
                        pos_rx_list.append(pos_rx)
                        pos_ry_list.append(pos_ry)
                            
                    if step % sampl != 0:           
                        move_crowd(robot_flag = 1.0, render_flag = False, omega=omega_rx, numthstep=step, gener_flag=False) #robot_flag < 0 , no robot            
                    
                    accum_flow_DQN_robot.append(np.sum(cout1))
                print ("end of Robot with DQN:")
                elapsed_time = time.time() - start_time
                #print(elapsed_time)
                accum_flow_DQN_robot_multip_runs.append(accum_flow_DQN_robot[-1])
                print "accum_flow_DQN_robot:"+ str(accum_flow_DQN_robot[-1])

            
#                for i in range(0,simulation_time):
#                    instane_flow_DQN_robot.append(np.sum(cout1[i*sampl:(i+1)*sampl]))

                plt.plot(pos_rx_list)
                plt.ylabel('Robot position on x axis')
                plt.xlabel('Time(s)')     
                plt.show()    
                plt.plot(pos_ry_list)
                plt.ylabel('Robot position on y axis')
                plt.xlabel('Time(s)')  
                plt.show() 
                pos_rx_stable = sum(pos_rx_list[-200:])/float(200)
                pos_ry_stable = sum(pos_ry_list[-200:])/float(200)
                print "pos_rx_stable:"+ str(pos_rx_stable)
                print "pos_ry_stable:"+ str(pos_ry_stable)
                pos_rx_stable_multip_runs.append(pos_rx_stable)
                pos_ry_stable_multip_runs.append(pos_ry_stable)
                
                if pos_rx_list[0] <= pos_rx_stable:
                    for idx, value in enumerate(pos_rx_list):
                        if value >= pos_rx_stable:
                            print idx
                            rx_idx = idx
                            break        
                else:
                    for idx, value in enumerate(pos_rx_list):
                        if value <= pos_rx_stable:
                            print idx
                            rx_idx = idx
                            break        
                    
                if pos_ry_list[0] <= pos_ry_stable:
                    for idx, value in enumerate(pos_ry_list):
                        if value >= pos_ry_stable:
                            print idx
                            ry_idx = idx
                            break        
                else:
                    for idx, value in enumerate(pos_ry_list):
                        if value <= pos_ry_stable:
                            print idx
                            ry_idx = idx
                            break        
                converge_time = max(rx_idx, ry_idx)
                print "converge_time:" + str(converge_time)  
                converge_time_multip_runs.append(converge_time)
               
#            plt.plot(instane_flow_DQN_robot)
#            plt.show()
            
            average_accum_outflow = sum(accum_flow_DQN_robot_multip_runs)/float(len(accum_flow_DQN_robot_multip_runs))
            print "average_accum_outflow:" + str(average_accum_outflow)
            average_accum_outflow_list.append(accum_flow_DQN_robot_multip_runs)
            
            average_pos_rx_stable = sum(pos_rx_stable_multip_runs)/float(len(pos_rx_stable_multip_runs))
            average_pos_ry_stable = sum(pos_ry_stable_multip_runs)/float(len(pos_ry_stable_multip_runs))
            print "average_pos_rx_stable:" + str(average_pos_rx_stable)
            print "average_pos_ry_stable:" + str(average_pos_ry_stable)            
            pos_rx_stable_list.append(pos_rx_stable_multip_runs)
            pos_ry_stable_list.append(pos_ry_stable_multip_runs)
            
            average_converge_time = sum(converge_time_multip_runs)/float(len(converge_time_multip_runs))
            print "average_converge_time:" + str(average_converge_time)
            converge_time_list.append(converge_time_multip_runs)

#            with open(os.path.join(model_path, 'converge_result.pkl'), 'w') as f:
#                pickle.dump([average_accum_outflow_list, pos_rx_stable_list, pos_ry_stable_list, converge_time_list], f)
                
    list_index = 0+6
    sum(average_accum_outflow_list[list_index])/float(len(average_accum_outflow_list[list_index]))
    sum(pos_rx_stable_list[list_index])/float(len(pos_rx_stable_list[list_index]))
    sum(pos_ry_stable_list[list_index])/float(len(pos_ry_stable_list[list_index]))
    sum(converge_time_list[list_index])/float(len(converge_time_list[list_index]))


    accum_flow_DQN_robot_multip_runs = []
    pos_rx_stable_multip_runs = []
    pos_ry_stable_multip_runs = []
    converge_time_multip_runs = []
    for i in range(num_experiments):  
        print ("Robot with DQN:")
        instane_flow_DQN_robot = []
        accum_flow_DQN_robot = []      
        init_flowAB()
        omega_rx = 0.0
        
        pos_rx = 0.5
        pos_ry = 0.5
        pos_rx_list = []
        pos_ry_list = []
        action_value_np = np.zeros((simulation_time, len(action_dic)))
        
        start_time = time.time()
        time_sec = 0
        for step in range(numstep+1):
            if step == 0:
                s,_=move_crowd(robot_flag = 1.0, render_flag = False, omega=omega_rx, numthstep=step, gener_flag=True)
#                action_value = sess.run(mainQN.Qout,feed_dict={mainQN.observation:np.expand_dims(s, axis=0)})
#                action_value_np[time_sec, :] = action_value
#                time_sec += 1
#                a = np.argmax(action_value) #
                a = sess.run(mainQN.predict,feed_dict={mainQN.observation:np.expand_dims(s, axis=0)})[0]
                omega_rx = 0.0
        
                move_direction = action_dic[str(a)]
                if move_direction == 'UP':
                    pos_ry = pos_ry + robot_step_len
                elif move_direction == 'DOWN':
                    pos_ry = pos_ry - robot_step_len
                elif move_direction == 'LEFT':
                    pos_rx = pos_rx - robot_step_len
                elif move_direction == 'RIGHT':
                    pos_rx = pos_rx + robot_step_len
                elif move_direction == 'STOP':
                    pass                      
                else: raise ValueError('unacceptable moving direction') 

                if pos_rx > rx_max:
                    pos_rx = rx_max
                elif pos_rx < rx_min:
                    pos_rx = rx_min
                    
                if pos_ry > 7.5:
                    pos_ry = 7.5
                elif pos_ry < ry_min:
                    pos_ry = ry_min                
                    
                pos_rx_list.append(pos_rx)
                pos_ry_list.append(pos_ry)
                    
            if step % sampl == 0 and step != 0:
                s,_=move_crowd(robot_flag = 1.0, render_flag = False, omega=omega_rx, numthstep=step, gener_flag=True)
#                action_value = sess.run(mainQN.Qout,feed_dict={mainQN.observation:np.expand_dims(s, axis=0)})
#                action_value_np[time_sec, :] = action_value
#                time_sec += 1
#                a = np.argmax(action_value) #
                a = sess.run(mainQN.predict,feed_dict={mainQN.observation:np.expand_dims(s, axis=0)})[0]
                
                omega_rx = 0.0
        
                move_direction = action_dic[str(a)]
                if move_direction == 'UP':
                    pos_ry = pos_ry + robot_step_len
                elif move_direction == 'DOWN':
                    pos_ry = pos_ry - robot_step_len
                elif move_direction == 'LEFT':
                    pos_rx = pos_rx - robot_step_len
                elif move_direction == 'RIGHT':
                    pos_rx = pos_rx + robot_step_len
                elif move_direction == 'STOP':
                    pass                      
                else: raise ValueError('unacceptable moving direction') 

                if pos_rx > rx_max:
                    pos_rx = rx_max
                elif pos_rx < rx_min:
                    pos_rx = rx_min
                    
                if pos_ry > 7.5:
                    pos_ry = 7.5
                elif pos_ry < ry_min:
                    pos_ry = ry_min                

                pos_rx_list.append(pos_rx)
                pos_ry_list.append(pos_ry)
                    
            if step % sampl != 0:           
                move_crowd(robot_flag = 1.0, render_flag = False, omega=omega_rx, numthstep=step, gener_flag=False) #robot_flag < 0 , no robot            
            
            accum_flow_DQN_robot.append(np.sum(cout1))
        print ("end of Robot with DQN:")
        elapsed_time = time.time() - start_time
        #print(elapsed_time)
        accum_flow_DQN_robot_multip_runs.append(accum_flow_DQN_robot[-1])
        print "accum_flow_DQN_robot:"+ str(accum_flow_DQN_robot[-1])

    
        for i in range(0,simulation_time):
            instane_flow_DQN_robot.append(np.sum(cout1[i*sampl:(i+1)*sampl]))
            
        plt.plot(instane_flow_DQN_robot)
        plt.show()
        plt.plot(accum_flow_DQN_robot)
        plt.show()        
        
        with open(os.path.join(model_path, 'outflow_rx3_5_ry0_5.pkl'), 'w') as f:  #outflow_7_5.pkl #outflow_rx2_ry2.pkl#outflow_rx3_5_ry0_5.pkl
            pickle.dump([instane_flow_DQN_robot, accum_flow_DQN_robot], f)        
#'../models/6-13_flowA_B_180_120/'#'../models/6-26_flowA_B_180_120_random_robot_position'#  

        with open(os.path.join('../models/6-13_flowA_B_180_120/', 'outflow_rx3_5_ry0_5.pkl'), 'r') as f:
            flow_random_robot_init = pickle.load(f)
                  
        
        case1 = plt.plot(instane_flow_DQN_robot, 'r', label='case1')
        case2 = plt.plot(flow_random_robot_init[0], 'b', label='case2')
        plt.ylabel('Instantaneous flow')
        plt.xlabel('Time(s)')          
        plt.show()        

        case1 = plt.plot(accum_flow_DQN_robot[::sampl], 'r', label='case1')
        case2 = plt.plot(flow_random_robot_init[1][::sampl], 'b', label='case.21')
#        plt.legend([case2, case1], ['case2', 'case1'])
        plt.ylabel('Accumulated flow')
        plt.xlabel('Time(s)')          
        plt.show() 
        
        plt.plot(pos_rx_list)
        plt.ylabel('Robot position on x axis')
        plt.xlabel('Time(s)')     
        plt.show()    
        plt.plot(pos_ry_list)
        plt.ylabel('Robot position on y axis')
        plt.xlabel('Time(s)')  
        plt.show() 
        pos_rx_stable = sum(pos_rx_list[-200:])/float(200)
        pos_ry_stable = sum(pos_ry_list[-200:])/float(200)
        print "pos_rx_stable:"+ str(pos_rx_stable)
        print "pos_ry_stable:"+ str(pos_ry_stable)
        pos_rx_stable_multip_runs.append(pos_rx_stable)
        pos_ry_stable_multip_runs.append(pos_ry_stable)
        
        if pos_rx_list[0] <= pos_rx_stable:
            for idx, value in enumerate(pos_rx_list):
                if value >= pos_rx_stable:
                    print idx
                    rx_idx = idx
                    break        
        else:
            for idx, value in enumerate(pos_rx_list):
                if value <= pos_rx_stable:
                    print idx
                    rx_idx = idx
                    break        
            
        if pos_ry_list[0] <= pos_ry_stable:
            for idx, value in enumerate(pos_ry_list):
                if value >= pos_ry_stable:
                    print idx
                    ry_idx = idx
                    break        
        else:
            for idx, value in enumerate(pos_ry_list):
                if value <= pos_ry_stable:
                    print idx
                    ry_idx = idx
                    break        
        converge_time = max(rx_idx, ry_idx)
        print "converge_time:" + str(converge_time)  
        converge_time_multip_runs.append(converge_time)    
            
#            plt.plot(pos_rx_list)
#            plt.ylabel('Robot position on x axis')
#            plt.xlabel('Time(s)')     
#            plt.show()    
#            plt.plot(pos_ry_list)
#            plt.ylabel('Robot position on y axis')
#            plt.xlabel('Time(s)')     
#            pos_rx_stable = sum(pos_rx_list[-200:])/float(200)
#            pos_ry_stable = sum(pos_ry_list[-200:])/float(200)
#            print "pos_rx_stable:"+ str(pos_rx_stable)
#            print "pos_ry_stable:"+ str(pos_ry_stable)
#            
#            pos_rx_stable_list.append(pos_rx_stable)
#            pos_ry_stable_list.append(pos_ry_stable)
            
        plt.plot(pos_rx_list, pos_ry_list)
        plt.xlim((0, 4))
        plt.ylabel('Robot position on y axis')
        plt.xlabel('Robot position on x axis')
        
#            if pos_rx_list[0] <= pos_rx_stable:
#                for idx, value in enumerate(pos_rx_list):
#                    if value >= pos_rx_stable:
#                        print idx
#                        rx_idx = idx
#                        break        
#            else:
#                for idx, value in enumerate(pos_rx_list):
#                    if value <= pos_rx_stable:
#                        print idx
#                        rx_idx = idx
#                        break        
#                
#            if pos_ry_list[0] <= pos_ry_stable:
#                for idx, value in enumerate(pos_ry_list):
#                    if value >= pos_ry_stable:
#                        print idx
#                        ry_idx = idx
#                        break        
#            else:
#                for idx, value in enumerate(pos_ry_list):
#                    if value <= pos_ry_stable:
#                        print idx
#                        ry_idx = idx
#                        break        
#            converge_time = max(rx_idx, ry_idx)
#            print "converge_time:" + str(converge_time)   
#            converge_time_list.append(converge_time)
    
    accum_flow_DQN_robot_multip_runs = []
    pos_rx_stable_multip_runs = []
    pos_ry_stable_multip_runs = []
    converge_time_multip_runs = []
    for i in range(num_experiments):  
        print ("Robot with DQN:")
        instane_flow_DQN_robot = []
        accum_flow_DQN_robot = []      
        init_flowAB()
        omega_rx = 0.0
        
        pos_rx = 0.5
        pos_ry = 0.5
        move_up = 0.0
        move_right = 0.0
        move_interval = 5
        change_update_time = 5000
        pos_rx_list = []
        pos_ry_list = []
        pos_rx_fake_list = []
        pos_ry_fake_list = []        
        ry_dir_list = []
        rx_dir_list = []
        action_value_np = np.zeros((simulation_time, len(action_dic)))
        
        start_time = time.time()
        time_sec = 0
        for step in range(numstep+1):
            if step == 0:
                s,_=move_crowd(robot_flag = 1.0, render_flag = True, omega=omega_rx, numthstep=step, gener_flag=True)
#                action_value = sess.run(mainQN.Qout,feed_dict={mainQN.observation:np.expand_dims(s, axis=0)})
#                action_value_np[time_sec, :] = action_value
#                time_sec += 1
#                a = np.argmax(action_value) #
                a = sess.run(mainQN.predict,feed_dict={mainQN.observation:np.expand_dims(s, axis=0)})[0]
                omega_rx = 0.0
        
                move_direction = action_dic[str(a)]
                if step < change_update_time*sampl:
                    if move_direction == 'UP':
                        pos_ry = pos_ry + robot_step_len
                        ry_dir_list.append(1)
                    elif move_direction == 'DOWN':
                        pos_ry = pos_ry - robot_step_len
                        ry_dir_list.append(-1)
                    elif move_direction == 'LEFT':
                        pos_rx = pos_rx - robot_step_len
                        rx_dir_list.append(1)
                    elif move_direction == 'RIGHT':
                        pos_rx = pos_rx + robot_step_len
                        rx_dir_list.append(-1)
                    elif move_direction == 'STOP':
                        pass                      
                    else: raise ValueError('unacceptable moving direction')                     
                    
                else:
                    if move_direction == 'UP':
                        move_up = move_up + robot_step_len
                    elif move_direction == 'DOWN':
                        move_up = move_up - robot_step_len
                    elif move_direction == 'LEFT':
                        move_right = move_right - robot_step_len
                    elif move_direction == 'RIGHT':
                        move_right = move_right + robot_step_len
                    elif move_direction == 'STOP':
                        pass                      
                    else: raise ValueError('unacceptable moving direction')                 
                                

                if pos_rx > rx_max:
                    pos_rx = rx_max
                elif pos_rx < rx_min:
                    pos_rx = rx_min
                    
                if pos_ry > 7.5:
                    pos_ry = 7.5
                elif pos_ry < ry_min:
                    pos_ry = ry_min                
                    
                pos_rx_list.append(pos_rx)
                pos_ry_list.append(pos_ry)
                pos_rx_fake_list.append(pos_rx)
                pos_ry_fake_list.append(pos_ry)
                    
            if step % sampl == 0 and step != 0:
                s,_=move_crowd(robot_flag = 1.0, render_flag = True, omega=omega_rx, numthstep=step, gener_flag=True)
#                action_value = sess.run(mainQN.Qout,feed_dict={mainQN.observation:np.expand_dims(s, axis=0)})
#                action_value_np[time_sec, :] = action_value
#                time_sec += 1
#                a = np.argmax(action_value) #
                a = sess.run(mainQN.predict,feed_dict={mainQN.observation:np.expand_dims(s, axis=0)})[0]
                
                omega_rx = 0.0
        
                move_direction = action_dic[str(a)]
                if step < change_update_time*sampl:
                    if move_direction == 'UP':
                        pos_ry = pos_ry + robot_step_len
                        ry_dir_list.append(1)
                        rx_dir_list.append(0)
                    elif move_direction == 'DOWN':
                        pos_ry = pos_ry - robot_step_len
                        ry_dir_list.append(-1)
                        rx_dir_list.append(0)
                    elif move_direction == 'LEFT':
                        pos_rx = pos_rx - robot_step_len
                        rx_dir_list.append(1)
                        ry_dir_list.append(0)
                    elif move_direction == 'RIGHT':
                        pos_rx = pos_rx + robot_step_len
                        rx_dir_list.append(-1)
                        ry_dir_list.append(0)
                    elif move_direction == 'STOP':
                        pass                      
                    else: raise ValueError('unacceptable moving direction')                                                                
                    
                else:                
                    if move_direction == 'UP':
                        move_up = move_up + robot_step_len
                    elif move_direction == 'DOWN':
                        move_up = move_up - robot_step_len
                    elif move_direction == 'LEFT':
                        move_right = move_right - robot_step_len
                    elif move_direction == 'RIGHT':
                        move_right = move_right + robot_step_len
                    elif move_direction == 'STOP':
                        pass                      
                    else: raise ValueError('unacceptable moving direction')   
                
                    if step % (sampl*move_interval) == 0:
                        pos_rx = pos_rx + move_right
                        pos_ry = pos_ry + move_up
                        move_up = 0.0
                        move_right = 0.0


                if pos_rx > rx_max:
                    pos_rx = rx_max
                elif pos_rx < rx_min:
                    pos_rx = rx_min
                    
                if pos_ry > 7.5:
                    pos_ry = 7.5
                elif pos_ry < ry_min:
                    pos_ry = ry_min                

                pos_rx_fake_list.append(pos_rx)
                pos_ry_fake_list.append(pos_ry)
                
                average_window_size = 10
                dir_change_thread = 2
                if step > 40*sampl:
                    if sum(rx_dir_list[step/sampl-average_window_size:step/sampl]) <= dir_change_thread:
                        pos_rx = sum(pos_rx_fake_list[step/sampl-average_window_size:step/sampl])/float(average_window_size)
                        pos_rx = round(pos_rx/robot_step_len)*robot_step_len
                    if sum(ry_dir_list[step/sampl-average_window_size:step/sampl]) <= dir_change_thread:
                        pos_ry = sum(pos_ry_fake_list[step/sampl-average_window_size:step/sampl])/float(average_window_size)
                        pos_ry = round(pos_ry/robot_step_len)*robot_step_len
                        
                pos_rx_list.append(pos_rx)
                pos_ry_list.append(pos_ry)
                    
            if step % sampl != 0:           
                move_crowd(robot_flag = 1.0, render_flag = False, omega=omega_rx, numthstep=step, gener_flag=False) #robot_flag < 0 , no robot            
            
            accum_flow_DQN_robot.append(np.sum(cout1))
        print ("end of Robot with DQN:")
        elapsed_time = time.time() - start_time
        #print(elapsed_time)
        accum_flow_DQN_robot_multip_runs.append(accum_flow_DQN_robot[-1])
        print "accum_flow_DQN_robot:"+ str(accum_flow_DQN_robot[-1])

    
        for i in range(0,simulation_time):
            instane_flow_DQN_robot.append(np.sum(cout1[i*sampl:(i+1)*sampl]))
            
        plt.plot(instane_flow_DQN_robot)
        plt.show()
        plt.plot(accum_flow_DQN_robot)
        plt.show()        
        
        with open(os.path.join(model_path, 'outflow_rx3_5_ry0_5.pkl'), 'w') as f:  #outflow_7_5.pkl #outflow_rx2_ry2.pkl#outflow_rx3_5_ry0_5.pkl
            pickle.dump([instane_flow_DQN_robot, accum_flow_DQN_robot], f)        
#'../models/6-13_flowA_B_180_120/'#'../models/6-26_flowA_B_180_120_random_robot_position'#  

        with open(os.path.join('../models/6-13_flowA_B_180_120/', 'outflow_rx3_5_ry0_5.pkl'), 'r') as f:
            flow_random_robot_init = pickle.load(f)
                  
        
        case1 = plt.plot(instane_flow_DQN_robot, 'r', label='case1')
        case2 = plt.plot(flow_random_robot_init[0], 'b', label='case2')
        plt.ylabel('Instantaneous flow')
        plt.xlabel('Time(s)')          
        plt.show()        

        case1 = plt.plot(accum_flow_DQN_robot[::sampl], 'r', label='case1')
        case2 = plt.plot(flow_random_robot_init[1][::sampl], 'b', label='case.21')
#        plt.legend([case2, case1], ['case2', 'case1'])
        plt.ylabel('Accumulated flow')
        plt.xlabel('Time(s)')          
        plt.show() 
        
        plt.plot(pos_rx_list)
        plt.ylabel('Robot position on x axis')
        plt.xlabel('Time(s)')     
        plt.show()    
        plt.plot(pos_ry_list)
        plt.ylabel('Robot position on y axis')
        plt.xlabel('Time(s)')  
        plt.show() 
        pos_rx_stable = sum(pos_rx_list[-200:])/float(200)
        pos_ry_stable = sum(pos_ry_list[-200:])/float(200)
        print "pos_rx_stable:"+ str(pos_rx_stable)
        print "pos_ry_stable:"+ str(pos_ry_stable)
        pos_rx_stable_multip_runs.append(pos_rx_stable)
        pos_ry_stable_multip_runs.append(pos_ry_stable)
        
        if pos_rx_list[0] <= pos_rx_stable:
            for idx, value in enumerate(pos_rx_list):
                if value >= pos_rx_stable:
                    print idx
                    rx_idx = idx
                    break        
        else:
            for idx, value in enumerate(pos_rx_list):
                if value <= pos_rx_stable:
                    print idx
                    rx_idx = idx
                    break        
            
        if pos_ry_list[0] <= pos_ry_stable:
            for idx, value in enumerate(pos_ry_list):
                if value >= pos_ry_stable:
                    print idx
                    ry_idx = idx
                    break        
        else:
            for idx, value in enumerate(pos_ry_list):
                if value <= pos_ry_stable:
                    print idx
                    ry_idx = idx
                    break        
        converge_time = max(rx_idx, ry_idx)
        print "converge_time:" + str(converge_time)  
        converge_time_multip_runs.append(converge_time)    


        plt.plot(pos_rx_list, pos_ry_list)
        plt.xlim((0, 4))
        plt.ylabel('Robot position on y axis')
        plt.xlabel('Robot position on x axis')


        
    window_size = 5
    
#    average_pos_rx_list = np.convolve(pos_rx_list, np.ones((window_size,))/window_size, mode='valid')
#    average_pos_ry_list = np.convolve(pos_ry_list, np.ones((window_size,))/window_size, mode='valid')
#    plt.plot(average_pos_rx_list)
#    plt.plot(average_pos_ry_list)
    
    average_instane_flow_no_robot = np.convolve(instane_flow_no_robot, np.ones((window_size,))/window_size, mode='valid')
    average_instane_flow_rand_robot = np.convolve(instane_flow_rand_robot, np.ones((window_size,))/window_size, mode='valid')
    average_instane_flow_DQN_robot = np.convolve(instane_flow_DQN_robot, np.ones((window_size,))/window_size, mode='valid')
    plt.plot(average_instane_flow_no_robot, 'k-', average_instane_flow_rand_robot, 'b-', average_instane_flow_DQN_robot, 'r-')
    plt.ylabel('Instantaneous flow')
    plt.xlabel('Time(s)')          
    plt.show()  
    
    #plt.plot(instane_flow_no_robot, 'k-', instane_flow_rand_robot, 'b-', instane_flow_DQN_robot, 'r-')
    plt.plot(accum_flow_no_robot[::sampl], 'k-', accum_flow_rand_robot[::sampl], 'b-', accum_flow_DQN_robot[::sampl], 'r-')
    plt.ylabel('Accumulated flow')
    plt.xlabel('Time(s)')          
    plt.show()    

    with open('../models/average_accumulated_flow.pkl', 'w') as f:
        pickle.dump([accum_flow_no_robot_multip_runs, accum_flow_rand_robot_multip_runs, accum_flow_DQN_robot_multip_runs], f)


#time_start = time.time()
#a = sess.run(mainQN.predict,feed_dict={mainQN.observation:np.expand_dims(s, axis=0)})[0]
#elapsed_time = time.time() - time_start
#print elapsed_time

#if PLOT_RESULT == True:
#    numstep = sampl*400
#    
#    print ("Without robot:")
#    instane_flow_no_robot = []
#    accum_flow_no_robot = []    
#    init_flowAB()    
#    for step in range(numstep+1):
#        i1,out_fl=move_crowd(robot_flag = -1.0, render_flag = False, omega=0, numthstep=step, gener_flag=True) #robot_flag < 0 , no robot
#        instane_flow_no_robot.append(out_fl)
#        accum_flow_no_robot.append(np.sum(cout1))
#
#    plt.plot(instane_flow_no_robot)
#    plt.show()
#    print(accum_flow_no_robot[-1])
##    plt.plot(accum_flow_no_robot)
##    plt.show()
#
#
#    print ("Robot is stil:")
#    instane_flow_stil_robot = []
#    accum_flow_stil_robot = []      
#    init_flowAB() 
#    for step in range(numstep+1):
#        i1,out_fl=move_crowd(robot_flag = 1.0, render_flag = False, omega=0, numthstep=step, gener_flag=True) #robot_flag < 0 , no robot
#        instane_flow_stil_robot.append(out_fl)
#        accum_flow_stil_robot.append(np.sum(cout1))
#        
#    plt.plot(instane_flow_stil_robot)
#    plt.show()
#    print(accum_flow_stil_robot[-1])
##    plt.plot(accum_flow_stil_robot)
##    plt.show()
#
#
#    print ("Robot takes random action:")
#    instane_flow_rand_robot = []
#    accum_flow_rand_robot = []      
#    init_flowAB() 
#    for step in range(numstep+1):
#        if step % sampl ==0:
#            a = np.random.randint(0, len(action_dic))
#
#            omega_rx = 0.0
#    
#            move_direction = action_dic[str(a)]
#            if move_direction == 'UP':
#                pos_ry = pos_ry + robot_step_len
#            elif move_direction == 'DOWN':
#                pos_ry = pos_ry - robot_step_len
#            elif move_direction == 'LEFT':
#                pos_rx = pos_rx - robot_step_len
#            elif move_direction == 'RIGHT':
#                pos_rx = pos_rx + robot_step_len
#            else: raise ValueError('unacceptable moving direction') 
#            
#            if pos_rx > 3.5:
#                pos_rx = 3.5
#            elif pos_rx < 0.5:
#                pos_rx = 0.5
#                
#            if pos_ry > 7.5:
#                pos_ry = 7.5
#            elif pos_ry < 0.5:
#                pos_ry = 0.5
#
#   
#        i1,out_fl=move_crowd(robot_flag = 1.0, render_flag = True, omega=omega_rx, numthstep=step, gener_flag=True) #robot_flag < 0 , no robot
#        instane_flow_rand_robot.append(out_fl)
#        accum_flow_rand_robot.append(np.sum(cout1))
#        
#    plt.plot(instane_flow_rand_robot)
#    plt.show()
#    print(accum_flow_rand_robot[-1])
##    plt.plot(accum_flow_rand_robot)
##    plt.show()
#
#    print ("Robot with DQN:")
#    instane_flow_DQN_robot = []
#    accum_flow_DQN_robot = []      
#    init_flowAB()
#    
#    for step in range(numstep+1):
#        if step == 0:
#            a = 0
#        if step % sampl == 0 and step != 0:
#            a = sess.run(mainQN.predict,feed_dict={mainQN.observation:np.expand_dims(s, axis=0)})[0]
#            
#            omega_rx = 0.0
#    
#            move_direction = action_dic[str(a)]
#            if move_direction == 'UP':
#                pos_ry = pos_ry + robot_step_len
#            elif move_direction == 'DOWN':
#                pos_ry = pos_ry - robot_step_len
#            elif move_direction == 'LEFT':
#                pos_rx = pos_rx - robot_step_len
#            elif move_direction == 'RIGHT':
#                pos_rx = pos_rx + robot_step_len
#            else: raise ValueError('unacceptable moving direction') 
#            
#            if pos_rx > 3.5:
#                pos_rx = 3.5
#            elif pos_rx < 0.5:
#                pos_rx = 0.5
#                
#            if pos_ry > 7.5:
#                pos_ry = 7.5
#            elif pos_ry < 0.5:
#                pos_ry = 0.5            
#                   
#        s,out_fl=move_crowd(robot_flag = 1.0, render_flag = True, omega=omega_rx, numthstep=step, gener_flag=True) #robot_flag < 0 , no robot
#        instane_flow_DQN_robot.append(out_fl)
#        accum_flow_DQN_robot.append(np.sum(cout1))
#        
#    plt.plot(instane_flow_DQN_robot)
#    plt.show()
#    print(accum_flow_DQN_robot[-1])





# -*- coding: utf-8 -*-
"""
Created on Thu Apr 23 14:46:39 2020

@author: coldhenry
"""
from matplotlib import pyplot as plt 
import gym_minigrid
import numpy as np
import gym
from utils import step, load_env, plot_env, draw_gif_from_seq, what, rotation, dir2num
import copy

MF = 0 # Move Forward
TL = 1 # Turn Left
TR = 2 # Turn Right
PK = 3 # Pickup Key
UD = 4 # Unlock Door

'''
The Coordinate System:
    (0,0): Top Left Corner
    (x,y): x-th column and y-th row
'''

#%% cost calculation
        
direcs = np.array([[1,0],[-1,0],[0,1],[0,-1]],dtype=int)
c_dict = {0:'MF', 1:'TL', 2:'TR',3:'PK',4:'UD'}
    
# ternimal cost
def terminal_cost(env,goal,FLAG):
    v_T = np.full((n,n,4),np.inf)
    for i in range(n):
        for j in range(n):
            for direc in range(4):
                if FLAG and what([i,j],env)=='Door':
                    v_T[i,j,direc] = 30000
                if what([i,j],env)=='Wall':
                    v_T[i,j,direc] = 30000
                else:
                    v_T[i,j,direc] = 200*((i-goal[0])**2+(j-goal[1])**2)
    return v_T


    # state cost
def stage_cost(env,goal,FLAG):    
    stg_cost = np.full((n, n, 4, 3), np.inf)   
    for i in range(n):
        for j in range(n):
            for direc in range(4):
                for u in range(3):
                    if FLAG and what([i,j],env)=='Door':
                        stg_cost[i,j,:,:] = 20000
                        continue
                    if what([i,j],env)=='Wall':
                        stg_cost[i,j,:,:] = 20000
                        continue
                    # Move Forward
                    if u == 0:
                        # position of next step
                        pos = np.array([i,j])+direcs[direc]
                        cost_fw = 1
                        if FLAG and what([i,j],env)=='Door':
                            cost_dis = 10000
                        if what([pos[0],pos[1]],env)=='Wall':
                            cost_dis = 10000
                        else:
                            # distance b/t next step and the goal
                            cost_dis = (pos[0]-goal[0])**2 + (pos[1]-goal[1])**2
                        stg_cost[i,j,direc,u] = cost_fw + cost_dis
                    
                    # Turn Left
                    if u == 1:
                        # the new direction after rotation
                        angle = (rotation(-90)@np.transpose(direcs[direc])).astype(int)
                        # position that rotate and move one step forward
                        pos_new = np.array([i,j])+angle
                        
                        # default cost
                        cost_rt = 5
                        # distance b/t original pos
                        cost_tr_dis = (i-goal[0])**2 + (j-goal[1])**2
                        if FLAG and what([pos_new[0],pos_new[1]],env)=='Door':
                            cost_dis = 10000
                        if what([pos_new[0],pos_new[1]],env)=='Wall':
                            cost_fw_dis = 10000
                        else:
                            # distance b/t the next step after a turn
                            cost_fw_dis = ((pos_new[0]-goal[0])**2 + (pos_new[1]-goal[1])**2)
                        stg_cost[i,j,direc,u] = cost_rt + cost_tr_dis + cost_fw_dis
                                            
                        
                    # Turn Right
                    if u == 2:
                        # the new direction after rotation
                        angle = (rotation(90)@np.transpose(direcs[direc])).astype(int)
                        # position that rotate and move one step forward
                        pos_new = np.array([i,j])+angle
                        cost_rt = 5
                        cost_tr_dis = (i-goal[0])**2 + (j-goal[1])**2
                        if FLAG and what([pos_new[0],pos_new[1]],env)=='Door':
                            cost_dis = 10000
                        if what([pos_new[0],pos_new[1]],env)=='Wall':
                            cost_fw_dis = 10000
                        else:
                            # distance b/t the next step after a turn
                            cost_fw_dis = ((pos_new[0]-goal[0])**2 + (pos_new[1]-goal[1])**2)
                        stg_cost[i,j,direc,u] = cost_rt + cost_tr_dis+ cost_fw_dis
                    #print("tr cost:", stg_cost[i,j,direc,u])
    return stg_cost


#%% Dynamic Programming

key_s1_,key_s2_,key_s3_,key_s4_ = [],[],[],[]
goal_s1_,goal_s2_,goal_s3_ = [],[],[]
door_s1_ = []

def update(v_T, stg_cost, T):
    v_t = np.full((n,n,4),np.inf)
    policy = np.empty([n,n,4,T])
    
    for t in range(T-1,-1,-1):
        #print(t)
        Q_t = np.full((n, n, 4, 3), np.inf)
        # use terminal cost for the first step
        if t == T-1:
            v = copy.deepcopy(v_T)
        else:
            v = copy.deepcopy(v_t)
            
        # calculate total cost                      
        for i in range(n):
            for j in range(n):
                for direc in range(4):
                    for u in range(3):  
                        if u == 0:
                            pos = np.array([i,j])+direcs[direc] 
                            if pos[0]<n and pos[0]>=0 and pos[1]<n and pos[1]>=0:
                                v_val = v[pos[0],pos[1],direc]
                            else:
                                v_val = 10000
                        if u == 1:
                            angle = (rotation(-90)@np.transpose(direcs[direc])).astype(int)
                            v_val = v[i,j,dir2num(angle)]
                        if u == 2:
                            angle = (rotation(90)@np.transpose(direcs[direc])).astype(int)
                            v_val = v[i,j,dir2num(angle)]
                            
                        # given a state, stage cost + the cost of its next step
                        Q_t[i,j,direc,u] = stg_cost[i,j,direc,u] + v_val
                                  
        #find optimal cost of each states                
        for i in range(n):
            for j in range(n):
                for direc in range(4):                    
                    # find the index of the minima
                    motion = np.where(Q_t[i,j,direc,:]==np.min(Q_t[i,j,direc,:]))[0][0]                       
                    opt_cost = Q_t[i,j,direc,motion]
                    v_t[i,j,direc] = opt_cost
                    
                    policy[i,j,direc,t] = motion
                    policy = policy.astype(int)
        
        # report collect
        key_s1_.append(v_t[2,1,1])
        key_s2_.append(v_t[1,2,2])
        #key_s3_.append(v_t[1,2,2])
        #key_s4_.append(v_t[2,6,3])
        goal_s1_.append(v_t[3,1,0])
        #goal_s2_.append(v_t[4,3,2])
        door_s1_.append(v_t[2,1,0])
         
        if (v_t==v).all():
            break
        
    return v_t, policy
    
#%% record and control

def visualize(seq, env, env_path, v_t, policy, T, task, goal):
    for t in range(T):
        # basic info
        pos = env.agent_pos
        vec = dir2num(env.dir_vec)
        front_cell = env.front_pos # Get the cell in front of the agent
        print("time:{t}, state: {p},{v}".format(t=t, p=pos, v=vec))
        if task=='Key':
            # the key is in front of the agent
            if what(front_cell, env) == 'Key':               
                seq.append(PK)
                cost, done = step(env, PK)
                print("----get Key----")
                return seq
            
        if task=='Door':
            # the door is in front of the agent
            if what(front_cell, env) == 'Door':
                seq.append(UD)
                cost, done = step(env, UD)
                seq.append(MF)   
                cost, done = step(env, MF)
                print("----unlock the door----")
                return seq
        
        # read the corresponding policy given current position
        control = policy[pos[0],pos[1],vec,t] 
        print("next move: {c}".format(c=c_dict[control]))       
        seq.append(control)
        cost, done = step(env, control)
        
        if done:
            print("Reached Goal")
            return seq
            
        
#%% mode 
    
def dynamic_programming(seq, env, env_path, goal, T, task, FLAG):
    v_T = terminal_cost(env,goal, FLAG)
    stg_cost = stage_cost(env,goal, FLAG)
    v_t, policy = update(v_T, stg_cost, T)
    seq = visualize(seq, env, env_path, v_t, policy, T, task, goal)
    return  v_t, policy, seq


def key_door_path(env, env_path, goal_key, goal_door, goal_final, FLAG):
    print("====Key Door Method====")
    seq = []
    v_t_key, policy_key, seq = dynamic_programming(seq, env, env_path, goal_key, 3, 'Key', FLAG)
    v_t_door, policy_door, seq = dynamic_programming(seq, env, env_path, goal_door, 3, 'Door', FLAG)
    v_t_final, policy_final, seq = dynamic_programming(seq, env, env_path, goal_final, 3, 'Goal', FLAG)
    total_cost = len(seq)
    return  seq, total_cost, policy_final, v_t_final
    

def direct_path(env, env_path, goal_final, FLAG):
    print("====Direct Path Method====")
    seq = []
    v_t_final, policy_final, seq = dynamic_programming(seq, env, env_path, goal_final, 5, 'Goal', FLAG)
    if seq != None:
        total_cost = len(seq)
    else:
        total_cost = 100
    return seq, total_cost, policy_final, v_t_final
    
#%% main 
# size of map
n = 6    
def main():
    
    #def main():
        
    print('========== Map Information =========== ')
    mapname = 'doorkey-6x6-shortcut'
    env_path = './envs/'+ mapname +'.env'
    print("Map Name: {n}".format(n=env_path))
    env, info = load_env(env_path) # load an environment
    
    # Visualize the environment
    plot_env(env) 
    
    print('<Environment Info>\n')
    print(info) # Map size
                # agent initial position & direction, 
                # key position, door position, goal position
    print('======================================\n')            
    
    goal_key = np.array(info['key_pos']);
    goal_door = np.array(info['door_pos']);
    goal_final = np.array(info['goal_pos']);
    
    # flag: close the door or not
    # 0 for key door path 
    # 1 for direct path
    # the path and cost using the key and door
    FLAG = 0
    seq_kd, cost_kd, policy_kd, v_t_kd = key_door_path(env, env_path, goal_key, goal_door, goal_final, FLAG)
    
    
    # the path and cost using the direct path
    FLAG = 1
    # reset the map
    env, info = load_env(env_path)
    seq_dr, cost_dr, policy_dr, v_t_dr = direct_path(env, env_path, goal_final, FLAG)
    
    if cost_kd < cost_dr:
        draw_gif_from_seq(seq_kd, load_env(env_path)[0], path='./gif/'+ mapname +'.gif')
    else:
        draw_gif_from_seq(seq_dr, load_env(env_path)[0], path='./gif/'+ mapname +'.gif')
    
    
    
if __name__ == '__main__':
    main()




    

#!/usr/bin/env python3
from __future__ import print_function, division, absolute_import, unicode_literals
from matplotlib import pyplot as plt
from urdf_optcontrol import optimizer
import rospy
from std_msgs.msg import Float64
import numpy as np
from casadi import sin, cos, fmax
from std_srvs.srv import Empty
import time
import pickle
import pathlib

ros = True
new_h = False
new_j = False

filepath = pathlib.Path(__file__).parent

if ros: 
    # INITIALIZING NODE AND PUBLISHERS
    rospy.init_node("softleg_control")
    pub_ankle = rospy.Publisher('/oneleg/ankle/theta_command', Float64, queue_size=10)
    pub_knee = rospy.Publisher('/oneleg/knee/theta_command', Float64, queue_size=10)
    pub_hip = rospy.Publisher('/oneleg/hip/theta_command', Float64, queue_size=10)

# INITIALIZING GAZEBO SERVICES TO STOP AND UNSTOP
pause_gazebo=rospy.ServiceProxy('/gazebo/pause_physics',Empty)
unpause_gazebo=rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
restart_gazebo=rospy.ServiceProxy('/gazebo/reset_simulation',Empty)

# DAMPING VALUES WROTE INSIDE GAZEBO'S PLUGIN
sea_damping = dict(leg_J1=0.05, leg_J2=0.05, leg_J3=0.05)

# ROBOT SPECIFICATIONS
urdf_path = '/home/andrea/rob_project/softleg_jump/src/example/template_description/urdf/softleg-demo.urdf'
root = 'foot'
end = 'body'

# POSITION AND VELOCITY OF THE CENTER OF THE MASS OF THE ROBOT
def v_y(q, qd):
    return -0.1209*qd[0]*sin(q[0])-0.0488*(qd[0]+qd[1])*sin(q[0]+q[1])-0.0291*(qd[0]+qd[1]+qd[2])*sin(q[0]+q[1]+q[2])
def v_x(q, qd):
    return -0.1209*qd[0]*cos(q[0])-0.0488*(qd[0]+qd[1])*cos(q[0]+q[1])-0.0291*(qd[0]+qd[1]+qd[2])*cos(q[0]+q[1]+q[2])
def cm_x(q):
    return -0.1209*sin(q[0])-0.0488*sin(q[0]+q[1])-0.0291*sin(q[0]+q[1]+q[2])
    
def a_y(q, qd, qdd):
    return -0.0488*(qd[0]+qd[1])**2*cos(q[0]+q[1]) -0.0488*(qdd[0]+qdd[1])*sin(q[0]+q[1]) +     -0.0291*(qd[0]+qd[1]+qd[2])**2*cos(q[0]+q[1]+q[2]) + -0.0291*(qdd[0]+qdd[1]+qdd[2])**2*sin(q[0]+q[1]+q[2]) +     -0.1209*sin(q[0])*qdd[0] -0.1209*cos(q[0])*(qd[0])**2 
    
# INITIAL POSE AND VELOCITY
# in_cond = [0.35,-2.4765,2.1265, 0, 0, 0]
in_cond = [0.35,-2.1745,2.0245,0,0,0]
# in_cond = [0.30,-1.603,1.703,0,0,0]

# HOMING PROCEDURE
if any(in_cond) and ros:

    # HOMING PHASE SETTINGS
    steps=50
    time_horizon=1

    # HOMING REQUIREMENTS
    target = lambda t : [0]*3
    cost_func = lambda q,qd,u,t: (u-q).T@(u-q)
    constr1 = lambda q,qd,u,ee,qdd: (-0.1,cm_x(q),0.1)
    final_constr1 = lambda q,qd,u,ee,qdd : (in_cond[0:3], u, in_cond[0:3])
    final_constr2 = lambda q,qd,u,ee,qdd : (in_cond[0:3], q, in_cond[0:3])

    if new_h:
        optimizer.load_robot(urdf_path, root, end, sea_damping=sea_damping)
        res = optimizer.load_problem(
            cost_func,
            steps,
            [0]*6,
            target,
            time_horizon = time_horizon,
            max_iter=600,
            my_constraint=[constr1],
            my_final_constraint=[final_constr1, final_constr2]
            )
        with open(filepath.joinpath('homing.input'),"wb") as file:
            pickle.dump(res, file)
    else:
        with open(filepath.joinpath('homing.input'),"rb") as file:
            res = pickle.load(file)
        

    #SAVING HOMING PHASE RESULTS
    u_ankle_home = np.array(res['u'][0])
    u_knee_home = np.array(res['u'][1])
    u_hip_home = np.array(res['u'][2])
    rate_home = rospy.Rate(steps/time_horizon)





# JUMP SETTINGS

def trajectory_target_(t):
    return [0, 0, 0]

def my_cost_func(q, qd, u, t):
    return -v_y(q,qd) # qd.T@qd  # 30*q.T@q # + 0.2*(u-q).T@(u-q) + 0.1*qd.T@qd

def my_final_term_cost(q_f, qd_f, u_f):
    vy = v_y(q_f,qd_f)
    return -10*vy # + 70*qd_f@qd_f


steps = 50
time_horizon = 0.05

def my_constraint1(q, q_dot, u, ee_pos, q_ddot):
    return [-30, 0, -30], q_dot, [0, 30, 0]
def my_constraint2(q, q_dot, u, ee_pos, q_ddot):
    return [-2]*3, q, [2]*3
def my_constraint3(q, q_dot, u, ee_pos, q_ddot):
    return [-0.1]*3, u-q, [0.1]*3
def my_constraint4(q, q_dot, u, ee_pos, q_ddot):
    return [-0.005], a_y(q, q_dot, q_ddot), [10000]
my_constraints=[my_constraint1, my_constraint2, my_constraint3, my_constraint4]

def my_final_constraint1(q, q_dot, u, ee_pos, q_ddot):
    return [-0.0], v_x(q,q_dot), [0.0]
def my_final_constraint2(q, q_dot, u, ee_pos, q_ddot):
    return [-0.0], cm_x(q), [0.0]
def my_final_constraint3(q, q_dot, u, ee_pos, q_ddot):
    # return [-0.1], q[0]+q[1]+q[2], [0.1]
    return [-0.05]*3, q, [0.05]*3
my_final_constraints = [my_final_constraint1, my_final_constraint2, my_final_constraint3]  

if new_j:
    optimizer.load_robot(urdf_path, root, end, sea_damping=sea_damping)
    res = optimizer.load_problem(
        my_cost_func,
        steps,
        in_cond,
        trajectory_target_,
        time_horizon = time_horizon,
        final_term_cost=my_final_term_cost, 
        max_iter=600,
        my_constraint=my_constraints,
        my_final_constraint=my_final_constraints
        )
    with open(filepath.joinpath('jump.input'),"wb") as file:
        pickle.dump(res, file)
else:
    with open(filepath.joinpath('jump.input'),"rb") as file:
        res = pickle.load(file)

# Print the results!
if new_j:
    fig = optimizer.show_result()
    plt.show()


q = (res['q'][0][-1], res['q'][1][-1], res['q'][2][-1])
qd = (res['qd'][0][-1], res['qd'][1][-1], res['qd'][2][-1])

print('\nFinal v_y: ', v_y(q,qd), 'm/s')
print('Final v_x: ', v_x(q,qd), 'm/s')
print('Final cm_x: ', cm_x(q)*100, 'cm')


u_ankle = np.array(res['u'][0])
u_knee = np.array(res['u'][1])
u_hip = np.array(res['u'][2])


if ros:
    for i in range(5):
        pub_ankle.publish(0)
        pub_knee.publish(0)
        pub_hip.publish(0)
        time.sleep(0.1)
    restart_gazebo()
    unpause_gazebo()
    input('Press any key to start the Homing procedure')
    # EXECUTING THE HOMING PROCEDURE
    for v1, v2, v3 in zip(u_ankle_home, u_knee_home, u_hip_home):
        pub_ankle.publish(float(v1))
        pub_knee.publish(float(v2))
        pub_hip.publish(float(v3))
        rate_home.sleep()
    # JUMPING PROCEDURE
    time.sleep(1.0)
    # pause_gazebo()
    rate = rospy.Rate(steps/res['T'])
    for v1, v2, v3 in zip(u_ankle, u_knee, u_hip):
        pub_ankle.publish(float(v1))
        pub_knee.publish(float(v2))
        pub_hip.publish(float(v3))

        rate.sleep()
    # pause_gazebo()

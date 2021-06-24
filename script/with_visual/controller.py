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
import pathlib
from jump import homing, jumping, rehoming

def publish_inputs(u, publishers):
    for ankle, knee, hip in zip(u['ankle'], u['knee'], u['hip']):
        publishers[0].publish(float(ankle))
        publishers[1].publish(float(knee))
        publishers[2].publish(float(hip))
        rospy.rate(u['rate']).sleep()

def reset_simulation(publishers, restart_function):
    for i in range(3):
        publishers[0].publish(0)
        publishers[1].publish(0)
        publishers[2].publish(0)
        time.sleep(0.1)
    restart_function()
    time.sleep(0.2)

ros = False
recalc_homing = False
recalc_jump = True
recalc_rehoming = False

filepath = pathlib.Path(__file__).parent

if ros: 
    # INITIALIZING NODE AND PUBLISHERS
    rospy.init_node("softleg_control")
    pub_ankle = rospy.Publisher('/oneleg/ankle/theta_command', Float64, queue_size=10)
    pub_knee = rospy.Publisher('/oneleg/knee/theta_command', Float64, queue_size=10)
    pub_hip = rospy.Publisher('/oneleg/hip/theta_command', Float64, queue_size=10)
    publishers = [pub_ankle, pub_knee, pub_hip]

# INITIALIZING GAZEBO SERVICES TO STOP AND UNSTOP
pause_gazebo=rospy.ServiceProxy('/gazebo/pause_physics',Empty)
unpause_gazebo=rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
restart_gazebo=rospy.ServiceProxy('/gazebo/reset_simulation',Empty)


# ROBOT SPECIFICATIONS
robot = {
    'urdf_path': pathlib.Path(__file__).absolute().parent.parent.parent.joinpath('urdf','softleg-light.urdf'),
    'root': 'foot',
    'end': 'body',
    'sea_damping': dict(leg_J1=0.05, leg_J2=0.05, leg_J3=0.05)
}

# POSITION AND VELOCITY OF THE CENTER OF THE MASS OF THE ROBOT    
CoM = {
    'pos_y': lambda q: 0.1209*cos(q[0])+0.0488*cos(q[0]+q[1])+0.0291*cos(q[0]+q[1]+q[2]),
    'pos_x': lambda q: -0.1209*sin(q[0])-0.0488*sin(q[0]+q[1])-0.0291*sin(q[0]+q[1]+q[2]),
    'vel_y': lambda q,qd: -0.1209*qd[0]*sin(q[0])-0.0488*(qd[0]+qd[1])*sin(q[0]+q[1])-0.0291*(qd[0]+qd[1]+qd[2])*sin(q[0]+q[1]+q[2]),
    'vel_x': lambda q,qd: -0.1209*qd[0]*cos(q[0])-0.0488*(qd[0]+qd[1])*cos(q[0]+q[1])-0.0291*(qd[0]+qd[1]+qd[2])*cos(q[0]+q[1]+q[2]),
    'acc_y': lambda q,qd,qdd: -0.0488*(qd[0]+qd[1])**2*cos(q[0]+q[1]) -0.0488*(qdd[0]+qdd[1])*sin(q[0]+q[1]) -0.0291*(qd[0]+qd[1]+qd[2])**2*cos(q[0]+q[1]+q[2]) -0.0291*(qdd[0]+qdd[1]+qdd[2])**2*sin(q[0]+q[1]+q[2]) -0.1209*sin(q[0])*qdd[0] -0.1209*cos(q[0])*(qd[0])**2,
    'acc_x': lambda q,qd,qdd: 0.0488*(qd[0]+qd[1])**2*sin(q[0]+q[1]) -0.0488*(qdd[0]+qdd[1])*cos(q[0]+q[1]) +0.0291*(qd[0]+qd[1]+qd[2])**2*sin(q[0]+q[1]+q[2]) -0.0291*(qdd[0]+qdd[1]+qdd[2])**2*cos(q[0]+q[1]+q[2]) +0.1209*sin(q[0])*(qd[0])**2 -0.1209*cos(q[0])*qdd[0]
}
def p_zero_mom(q,qd,qdd,m=3.2,g=9.81):
    F = m*(g+CoM['acc_y'](q,qd,qdd))
    return (F*CoM['pos_x'](q)-m*CoM['acc_y'](q,qd,qdd)*CoM['pos_y'](q))/F
CoM['p_zero_mom'] = p_zero_mom

# INITIAL POSE AND VELOCITY
squat_position = [0.35,-2.1745,2.0245]

# HOMING PROCEDURE
u_home = homing(final_cond=squat_position,
                recalc=recalc_homing,
                center_of_mass=CoM,
                robot=robot
)
# JUMP PROCEDURE
u_jump = jumping(   initial_cond=squat_position,
                    recalc=recalc_jump,
                    center_of_mass=CoM,
                    robot=robot
)

qf = [u_jump['q'][0][-1], u_jump['q'][1][-1], u_jump['q'][2][-1]]
qdf = [u_jump['qd'][0][-1], u_jump['qd'][1][-1], u_jump['qd'][2][-1]]

# RE-HOMING PROCEDURE
initial_cond = [float(x) for x in qf+qdf]

u_rehome = rehoming(initial_cond=initial_cond,
                    final_cond=squat_position,
                    recalc=recalc_rehoming,
                    center_of_mass=CoM,
                    robot=robot
)

print('\nFinal v_y: ', CoM['vel_y'](qf,qdf), 'm/s')
print('Final v_x: ', CoM['vel_x'](qf,qdf), 'm/s')
print('Final pos_x: ', CoM['pos_x'](qf)*100, 'cm')

if ros:
    reset_simulation(publishers, restart_gazebo)
    input('Press Enter to start')
    publish_inputs(u_home, publishers)
    time.sleep(1.0)
    for i in range(100):

        publish_inputs(u_jump, publishers)
        time.sleep(0.05)

        publish_inputs(u_rehome, publishers)
        time.sleep(1.0)


plt.show()

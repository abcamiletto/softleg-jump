from urdf_optcontrol import optimizer
import rospy
import numpy as np
import pickle
import pathlib

store_path = pathlib.Path(__file__).parent.parent.joinpath('stored_results','homing.input')

def homing(final_cond, recalc, center_of_mass, robot):

    # HOMING PHASE SETTINGS
    steps=50
    time_horizon=1

    # HOMING REQUIREMENTS
    target = lambda t : [0]*3
    cost_func = lambda q,qd,u,t: (u-q).T@(u-q)
    constr1 = lambda q,qd,u,ee,qdd: (-0.1,center_of_mass['pos_x'](q),0.1)
    final_constr1 = lambda q,qd,u,ee,qdd : (final_cond, u, final_cond)
    final_constr2 = lambda q,qd,u,ee,qdd : (final_cond, q, final_cond)

    if recalc:
        optimizer.load_robot(robot['urdf_path'], robot['root'], robot['end'], sea_damping=robot['sea_damping'])
        res = optimizer.load_problem(
            cost_func = cost_func,
            control_steps = steps,
            initial_cond = [0]*6,
            trajectory_target = target,
            time_horizon = time_horizon,
            max_iter = 600,
            my_constraint = [constr1],
            my_final_constraint = [final_constr1, final_constr2]
            )
        fig = optimizer.show_result()

        u_home = {
            'ankle': np.array(res['u'][0]),
            'knee': np.array(res['u'][1]),
            'hip': np.array(res['u'][2]),
            'rate': rospy.Rate(steps/time_horizon),
            'plots': fig,
            'q': res['q'],
            'qd': res['qd']
        }


        with open(store_path,"wb") as file:
            pickle.dump(u_home, file)
    else:
        with open(store_path,"rb") as file:
            u_home = pickle.load(file)

    return u_home


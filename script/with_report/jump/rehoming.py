from urdf_optcontrol import optimizer
import rospy
import numpy as np
import pickle
import pathlib

store_path = pathlib.Path(__file__).parent.parent.joinpath('stored_results','rehoming.input')


def rehoming(initial_cond, final_cond, recalc, center_of_mass, robot):

    # HOMING PHASE SETTINGS
    steps=40
    time_horizon=1.0

    # HOMING REQUIREMENTS
    target = lambda t : [0]*3
    cost_func = lambda q,qd,u,t: qd.T@qd
    constr1 = lambda q,qd,u,ee,qdd: (-0.05, center_of_mass['pos_x'](q),0.05)
    constr2 = lambda q, qd, u, ee, qdd: ([-1,-1,-1], u-q, [1,1,1])
    # final_constr1 = lambda q,qd,u,ee,qdd : (final_cond, u, final_cond)
    final_constr2 = lambda q,qd,u,ee,qdd : (final_cond, q, final_cond)

    if recalc:
        optimizer.load_robot(robot['urdf_path'], robot['root'], robot['end'], sea_damping=robot['sea_damping'])
        res = optimizer.load_problem(
            cost_func = cost_func,
            control_steps = steps,
            initial_cond = initial_cond,
            trajectory_target = target,
            time_horizon = time_horizon,
            max_iter=600,
            my_constraint=[constr1, constr2],
            my_final_constraint=[final_constr2]
            )

        #SAVING REHOME PHASE RESULTS
        u_rehome = {
            'ankle': np.array(res['u'][0]),
            'knee': np.array(res['u'][1]),
            'hip': np.array(res['u'][2]),
            'rate': steps/time_horizon,
            'q': res['q'],
            'qd': res['qd']
        }

        with open(store_path,"wb") as file:
            pickle.dump(u_rehome, file)
    else:
        with open(store_path,"rb") as file:
            u_rehome = pickle.load(file)

    return u_rehome

from urdf_optcontrol import optimizer
import rospy
import numpy as np
import pickle
import pathlib

store_path = pathlib.Path(__file__).parent.parent.joinpath('stored_results','rehoming.input')


def rehoming(initial_cond, final_cond, recalc, center_of_mass, robot):

    # HOMING PHASE SETTINGS
    steps=40
    time_horizon=0.75

    # HOMING REQUIREMENTS
    target = lambda t : [0]*3
    cost_func = lambda q,qd,u,t: qd.T@qd

    constrs = []
    constrs.append(lambda q, qd, u, ee, qdd: ([-3.14,-3.14,-3.14], q, [3.14,3.14,3.14]))
    constrs.append(lambda q, qd, u, ee, qdd: ([-3.14,-3.14,-3.14], u, [3.14,3.14,3.14]))
    constrs.append(lambda q, qd, u, ee, qdd: ([-0.8], center_of_mass['pos_x'](q), [0.8]))

    final_constrs = []
    final_constrs.append(lambda q,qd,u,ee,qdd : (final_cond, u, final_cond))
    final_constrs.append(lambda q,qd,u,ee,qdd : (final_cond, q, final_cond))

    if recalc:
        optimizer.load_robot(robot['urdf_path'], robot['root'], robot['end'], sea_damping=robot['sea_damping'])
        res = optimizer.load_problem(
            cost_func = cost_func,
            control_steps = steps,
            initial_cond = initial_cond,
            trajectory_target = target,
            time_horizon = time_horizon,
            max_iter=600,
            my_constraint=constrs,
            my_final_constraint=final_constrs
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

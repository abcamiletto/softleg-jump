from urdf_optcontrol import optimizer
import rospy
import numpy as np
import pickle
import pathlib

store_path = pathlib.Path(__file__).parent.parent.joinpath('stored_results','jump.input')

def jumping(initial_cond, recalc, center_of_mass, robot):
    initial_cond = initial_cond + [0,0,0]

    # JUMP SETTINGS
    trajectory_target_= lambda t: [0, 0, 0]
    cost_func = lambda q,qd,u,t: -center_of_mass['vel_y'](q,qd)
    final_cost = lambda qf,qdf,uf: -10*center_of_mass['vel_y'](qf,qdf)

    steps = 50
    time_horizon = 0.05

    constraint1 = lambda q, qd, u, ee, qdd: ([-30,0,-30], qd, [0,30,0])
    constraint2 = lambda q, qd, u, ee, qdd: ([-2,-2,-2], q, [2,2,2])
    constraint3 = lambda q, qd, u, ee, qdd: ([-0.1]*3, u-q, [0.1]*3)
    constraint4 = lambda q, qd, u, ee, qdd: ([-0.005], center_of_mass['acc_y'](q, qd, qdd), [10000])
    my_constraints=[constraint1, constraint2, constraint3, constraint4]

    final_constraint1 = lambda q, qd, u, ee, qdd: ([-0.0], center_of_mass['vel_x'](q,qd), [0.0])
    final_constraint2 = lambda q, qd, u, ee, qdd: ([-0.0], center_of_mass['pos_x'](q), [0.0])
    final_constraint3 = lambda q, qd, u, ee, qdd: ([-0.05]*3, q, [0.05]*3)
    my_final_constraints = [final_constraint1, final_constraint2, final_constraint3]  

    if recalc:
        optimizer.load_robot(robot['urdf_path'], robot['root'], robot['end'], sea_damping=robot['sea_damping'])
        res = optimizer.load_problem(
            cost_func=cost_func,
            control_steps=steps,
            initial_cond=initial_cond,
            trajectory_target=trajectory_target_,
            time_horizon = time_horizon,
            final_term_cost=final_cost, 
            max_iter=600,
            my_constraint=my_constraints,
            my_final_constraint=my_final_constraints
            )
        fig = optimizer.show_result()

        #SAVING JUMP PHASE RESULTS
        u_jump = {
            'ankle': np.array(res['u'][0]),
            'knee': np.array(res['u'][1]),
            'hip': np.array(res['u'][2]),
            'rate': rospy.Rate(steps/time_horizon),
            'plots': fig,
            'q': res['q'],
            'qd': res['qd']
        }

        with open(store_path,"wb") as file:
            pickle.dump(u_jump, file)
    else:
        with open(store_path,"rb") as file:
            u_jump = pickle.load(file)


    return u_jump
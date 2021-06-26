from urdf_optcontrol import optimizer
import rospy
import numpy as np
import pickle
import pathlib
from jump.visual import visualize


store_path = pathlib.Path(__file__).parent.parent.joinpath('stored_results','jump.input')

def jumping(initial_cond, recalc, center_of_mass, robot):
    initial_cond = initial_cond + [0,0,0]

    # JUMP SETTINGS
    trajectory_target_= lambda t: [0, 0, 0]
    cost_func = lambda q,qd,u,t: 0.001 * qd.T@qd
    final_cost = lambda qf,qdf,uf: -70*center_of_mass['vel_y'](qf,qdf)    # verifica i valori 

    steps = 20
    time_horizon = 0.05

    constraint1 = lambda q, qd, u, ee, qdd: ([-3,-3,-3], q, [3,3,3])
    constraint2 = lambda q, qd, u, ee, qdd: ([-3,-3,-3], u, [3,3,3])
    constraint3 = lambda q, qd, u, ee, qdd: ([-1,-1,-1], u-q, [1,1,1])
    constraint4 = lambda q, qd, u, ee, qdd: ([0], center_of_mass['acc_y'](q,qd,qdd), [np.inf])
    constraint5 = lambda q, qd, u, ee, qdd: ([-0.095], center_of_mass['p_zero_mom'](q,qd,qdd), [0.095])
    my_constraints = [constraint1,constraint2,constraint3,constraint4,constraint5]
    constr_info = ['Constraints on q', 'Constraints on u', 'Constraints on u-q', 'Constraints on acc_y', 'Constraints on P_zero_mom']

    final_constraint1 = lambda q, qd, u, ee, qdd: ([-0.0], center_of_mass['vel_x'](q,qd), [0.0])
    final_constraint2 = lambda q, qd, u, ee, qdd: ([-0.0], center_of_mass['pos_x'](q), [0.0])
    final_constraint3 = lambda q, qd, u, ee, qdd: ([-0.2], q[0]+q[1]+q[2], [0.2])
    my_final_constraints = [final_constraint1, final_constraint2, final_constraint3]  
    final_constr_info = ['vel_x of CoM', 'pos_x of CoM', 'absolute angle of last joint', 'sensible final position']

    if recalc:
        optimizer.load_robot(robot['urdf_path'], robot['root'], robot['end'], sea_damping=robot['sea_damping'])
        res = optimizer.load_problem(
            cost_func=cost_func,
            control_steps=steps,
            initial_cond=initial_cond,
            trajectory_target=trajectory_target_,
            time_horizon = time_horizon,
            final_term_cost=final_cost, 
            max_iter=5000,
            my_constraint=my_constraints,
            my_final_constraint=my_final_constraints
            )
        fig = optimizer.show_result()

        #SAVING JUMP PHASE RESULTS
        u_jump = {
            'ankle': np.array(res['u'][0]),
            'knee': np.array(res['u'][1]),
            'hip': np.array(res['u'][2]),
            'rate': steps/time_horizon,
            'q': res['q'],
            'qd': res['qd'],
            'qdd': res['qdd'],
            't': time_horizon,
            'steps': steps
        }

        with open(store_path,"wb") as file:
            pickle.dump(u_jump, file)
    else:
        with open(store_path,"rb") as file:
            u_jump = pickle.load(file)

    visualize(u_jump, cost_func, final_cost, my_constraints, constr_info, my_final_constraints, final_constr_info, center_of_mass)

    return u_jump

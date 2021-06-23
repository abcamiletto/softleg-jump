from casadi.casadi import cos
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.offsetbox import AnchoredText
from matplotlib.gridspec import GridSpec
from pylab import savefig
import pathlib
from math import ceil, sqrt
file_path = pathlib.Path(__file__).parent


def visualize(jump, cost_func, final_term, constraints, desc):
    plt.close("all")
    state, settings = decode_input(jump)
    main_plot(**state, **settings, cost_func=cost_func, final_term=final_term, constraints=constraints, desc=desc)


def main_plot(q,qd,qdd,u,t,steps, cost_func, final_term, constraints, desc):
    period = t/steps
    tgrid = [period*k for k in range(steps+1)]
    
    fig1 = plot_q(q, qd, qdd, u, tgrid)
    fig2 = plot_sets(q, qd, qdd, u, cost_func,final_term, tgrid)
    fig3 = plot_constraints(q, qd, qdd, u, constraints, tgrid, desc)

    return None

def plot_q(q, qd, qdd, u, tgrid):
    label = ['ankle', 'knee', 'hip']
    gridspec_kw={   'width_ratios': [2, 1, 1],
                    'height_ratios': [1, 1, 1],
                    'wspace': 0.4,
                    'hspace': 0.4}
    fig, axes = plt.subplots(nrows=3, ncols=3, figsize=(11,8), gridspec_kw=gridspec_kw)
    fig.suptitle('Softleg Joints Dynamics')
    for idx, ax in enumerate(axes):
        ax[0].plot(tgrid, q[idx], '-')
        ax[0].plot(tgrid[:-1], u[idx], '--')
        ax[0].legend([label[idx]+'_q',label[idx]+'_u'])
        ax[0].set_xlabel('time')
        ax[0].set_ylabel(label[idx] + ' position')

        ax[1].plot(tgrid, qd[idx], 'g-')
        ax[1].legend([label[idx]+'_qd'])
        ax[1].set_xlabel('time')
        ax[1].set_ylabel(label[idx] + ' velocity')

        ax[2].plot(tgrid[:-1], qdd[idx], 'y-')
        ax[2].legend([label[idx]+'_qdd'])
        ax[2].set_xlabel('time')
        ax[2].set_ylabel(label[idx] + ' acceleration')
    savefig(file_path.joinpath('images','joints.png'))
    return fig

def plot_sets(q, qd, qdd, u, cost_func,final_term, tgrid):
    label = ['ankle', 'knee', 'hip']
    gridspec_kw={   'width_ratios': [ 1],
                    'height_ratios': [1],
                    'wspace': 0.4,
                    'hspace': 0.4}
    fig, axes = plt.subplots(nrows=1, ncols=1, figsize=(6,4), gridspec_kw=gridspec_kw)
    fig.suptitle('Cost Function')

    ref = lambda q : [np.array([q[0][idx],q[1][idx],q[2][idx]]) for idx in range(len(q[0]))]
    cost_plot = np.array([cost_func(q,qd,u,0) for q,qd,u in zip(ref(q), ref(qd), ref(u))])
    cost_plot = np.squeeze(cost_plot)
    cumulated_cost = [sum(cost_plot[:idx+1]) for idx in range(len(cost_plot))]
    end_point = [tgrid[-2], cumulated_cost[-1]]
    qf = [q[0][-1], q[1][-1], q[2][-1]]
    qdf = [qd[0][-1], qd[1][-1], qd[2][-1]]
    uf = [u[0][-1], u[1][-1], u[2][-1]]
    final_cost = float(final_term(qf,qdf,uf))

    axes.plot(tgrid[:-1], cumulated_cost, '-', color='tab:orange')
    axes.fill_between(tgrid[:-1], cumulated_cost, color='tab:orange') 
    axes.plot(tgrid[:-1], cost_plot, '-', color='tab:blue')
    axes.fill_between(tgrid[:-1], cost_plot, color='tab:blue')
    axes.arrow(tgrid[-2], 0, 0, final_cost, head_length=0.1, color='tab:pink', width=0.0005)
    axes.legend(['cumulated cost function', 'cost function', 'final term'])
    string = f'Cost Func: {cumulated_cost[-1]:.2e} \nFinal Term: {final_cost:.2e}'
    axes.text(0,1.015,string, transform = axes.transAxes)

    rateo = abs(cumulated_cost[-1] / final_cost)
    print(rateo)
    if rateo < 1/200 or rateo > 200:
        # these are matplotlib.patch.Patch properties
        props = dict(boxstyle='round', facecolor='tab:red', alpha=0.8)
        # place a text box in upper left in axes coords
        axes.text(0.25, 0.45, 'WARNING: Cost function\nand Final term are on way\ndifferent orders of magnitude', transform=axes.transAxes, fontsize=12, verticalalignment='center', bbox=props)

    axes.set_xlabel('time')
    axes.set_ylabel('cost function')
    return fig

def plot_constraints(q, qd, qdd, u, constraints, tgrid, titles):
    n_constr = len(constraints)

    fig = plt.figure(constrained_layout=True)
    gs = GridSpec(n_constr, 3, figure=fig)

    for idx, constraint in enumerate(constraints):
        ref = lambda q : [np.array([q[0][idx],q[1][idx],q[2][idx]]) for idx in range(len(q[0]))]
        constr_plot = [constraint(q,qd,u,0, qdd) for q,qd,u,qdd in zip(ref(q), ref(qd), ref(u), ref(qdd))]
        low_bound = np.array([instant[0] for instant in constr_plot])
        value = np.array([instant[1] for instant in constr_plot])
        high_bound = np.array([instant[2] for instant in constr_plot])
        if len(value[0]) == 1: # one dim constraint
            ax = fig.add_subplot(gs[idx, :])
            ax.set_facecolor((1.0, 0.5, 0.45))
            highB = high_bound[0] if high_bound[0] < 9999 else 40
            ax.axhspan(low_bound[0], highB, facecolor='w')
            ax.plot(tgrid[:-1], low_bound, '-', color='tab:red')
            ax.plot(tgrid[:-1], value, '-')
            ax.plot(tgrid[:-1], high_bound, '-', color='tab:red')
            ax.set_xlabel('time')
            ax.set_title(titles[idx])
        elif len(value[0]) == 3:
            for component in range(3):
                ax = fig.add_subplot(gs[idx, component])
                ax.set_facecolor((1.0, 0.5, 0.45))
                ax.axhspan(low_bound[0][component], high_bound[0][component] , facecolor='w')
                ax.plot(tgrid[:-1], [array[component] for array in low_bound], '-', color='tab:red')
                ax.plot(tgrid[:-1], [array[component] for array in value], '-')
                ax.plot(tgrid[:-1], [array[component] for array in high_bound], '-', color='tab:red')
                ax.set_xlabel('time')
                if component == 1: ax.set_title(titles[idx])

        



def decode_input(jump):
    q = [np.array(joint) for joint in jump['q']]
    qd = [np.array(joint) for joint in jump['qd']]
    qdd = [np.array(joint) for joint in jump['qdd']]
    u = [np.array(jump['ankle']), np.array(jump['knee']), np.array(jump['hip'])]

    state = {
        'q':q,
        'qd':qd,
        'qdd':qdd,
        'u':u
    }

    settings = {
        't':jump['t'],
        'steps':jump['steps'],
    }

    return state, settings
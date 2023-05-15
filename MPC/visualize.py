import numpy as np 
from numpy import sin, cos, pi
from matplotlib import pyplot as plt, animation
from mpl_toolkits.mplot3d import Axes3D
from common import *

# Animation fails with MacOs backend
plt.rcParams["backend"] = "TkAgg"

def plot_controls(cat_U, timestamp):
    ''' cat_U       -> intial value of each solution in the control history
        timestamp   -> time at each computed solution '''
    # Plot control
    figU, axsU  = plt.subplots(1, 1, figsize=(7, 15))    
    # exclude inital, final controls
    w1 = np.ravel(cat_U[0, :-N -1])
    w2 = np.ravel(cat_U[1, :-N -1])
    w3 = np.ravel(cat_U[2, :-N -1 ])
    w4 = np.ravel(cat_U[3, :-N -1])
    num_rows = w4.shape[0]
    w_ref = np.ones(num_rows) * hov_rpm

    axsU.stairs(w1, timestamp/stepTime, label='w1 ', color='lightcoral')
    axsU.stairs(w2, timestamp/stepTime, label='w2 ', color='moccasin')
    axsU.stairs(w3, timestamp/stepTime, label='w3 ', color='darkseagreen')
    axsU.stairs(w4, timestamp/stepTime, label='w4 ', color='lightsteelblue')
    axsU.stairs(w_ref, timestamp/stepTime, label='w_hov ', color='darkred')
    
    axsU.set_ylim(np.amin(cat_U) - 0.1, np.amax(cat_U) + 0.1)

    axsU.set_title('Control inputs')
    axsU.set_ylabel('propellor angular velocities (rad/s)')
    axsU.set_xlabel('MPC iterations')
    axsU.legend()
    plt.show()
    
iter = 0

def plot_states(cat_ST, t):

    def init():
        return path, horizon, sphere_i


    def animate(interval):
        
        # update path
        # print("animate pos", np.shape(cat_ST), cat_ST)
        global iter
        iter = iter +10
        path.set_data(cat_ST[0:2, 0, :iter])
        path.set_3d_properties(cat_ST[2, 0, :iter])
    
        # update horizon
        horizon.set_data(cat_ST[0:2, 0, :iter])
        horizon.set_3d_properties(cat_ST[2, 0, :iter])
        
        # update bot sphere position, orientation 
        # sphere_i.set_alpha(0)
        sphere_i._offsets3d = (cat_ST[0:3, 0, :iter])
        # sphere_i.set_alpha(0.2)

        return path, horizon, sphere_i

    fig = plt.figure()
    # plt.ion()
    ax = Axes3D(fig, auto_add_to_figure=False)
    ax.azim = -25
    ax.elev = 15
    fig.add_axes(ax)
    

    # path
    path = ax.plot([], [], 'blue', alpha=0.5, linewidth=0.5)[0]
    # horizon
    horizon, = ax.plot([], [],'x-g', alpha=0.5)

    cage_x = [-0.5, 2.5]  # dimensions in meters
    cage_y = [-0.5, 2.5]  # dimensions in meters
    cage_z = [0, 2]       # dimensions in meters
    ax.set_xlim3d(left = cage_x[0], right = cage_x[1])
    ax.set_ylim3d(bottom = cage_y[0], top = cage_y[1])
    ax.set_zlim3d(bottom = cage_z[0], top = cage_z[1])

    # Sphere around bot
    sphere_i = ax.scatter(init_st[0], init_st[1], init_st[2], s=pi * rob_rad**2 * 10000, c='blue', alpha=0.2)

    # Sphere around obstacle position
    sphere_o = ax.scatter( obst_st[0], obst_st[1], obst_st[2], s=pi * obst_rad**2 * 10000, c='red', alpha=0.2)
    
    # Sphere around target point
    sphere_t = ax.scatter( targ_st[0], targ_st[1], targ_st[2], s=pi * rob_rad**2 * 10000, c='green', alpha=0.2)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    anim = animation.FuncAnimation(fig=fig, func=animate, init_func=init, interval=80, frames=len(t),  blit=True)

    plt.show()
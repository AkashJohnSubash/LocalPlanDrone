import numpy as np 
from numpy import sin, cos, pi
from matplotlib import pyplot as plt, animation
from mpl_toolkits.mplot3d import Axes3D
from common import *

#Parameters defined in common.py

def plot_dataset(cat_U, timestamp):
    ''' cat_U       -> intial value of each solution in the control history
        timestamp   -> time at each computed solution '''
    # Plot control
    figU, axsU  = plt.subplots(4, 1, figsize=(7, 15))    
    w1 = np.ravel(cat_U[n_controls   : -4: n_controls])   # excluding init, final
    w2 = np.ravel(cat_U[n_controls+1 : -4: n_controls])
    w3 = np.ravel(cat_U[n_controls+2 : -4: n_controls])
    w4 = np.ravel(cat_U[n_controls+3 : -4: n_controls])
    
    axsU[0].stairs(w1, timestamp, label='w1 (rad/s)', color='b' )
    axsU[1].stairs(w2, timestamp, label='w2 (rad/s)', color='g')
    axsU[2].stairs(w3, timestamp, label='w3 (rad/s)', color='y' )
    axsU[3].stairs(w4, timestamp, label='w4 (rad/s)', color='r')
    axsU[0].set_title('Control Inputs')
    for ax in axsU:
         ax.legend()
    plt.show()
    
def simulate3D(cat_ST, t):
    init_azim = -45


    def init():
        return path, horizon, sphere_i


    def animate(interval):
        # rotate viewpoint (Not working, TODO find solution)
        #ax.azim = ax.azim-10
        
        # update path
        path.set_data(cat_ST[0:2, 0, :interval])
        path.set_3d_properties(cat_ST[2, 0, :interval])

        # update horizon
        horizon.set_data(cat_ST[0, :, interval], cat_ST[1, :, interval])
        horizon.set_3d_properties(cat_ST[2, :, interval])
        
        # update bot sphere (Not working, TODO find solution) 
        sphere_i._offsets3d = (cat_ST[0, 0, :interval], cat_ST[1, 0, :interval], cat_ST[2, 0, :interval])

        return path, horizon, sphere_i

    fig = plt.figure()
    #plt.ion()
    ax = Axes3D(fig, auto_add_to_figure=False)
    fig.add_axes(ax)

    # path
    path = ax.plot([], [], 'b', alpha=0.5, linewidth=1.5)[0]
    # horizon
    horizon, = ax.plot([], [],'x-g', alpha=0.5)

    min_scale = min(init_st[0], init_st[1], init_st[2], targ_st[0], targ_st[1], init_st[2]) - 2
    max_scale = max(init_st[0], init_st[1], init_st[2], targ_st[0], targ_st[1], init_st[2]) + 2
    
    ax.set_xlim3d(left = min_scale, right = max_scale)
    ax.set_ylim3d(bottom = min_scale, top = max_scale)
    ax.set_zlim3d(bottom = min_scale, top = max_scale)

    #Sphere around bot
    sphere_i = ax.scatter(init_st[0], init_st[1], init_st[2], s=pi * rob_rad**2 * 500, c='b', alpha=0.2)

    # Sphere around obstacle position
    sphere_o = ax.scatter( obst_st[0], obst_st[1], obst_st[2], s=pi * rob_rad**2 * 500, c='r', alpha=0.2)
    
    # Sphere around target point
    sphere_t = ax.scatter( targ_st[0], targ_st[1], targ_st[2], s=pi * rob_rad**2 * 500, c='g', alpha=0.2)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    anim = animation.FuncAnimation(fig=fig, func=animate, init_func=init, frames=len(t), interval=hznStep*500, blit=True)

    plt.show()
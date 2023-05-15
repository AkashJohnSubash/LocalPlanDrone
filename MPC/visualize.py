import numpy as np 
from numpy import pi
from matplotlib import pyplot as plt, animation
from mpl_toolkits.mplot3d import Axes3D
from common import *

# Animation fails with MacOs backend
plt.rcParams["backend"] = "TkAgg"

def plot_dataset(cat_U, timestamp):
    ''' cat_U       -> intial value of each solution in the control history
        timestamp   -> time at each computed solution '''
    # Plot control
    figU, axsU  = plt.subplots(1, 1, figsize=(7, 15))    
    w1 = np.ravel(cat_U[n_controls   : -4: n_controls])   # excluding init, final
    w2 = np.ravel(cat_U[n_controls+1 : -4: n_controls])
    w3 = np.ravel(cat_U[n_controls+2 : -4: n_controls])
    w4 = np.ravel(cat_U[n_controls+3 : -4: n_controls])
    w_ref = np.ones(w4.shape[0]) * u_hov

    axsU.stairs(w1, timestamp/stepTime, label='w1 ', color='b' )
    axsU.stairs(w2, timestamp/stepTime, label='w2 ', color='g')
    axsU.stairs(w3, timestamp/stepTime, label='w3 ', color='y' )
    axsU.stairs(w4, timestamp/stepTime, label='w4 ', color='r')
    axsU.stairs(w_ref, timestamp/stepTime, label='w_hov ', color='darkred')
    
    axsU.set_ylim(np.amin(cat_U) - 0.1, np.amax(cat_U) + 0.1)


    axsU.set_title('Control inputs')
    axsU.set_ylabel('propellor angular velocities (rad/s)')
    axsU.set_xlabel('MPC iterations')
    axsU.legend()
    plt.show()
    
def simulate3D(cat_ST, t):

    def init():
        
        return path, horizon, sphere_i


    def animate(interval):
        
        # update path
        path.set_data(cat_ST[0:2, 0, :interval])
        path.set_3d_properties(cat_ST[2, 0, :interval])
    
        # update horizon
        horizon.set_data(cat_ST[0, :, interval], cat_ST[1, :, interval])
        horizon.set_3d_properties(cat_ST[2, :, interval])
        
        # update bot sphere position, orientation 
        sphere_i._offsets3d = (cat_ST[0, 0, :interval], cat_ST[1, 0, :interval], cat_ST[2, 0, :interval])

        return path, horizon, sphere_i

    fig = plt.figure()
    #plt.ion()
    ax = Axes3D(fig, auto_add_to_figure=False)
    ax.azim = -25
    ax.elev = 15
    fig.add_axes(ax)
    

    # path
    path = ax.plot([], [], 'b', alpha=0.5, linewidth=0.5)[0]
    # horizon
    horizon, = ax.plot([], [],'x-g', alpha=0.5)

    cage_x = [-0.5, 1.5]  #dimensions in meters
    cage_y = [-0.5, 1.5]  #dimensions in meters
    cage_z = [0, 2]       #dimensions in meters
    ax.set_xlim3d(left = cage_x[0], right = cage_x[1])
    ax.set_ylim3d(bottom = cage_y[0], top = cage_y[1])
    ax.set_zlim3d(bottom = cage_z[0], top = cage_z[1])

    #Sphere around bot
    sphere_i = ax.scatter(init_st[0], init_st[1], init_st[2], s=pi * rob_rad**2 * 14000, c='b', alpha=0.2)

    # Sphere around obstacle position
    sphere_o = ax.scatter( obst_st[0], obst_st[1], obst_st[2], s=pi * obst_rad**2 * 14000, c='r', alpha=0.2)
    
    # Sphere around target point
    sphere_t = ax.scatter( targ_st[0], targ_st[1], targ_st[2], s=pi * rob_rad**2 * 14000, c='g', alpha=0.2)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    anim = animation.FuncAnimation(fig=fig, func=animate, init_func=init, frames=len(t), interval=20, blit=True)

    plt.show()
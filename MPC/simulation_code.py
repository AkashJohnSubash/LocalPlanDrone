import numpy as np 
from numpy import sin, cos, pi
import matplotlib.pyplot as plt
from matplotlib import animation
from time import time
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

def simulate(cat_ST, cat_U, t, save=False):
    ''' cat_U       -> intial value of each solution in the control history
        cat_ST      -> full predition of each solution in the State history
        timestamp   -> time at each computed solution 
        global vars : hznStep, hznLen, rob_rad, obs_rad, init_st, targ_st, obst_st '''
                            # [x, y, z, qw, qx, qy, qz]
    def create_triangle(state=[0, 0, 0,  1,  0,  0, 0], h=0.14, w=0.09, update=False):
        phi, th, psi = quatern2euler(state[3 : ])
        x, y = state[0 : 2]
        triangle = np.array([   [h, 0   ],
                                [0,  w/2],
                                [0, -w/2],
                                [h, 0   ]]).T

        rotation_matrix = np.array([[cos(th), -sin(th)],
                                    [sin(th),  cos(th)]])

        coords = np.array([[x, y]]) + (rotation_matrix @ triangle).T
        if update == True:
            return coords
        else:
            return coords[:3, :]

    def init():
        return path, horizon, current_state, target_state, bot_boundary

    def animate(i):
        # get variables
        x = cat_ST[0, 0, i]
        y = cat_ST[1, 0, i]
        #phi, th, psi = quatern2euler([cat_ST[3, 0, i], cat_ST[3, 0, i], cat_ST[3, 0, i], cat_ST[3, 0, i]])

        # update path
        if i == 0:
            path.set_data(np.array([]), np.array([]))
        x_new = np.hstack((path.get_xdata(), x))
        y_new = np.hstack((path.get_ydata(), y))
        path.set_data(x_new, y_new)

        # update horizon
        x_new = cat_ST[0, :, i]
        y_new = cat_ST[1, :, i]
        horizon.set_data(x_new, y_new)

        # update current_state, bounding circle
        current_state.set_xy(create_triangle(cat_ST[0:8, 0, i], update=True))
        bot_boundary.set_center([x, y])
        
        # update target_state
        # xy = target_state.get_xy()
        # target_state.set_xy(xy)            

        return path, horizon, current_state, target_state, bot_boundary

    # create figure and axes
    fig, ax = plt.subplots(figsize=(6, 6))
    min_scale = min(init_st[0], init_st[1], targ_st[0], targ_st[1]) - 2
    max_scale = max(init_st[0], init_st[1], targ_st[0], targ_st[1]) + 2
    ax.set_xlim(left = min_scale, right = max_scale)
    ax.set_ylim(bottom = min_scale, top = max_scale)

    # create lines:
    # path
    path, = ax.plot([], [], 'k', linewidth=2)
    # horizon
    horizon, = ax.plot([], [], 'x-g', alpha=0.5)
    
    # Generate triangle from current x, y, theta
    current_triangle = create_triangle(init_st)        
    current_state = ax.fill(current_triangle[:, 0], current_triangle[:, 1], color='y')
    current_state = current_state[0]
    # current state's boundary circle # TODO around mid-point
    bot_boundary = plt.Circle(current_triangle[0], rob_rad, color='y', fill = False, linestyle = '--')
    ax.add_artist(bot_boundary)
    
    # Generate triangle from target's x, y, theta
    target_triangle = create_triangle(targ_st)
    target_state = ax.fill(target_triangle[:, 0], target_triangle[:, 1], color='b')
    target_state = target_state[0]

    # Generate triangle from obstacle's x, y, theta
    obst_triangle = create_triangle(obst_st)
    obst_state = ax.fill(obst_triangle[:, 0], obst_triangle[:, 1], color='b')
    obst_state = obst_state[0]
    # obstace boundary circle
    obst_boundary = plt.Circle(obst_triangle[0], obst_rad, color='r', fill = False, linestyle = '--')
    ax.add_artist(obst_boundary)

    sim = animation.FuncAnimation(  fig=fig, func=animate, init_func=init, frames=len(t), interval=hznStep*400, blit=True,
                                    repeat=True)
    plt.grid()
    plt.show()

    if save == True:
        sim.save('./animation' + str(time()) +'.gif', writer='ffmpeg', fps=30)

    return
import numpy as np 
from numpy import sin, cos, pi
import matplotlib.pyplot as plt
from matplotlib import animation
from time import time

# redefnition #TODO put in config .yml
n_state    = 13
n_control = 4

def plot_dataset(cat_U, timestamp):
    ''' cat_U       -> intial value of each solution in the control history
        timestamp   -> time at each computed solution '''
    # Plot control
    figU, axsU  = plt.subplots(4, 1, figsize=(7, 15))    
    w1 = np.ravel(cat_U[n_control   : -4: n_control])   # excluding init, final
    w2 = np.ravel(cat_U[n_control+1 : -4: n_control])
    w3 = np.ravel(cat_U[n_control+2 : -4: n_control])
    w4 = np.ravel(cat_U[n_control+3 : -4: n_control])
    
    axsU[0].stairs(w1, timestamp, label='w1 (rad/s)', color='b' )
    axsU[1].stairs(w2, timestamp, label='w2 (rad/s)', color='g')
    axsU[2].stairs(w3, timestamp, label='w3 (rad/s)', color='y' )
    axsU[3].stairs(w4, timestamp, label='w4 (rad/s)', color='r')
    axsU[0].set_title('Control Inputs')
    for ax in axsU:
         ax.legend()
    plt.show()

def simulate(cat_ST, cat_U, t, step_horizon, N, ref_st, bot_rad, obst_coord, obs_rad, save=False):
    ''' cat_U -> intial value of each solution in the control history
        timestamp -> time at each computed solution '''

    def create_triangle(state=[0,0,0], h=0.14, w=0.09, update=False):
        x, y, th = state
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
        th = cat_ST[3, 0, i]

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
        current_state.set_xy(create_triangle([x, y, th], update=True))
        bot_boundary.set_center([x, y])
        
        # update target_state
        # xy = target_state.get_xy()
        # target_state.set_xy(xy)            

        return path, horizon, current_state, target_state, bot_boundary

    # create figure and axes
    fig, ax = plt.subplots(figsize=(6, 6))
    min_scale = min(ref_st[0], ref_st[1], ref_st[13], ref_st[14]) - 2
    max_scale = max(ref_st[0], ref_st[1], ref_st[13], ref_st[14]) + 2
    ax.set_xlim(left = min_scale, right = max_scale)
    ax.set_ylim(bottom = min_scale, top = max_scale)

    # create lines:
    # path
    path, = ax.plot([], [], 'k', linewidth=2)
    # horizon
    horizon, = ax.plot([], [], 'x-g', alpha=0.5)
    
    # current state
    current_triangle = create_triangle([ref_st[0], ref_st[1], ref_st[3]])
    current_state = ax.fill(current_triangle[:, 0], current_triangle[:, 1], color='y')
    current_state = current_state[0]
    # current state's boundary circle # TODO around mid-point
    bot_boundary = plt.Circle(current_triangle[0], bot_rad, color='y', fill = False, linestyle = '--')
    ax.add_artist(bot_boundary)
    # target state
    target_triangle = create_triangle([ref_st[13], ref_st[14], ref_st[16]])
    target_state = ax.fill(target_triangle[:, 0], target_triangle[:, 1], color='b')
    target_state = target_state[0]

    # obstace state
    obst_triangle = create_triangle([obst_coord[0], obst_coord[1], obst_coord[3] ])
    obst_state = ax.fill(obst_triangle[:, 0], obst_triangle[:, 1], color='b')
    obst_state = obst_state[0]
    # obstace boundary circle
    obst_boundary = plt.Circle(obst_triangle[0], obs_rad, color='r', fill = False, linestyle = '--')
    ax.add_artist(obst_boundary)

    sim = animation.FuncAnimation(  fig=fig, func=animate, init_func=init, frames=len(t), interval=step_horizon*400, blit=True,
                                    repeat=True)
    plt.grid()
    plt.show()

    if save == True:
        sim.save('./animation' + str(time()) +'.gif', writer='ffmpeg', fps=30)

    return
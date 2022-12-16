import numpy as np 
from numpy import sin, cos, pi
import matplotlib.pyplot as plt
from matplotlib import animation
from time import time

def plot_dataset(ctrl_data = [], state_data = [], timestamp = []):
    fig, axs  = plt.subplots(2, 1, figsize=(7, 15))
    v_lastIdx = len(ctrl_data[0,:])
    w_lastIdx = len(ctrl_data[1,:])
    axs[0].stairs(ctrl_data[0,:][: v_lastIdx], timestamp[: v_lastIdx+ 1], label='v (m/s)', color='b' )
    axs[1].stairs(ctrl_data[1,:][: w_lastIdx], timestamp[: w_lastIdx+ 1], label='w (rad/s)', color='g')
    axs[0].set_title('Control Inputs')
    for ax in axs:
         ax.legend()
    plt.show()

def simulate(cat_states, cat_controls, t, step_horizon, N, reference, robot_diam, save=False):
    #ang=0:0.005:2*pi
    #xp = robot_diam
    #TODO print("\nDEBUG REFERENCE DATA\n", reference, reference.shape)
    
    def create_triangle(state=[0,0,0], h=0.14, w=0.09, update=False):
        x, y, th = state
        triangle = np.array([   [h, 0   ],
                                [0,  w/2],
                                [0, -w/2],
                                [h, 0   ]]).T

        rotation_matrix = np.array([[cos(th), -sin(th)],
                                    [sin(th),  cos(th)]])

        coords = np.array([[x, y]]) + (rotation_matrix @ triangle).T
        print("\nDEBUG2 \n", coords)
        if update == True:
            return coords
        else:
            return coords[:3, :]

    

    def init():
        #TODO remove circle = ax.circle(xy = current_triangle[0, :], radius = robot_diam, color = "coral")
        return path, horizon, current_state, target_state#, circle

    def animate(i):
        # get variables
        x = cat_states[0, 0, i]
        y = cat_states[1, 0, i]
        th = cat_states[2, 0, i]

        # update path
        if i == 0:
            path.set_data(np.array([]), np.array([]))
        x_new = np.hstack((path.get_xdata(), x))
        y_new = np.hstack((path.get_ydata(), y))
        path.set_data(x_new, y_new)

        # update horizon
        x_new = cat_states[0, :, i]
        y_new = cat_states[1, :, i]
        horizon.set_data(x_new, y_new)

        # update current_state
        current_state.set_xy(create_triangle([x, y, th], update=True))

        # update target_state
        # xy = target_state.get_xy()
        # target_state.set_xy(xy)            

        return path, horizon, current_state, target_state,

    # create figure and axes
    fig, ax = plt.subplots(figsize=(6, 6))
    min_scale = min(reference[0], reference[1], reference[3], reference[4]) - 2
    max_scale = max(reference[0], reference[1], reference[3], reference[4]) + 2
    ax.set_xlim(left = min_scale, right = max_scale)
    ax.set_ylim(bottom = min_scale, top = max_scale)

    # create lines:
    #   path
    path, = ax.plot([], [], 'k', linewidth=2)
    #   horizon
    horizon, = ax.plot([], [], 'x-g', alpha=0.5)
    #   current_state
    #TODO print("\nDEBUG1 \n", reference[:3])
    current_triangle = create_triangle(reference[:3])
    current_state = ax.fill(current_triangle[:, 0], current_triangle[:, 1], color='r')
    current_state = current_state[0]
    print("\nDEBUG3\n", current_triangle)

    # circle
    # current_circle = create_circle(reference[:3])
    # current_circle = ax.fill(current_circle[:, 0], current_circle[:, 1], color='g')
    # current_circle = current_circle[0]

    # target_state
    target_triangle = create_triangle(reference[3:])
    target_state = ax.fill(target_triangle[:, 0], target_triangle[:, 1], color='b')
    target_state = target_state[0]

    sim = animation.FuncAnimation(  fig=fig, func=animate, init_func=init, frames=len(t), interval=step_horizon*100, blit=True,
                                    repeat=True)
    plt.grid()
    plt.show()

    if save == True:
        sim.save('./animation' + str(time()) +'.gif', writer='ffmpeg', fps=30)

    return
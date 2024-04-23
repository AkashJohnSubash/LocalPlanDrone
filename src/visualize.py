import numpy as np 
from numpy import pi
# from matplotlib import pyplot as plt, animation
from mpl_toolkits.mplot3d import Axes3D
from common import *

import matplotlib as mpl
from matplotlib import pyplot as plt, animation
import shutil

# plt.rcParams["backend"] = "TkAgg"

text_usetex = True if shutil.which('latex') else False
params = {
        'text.latex.preamble': r"\usepackage{gensymb} \usepackage{amsmath}",
        'axes.labelsize': 12,
        'axes.titlesize': 12,
        'legend.fontsize': 12,
        'xtick.labelsize': 12,
        'ytick.labelsize': 12,
        'text.usetex': text_usetex,
        'font.family': 'serif'
}

mpl.rcParams.update(params)


def plot_dataset(timestamps, hist_u):
    ''' cat_U       -> intial value of each solution in the control history
        timestamp   -> time at each computed solution '''
    # Plot control
    figU, axsU  = plt.subplots(1, 1, figsize=(4.8, 2.7))    
    w1 = np.ravel(hist_u[0, 2:-4])   # excluding init, final
    w2 = np.ravel(hist_u[1, 2:-4])
    w3 = np.ravel(hist_u[2, 2:-4])
    w4 = np.ravel(hist_u[3, 2:-4])
    w_ref = np.ones(w4.shape[0]) * u_hov

    axsU.stairs(w1, timestamps[2:-2], label='$\\Omega_{1}$', color='lightcoral' )
    axsU.stairs(w2, timestamps[2:-2], label='$\\Omega_{2}$', color='plum')
    axsU.stairs(w3, timestamps[2:-2], label='$\\Omega_{3}$', color='darkseagreen' )
    axsU.stairs(w4, timestamps[2:-2], label='$\\Omega_{4}$', color='lightsteelblue')
    # axsU.stairs(w_ref, timestamps[:], label='$\\Omega_{\mathrm{ref}}$', color='lightseagreen')
    
    axsU.set_ylim(np.amin(hist_u) - 0.1, np.amax(hist_u) + 0.1)


    # axsU.set_title('Control inputs')
    axsU.set_ylabel('$u\,\mathrm{(rad\,s^{-1})}$')
    axsU.set_xlabel('time (s)')
    axsU.legend(loc='upper right', ncol=2)
    axsU.grid()

    figU.savefig('u.pdf', format='pdf', bbox_inches='tight')
    plt.show()
    
def animOptVars(time_stamps, traj_zeta0, traj_u0):
    anim_running = True
    def init():
        time_text.set_text('')
        return path, horizon, sphere_i


    def onClick(event):
        nonlocal anim_running
        if anim_running:
            anim.event_source.stop()
            anim_running = False
        else:
            anim.event_source.start()
            anim_running = True


    def animate(iter):
        
        # update path
        path.set_data(traj_zeta0[0:2, 0, :iter])
        path.set_3d_properties(traj_zeta0[2, 0, :iter])
    
        # update horizon
        horizon.set_data(traj_zeta0[0:2, :, iter])
        horizon.set_3d_properties(traj_zeta0[2, :, iter])
        
        # update bot sphere position, orientation 
        sphere_i._offsets3d = (traj_zeta0[0:3, 0:1, iter])

        # Update control plot 
        omg0Ax.set_data(traj_u0[0, :iter], time_stamps[:iter +1] )
        omg1Ax.set_data(traj_u0[1, :iter], time_stamps[ :iter +1] )
        omg2Ax.set_data(traj_u0[2, :iter], time_stamps[ :iter +1] )
        omg3Ax.set_data(traj_u0[3, :iter], time_stamps[ :iter +1] )

        return path, horizon, sphere_i

    fig = plt.figure(figsize=(15,9))

    # plot control u at (1,3) bottom right
    u = fig.add_subplot(3, 3, 1)
    u.set_ylim(np.amin(np.ravel(traj_u0[:, :-2])) - 0.2, 
               np.amax(np.ravel(traj_u0[:, :-2])) + 0.2)
    u.set_xlim(0, np.amax(time_stamps[:]) + 0.2)
    u.set_xlabel('time (s)')
    u.set_ylabel('$u\,\mathrm{(rad\,s^{-1})}$')

    omg0Ax = u.stairs([], [0], baseline=None,label="$\\Omega_{0}$", color="lightcoral")
    omg1Ax = u.stairs([], [0], baseline=None,label="$\\Omega_{1}$", color="plum" )
    omg2Ax = u.stairs([], [0], baseline=None,label="$\\Omega_{2}$", color="darkseagreen" )
    omg3Ax = u.stairs([], [0], baseline=None,label="$\\Omega_{3}$", color="lightsteelblue" )
    u.legend(loc='upper right', ncol=2)

    # plot cartesian 3D view
    ax3d = fig.add_subplot(3, 3, (5, 9), projection='3d')
    ax3d.azim = -25
    ax3d.elev = 15
    fig.add_axes(ax3d)

    # time field 
    time_text = ax3d.text2D(0.02, 0.95, '', transform=ax3d.transAxes)

    # path
    path = ax3d.plot([], [], 'b', alpha=0.5, linewidth=0.5)[0]
    # horizon
    horizon, = ax3d.plot([], [],'x-g', alpha=0.5)

    cage_x = [-0.5, 1.5]  # dimensions in meters
    cage_y = [-0.5, 1.5]  # dimensions in meters
    cage_z = [0, 2]       # dimensions in meters

    ax3d.set_xlim3d(left = cage_x[0], right = cage_x[1])
    ax3d.set_ylim3d(bottom = cage_y[0], top = cage_y[1])
    ax3d.set_zlim3d(bottom = cage_z[0], top = cage_z[1])

    #Sphere around bot
    sphere_i = ax3d.scatter(init_st[0], init_st[1], init_st[2], s=pi * rob_rad**2 * 14000, c='b', alpha=0.2)

    # Sphere around obstacle position
    sphere_o = ax3d.scatter( obst_st[0], obst_st[1], obst_st[2], s=pi * obst_rad**2 * 14000, c='r', alpha=0.2)
    
    # Sphere around target point
    sphere_t = ax3d.scatter( targ_st[0], targ_st[1], targ_st[2], s=pi * rob_rad**2 * 14000, c='g', alpha=0.2)

    ax3d.set_xlabel('X')
    ax3d.set_ylabel('Y')
    ax3d.set_zlabel('Z')
    
    fig.canvas.mpl_connect('button_press_event', onClick)
    anim = animation.FuncAnimation(fig=fig, 
                                   func=animate,
                                   init_func=init, 
                                   frames=len(time_stamps), 
                                   interval=60, 
                                   repeat=True,
                                   blit=False)

    fig.tight_layout()
    plt.show()
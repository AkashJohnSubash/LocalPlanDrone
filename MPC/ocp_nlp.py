from casadi import *
from common import *
from sys_dynamics import SysDyn as Sys, Predictor as Pred
 
def setup_nlp():
    
    # generates optimal trajectory using an NMPC cost function
    SysObj = Sys()
    ST = SysObj.St;   U = SysObj.U;   P = SysObj.P
    dyn_fp = SysObj.ForwardDynamics()

    cost_fn = 0                  # cost function
    g = ST[:, 0] - P[:n_states]  # constraints in the equation
    U_hov = np.array([hover_krpm, hover_krpm, hover_krpm, hover_krpm])

    '''-----------Formulate OCP as inequality constrained NLP------------'''
    # Multiple shooting equality constraint (Optima = RK4 integrator soln)
    for k in range(hznLen):
        st = ST[:, k]
        U_k = U[:, k] 
        cost_fn = cost_fn + ((st - P[n_states:]).T @ Q @ (st - P[n_states:])) + (U_k - U_hov).T @ R @ (U_k - U_hov)
        st_opt = ST[:, k+1]
        st_est = Pred.rk4_integrator(dyn_fp, st, U_k, stepTime)
        g = vertcat(g, st_opt - st_est)           

    # Path inequality constraint (obstacle avoidance)
    for k in range(hznLen +1): 
        euclid = (ST[0: 3, k] - obst_st[0:3])
        g = vertcat(g , ((euclid.T @ euclid) -( rob_rad + obst_rad)))          

    OPT_variables = vertcat( ST.reshape((-1, 1)),  U.reshape((-1, 1)) )
    nlp_prob = { 'f': cost_fn, 'x': OPT_variables, 'g': g, 'p': P }
    '''-----------------------Configure solver-----------------------------'''


    opts = {'ipopt'     : { 'max_iter': 1000, 'print_level': 0, 'acceptable_tol': 1e-8, 'acceptable_obj_change_tol': 1e-6}, #'hessian_approximation': 'limited-memory'},
            'print_time': 0,
            'jit' : True,
            'jit_options' : {'flags' : ['-O3'], 'verbose': True},
            'jit_cleanup' : True
            #'compiler': 'shell' , 
            }

    solver = nlpsol('solver', 'ipopt', nlp_prob, opts)

    st_size = n_states * (hznLen+1)
    U_size = n_controls * hznLen

    '''-----------------------Define NLP bounds-----------------------------'''

    # Bounds on decision variables
    lbx = DM.zeros((st_size + U_size, 1))
    ubx = DM.zeros((st_size + U_size, 1))
    # State bounds
    lbx[0:  st_size: n_states] = -1;            ubx[0: st_size: n_states] = 1.5                   # x lower, upper bounds
    lbx[1:  st_size: n_states] = -1;            ubx[1: st_size: n_states] = 1.5                   # y bounds
    lbx[2:  st_size: n_states] = 0;             ubx[2: st_size: n_states] = 2                 # z bounds
    lbx[3:  st_size: n_states] = -inf;          ubx[3:  st_size: n_states] = inf                  # qw bounds TODO find appropriate val
    lbx[4:  st_size: n_states] = -inf;          ubx[4:  st_size: n_states] = inf                  # qx bounds
    lbx[5:  st_size: n_states] = -inf;          ubx[5:  st_size: n_states] = inf                  # qy bounds
    lbx[6:  st_size: n_states] = -inf;          ubx[6:  st_size: n_states] = inf                  # qz bounds
    lbx[7:  st_size: n_states] = v_min;         ubx[7:  st_size: n_states] = v_max              # u bounds
    lbx[8:  st_size: n_states] = v_min;         ubx[8:  st_size: n_states] = v_max              # v bounds
    lbx[9:  st_size: n_states] = v_min;         ubx[9:  st_size: n_states] = v_max              # w bounds
    lbx[10: st_size: n_states] = w_min;         ubx[10: st_size: n_states] = w_max              # p bounds TODO find appropriate val
    lbx[11: st_size: n_states] = w_min;         ubx[11: st_size: n_states] = w_max              # q bounds
    lbx[12: st_size: n_states] = w_min;         ubx[12: st_size: n_states] = w_max              # r bounds
    # Control bounds
    lbx[st_size    : : n_controls] = 0;         ubx[st_size     : : n_controls] = max_krpm        # w1 bounds
    lbx[st_size +1 : : n_controls] = 0;         ubx[st_size+1   : : n_controls] = max_krpm        # w2 bounds
    lbx[st_size +2 : : n_controls] = 0;         ubx[st_size+2   : : n_controls] = max_krpm        # w3 bounds
    lbx[st_size +3 : : n_controls] = 0;         ubx[st_size+3   : : n_controls] = max_krpm        # w4 bounds

    # Bounds on constraints
                    #MS,        Path,       Smoothen
    lbg = DM.zeros((st_size + (hznLen+1)))
    ubg = DM.zeros((st_size + (hznLen+1)))

    # MS constraints: pred_st - optim_st = 0
    lbg[0 : st_size] = 0;                         ubg[0        : st_size] = 0

    # Path constraints: 0 < Euclidian - sum(radii) < inf
    lbg[st_size : st_size + (hznLen+1)]   = 0; ubg[st_size  : st_size+ (hznLen+1)] = inf

    args = {    'lbg': lbg,                    # constraints lower bound
                'ubg': ubg,                    # constraints upper bound
                'lbx': lbx,
                'ubx': ubx}

    '''---------------------------------------------------------------'''

    return args, solver
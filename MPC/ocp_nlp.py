from casadi import *
from common import *
from sys_dynamics import SysDyn
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from time import  time
import scipy

U_hov = np.array([hov_rpm, hov_rpm, hov_rpm, hov_rpm])

def setup_nlp():
    '''Acados settings for NLP'''
    
    # create render arguments
    ocp = AcadosOcp()

    # export model
    model, constraint = SysDyn.acados_model_format()

    # define acados ODE
    model_ac = AcadosModel()
    model_ac.f_impl_expr = model.f_impl_expr
    model_ac.f_expl_expr = model.f_expl_expr
    model_ac.x = model.x
    model_ac.xdot = model.xdot
    model_ac.u = model.u
    model_ac.z = model.z
    model_ac.p = model.p
    model_ac.name = model.name
    ocp.model = model_ac

    # define initial conditions
    model.x0 = np.copy(init_st) # MOVED def from model

    # define constraint
    # model_ac.con_h_expr = constraint.expr

    # set dimensions
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    #ny_e = nx  # No terminal cost

    ocp.dims.N = hznLen

    # set cost
    ocp.cost.cost_type = "LINEAR_LS"                # TODO Implicitly LLS ?
    ocp.cost.W = scipy.linalg.block_diag(Q, R)

    Vx = np.zeros((ny, nx))
    Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx = Vx

    Vu = np.zeros((ny, nu))
    Vu[13,0] = 1.0
    Vu[14,1] = 1.0
    Vu[15,2] = 1.0
    Vu[16,3] = 1.0
    ocp.cost.Vu = Vu

    # set intial references        # nx +  nu
    ocp.cost.yref = np.concatenate((targ_st, U_hov))
    # ocp.cost.yref_e = np.array([0, 0, 0, 0, 0, 0]) # No terminal cost

    ocp.constraints.x0  = model.x0
    ocp.constraints.lbu = np.array([      0,       0,       0,        0]) #TODO check, constrain all NuxN controls ?
    ocp.constraints.ubu = np.array([max_rpm, max_rpm, max_rpm,  max_rpm])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])

    #TODO Constrain state decision variables
    # set QP solver and integration
    ocp.solver_options.tf = stepTime
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"

    # create solver
    acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")

    return constraint, model, acados_solver

'''IPOPT OCP fomulation'''


    # ''# Generates optimal trajectory using an NMPC cost function
    # ST, U, P = SysDyn.init()
    # #ST = SysObj.St;   U = SysObj.U;   P = SysObj.P
    # dyn_fp = SysDyn.model_ode()

    # cost_fn = 0                  # cost function
    # g = ST[:, 0] - P[:n_states]  # constraints in the equation
    # #U_hov = np.array([hov_rpm, hov_rpm, hov_rpm, hov_rpm]) #Duplicate

    # '''-----------Formulate OCP as inequality constrained NLP------------'''
    # # MPC cost, Initial value constraint
    # for k in range(hznLen):
    #     st = ST[:, k]
    #     U_k = U[:, k] 
    #     cost_fn = cost_fn + ((st - P[n_states:]).T @ Q @ (st - P[n_states:])) + (U_k - U_hov).T @ R @ (U_k - U_hov)
    #     st_opt = ST[:, k+1]
    #     st_est = Predictor.rk4_explicit(dyn_fp, st, U_k, stepTime)
    #     g = vertcat(g, st_opt - st_est)           

    # # Path inequality constraint (obstacle avoidance)
    # for k in range(hznLen +1): 
    #     euclid = (ST[0: 3, k] - obst_st[0:3])
    #     g = vertcat(g , ((euclid.T @ euclid) -( rob_rad + obst_rad)))          

    # OPT_variables = vertcat( ST.reshape((-1, 1)),  U.reshape((-1, 1)) )
    # nlp_prob = {'f': cost_fn, 'x': OPT_variables, 'g': g, 'p': P }

    # '''-----------------------Configure solver-----------------------------'''


    # opts = {'ipopt'     : {'max_iter': 1000, 'print_level': 0, 'acceptable_tol': 1e-8,
    #                        'acceptable_obj_change_tol': 1e-6, 'linear_solver' :'mumps'},
    #         'print_time': 0, 
    #         'jit' : False,
    #         'compiler' : 'shell',
    #         'jit_options' : { 'verbose': True, 'flags' : ['-O1']},
    #         'jit_cleanup' : True,
    #         }

    # solver = nlpsol('solver', 'ipopt', nlp_prob, opts)
    # st_size = n_states * (hznLen+1)
    # U_size = n_controls * hznLen

    # '''-----------------------Define NLP bounds-----------------------------'''

    # # Bounds on decision variables
    # lbx = DM.zeros((st_size + U_size, 1))
    # ubx = DM.zeros((st_size + U_size, 1))
    # # State bounds
    # lbx[0:  st_size: n_states] = -1;            ubx[0: st_size: n_states] = 1.5                   # x lower, upper bounds
    # lbx[1:  st_size: n_states] = -1;            ubx[1: st_size: n_states] = 1.5                   # y bounds
    # lbx[2:  st_size: n_states] = 0;             ubx[2: st_size: n_states] = 2                     # z bounds
    # lbx[3:  st_size: n_states] = -inf;          ubx[3:  st_size: n_states] = inf                  # qw bounds
    # lbx[4:  st_size: n_states] = -inf;          ubx[4:  st_size: n_states] = inf                  # qx bounds
    # lbx[5:  st_size: n_states] = -inf;          ubx[5:  st_size: n_states] = inf                  # qy bounds
    # lbx[6:  st_size: n_states] = -inf;          ubx[6:  st_size: n_states] = inf                  # qz bounds
    # lbx[7:  st_size: n_states] = v_min;         ubx[7:  st_size: n_states] = v_max                # u bounds
    # lbx[8:  st_size: n_states] = v_min;         ubx[8:  st_size: n_states] = v_max                # v bounds
    # lbx[9:  st_size: n_states] = v_min;         ubx[9:  st_size: n_states] = v_max                # w bounds
    # lbx[10: st_size: n_states] = w_min;         ubx[10: st_size: n_states] = w_max                # p bounds
    # lbx[11: st_size: n_states] = w_min;         ubx[11: st_size: n_states] = w_max                # q bounds
    # lbx[12: st_size: n_states] = w_min;         ubx[12: st_size: n_states] = w_max                # r bounds
    # # Control bounds
    # lbx[st_size    : : n_controls] = 0;         ubx[st_size     : : n_controls] = max_rpm        # w1 bounds
    # lbx[st_size +1 : : n_controls] = 0;         ubx[st_size+1   : : n_controls] = max_rpm        # w2 bounds
    # lbx[st_size +2 : : n_controls] = 0;         ubx[st_size+2   : : n_controls] = max_rpm        # w3 bounds
    # lbx[st_size +3 : : n_controls] = 0;         ubx[st_size+3   : : n_controls] = max_rpm        # w4 bounds

    # # Bounds on constraints
    # # Initial value,   Path,
    # lbg = DM.zeros((st_size )+ (hznLen+1))
    # ubg = DM.zeros((st_size )+ (hznLen+1))

    # # Initial value constraints: pred_st - optim_st = 0
    # lbg[0 : st_size] = 0;                           ubg[0        : st_size] = 0

    # # Path constraints: 0 < Euclidian - sum(radii) < inf
    # lbg[st_size : st_size + (hznLen+1)]   = 0;      ubg[st_size  : st_size+ (hznLen+1)] = inf

    # args = {    'lbg': lbg,                    # constraints lower bound
    #             'ubg': ubg,                    # constraints upper bound
    #             'lbx': lbx,
    #             'ubx': ubx}

    # '''---------------------------------------------------------------'''

    # return args, solver
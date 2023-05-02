from casadi import *
from common import *
from sys_dynamics import SysDyn
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver, AcadosSimSolver
import scipy

U_hov = np.array([hov_rpm, hov_rpm, hov_rpm, hov_rpm])

def setup_nlp():
    '''Acados settings for NLP'''
    
    # create render arguments
    ocp = AcadosOcp()

    # export model
    f_expl, f_impl, st, st_dt, ctrl = SysDyn.model_ode()

    # define acados ODE
    model_ac = AcadosModel()
    model_ac.f_impl_expr = f_impl
    model_ac.f_expl_expr = f_expl
    model_ac.x = st
    model_ac.xdot = st_dt
    model_ac.u = ctrl
    # model_ac.z = model.z
    # model_ac.p = model.p
    model_ac.name = "CrazyFlie21_model"
    ocp.model = model_ac

    # define constraint
    # model_ac.con_h_expr = constraint.expr

    # set dimensions
    nx = model_ac.x.size()[0]
    nu = model_ac.u.size()[0]
    ny = nx + nu
    ny_e = nx  # No terminal cost

    ocp.dims.N = N

    # set cost
    ocp.cost.cost_type = "EXTERNAL" #"LINEAR_LS"
    ocp.cost.cost_type_e = "EXTERNAL" #"LINEAR_LS"
    ocp.model.cost_expr_ext_cost = (st - targ_st).T @ Q @ (st - targ_st) + (ctrl - ref_u).T @ R @ (ctrl - ref_u)
    ocp.model.cost_expr_ext_cost_e = (st - targ_st).T @ Q_e @ (st - targ_st)

    # Bounds on decision variables
    
    # State bounds
    lbx = [0] * nx;         ubx = [0] * nx
    lbu = [0] * nu;         ubu = [0] * nu
    
    lbx[0] = -2;           ubx[0] = 2           # x lower, upper bounds
    lbx[1] = -2;           ubx[1] = 2           # y bounds
    lbx[2] = -2;           ubx[2] = 2          # z bounds
    lbx[3] = -10e15;        ubx[3] = 10e15       # qw bounds
    lbx[4] = -10e15;        ubx[4] = 10e15       # qx bounds
    lbx[5] = -10e15;        ubx[5] = 10e15       # qy bounds
    lbx[6] = -10e15;        ubx[6] = 10e15       # qz bounds
    lbx[7] = v_min;        ubx[7] = v_max      # u bounds
    lbx[8] = v_min;        ubx[8] = v_max      # v bounds
    lbx[9] = v_min;        ubx[9] = v_max      # w bounds
    lbx[10] = w_min;        ubx[10] = w_max     # p bounds
    lbx[11] = w_min;        ubx[11] = w_max     # q bounds
    lbx[12] = w_min;        ubx[12] = w_max     # r bounds
    
    # Control bounds
    lbu[0] = 0;         ubu[0] = max_rpm        # w1 bounds
    lbu[1] = 0;         ubu[1] = max_rpm        # w2 bounds
    lbu[2] = 0;         ubu[2] = max_rpm        # w3 bounds
    lbu[3] = 0;         ubu[3] = max_rpm        # w4 bounds
    
    # define initial conditions
    ocp.constraints.x0  = np.copy(init_st)
    
    # bounds on shooting nodes 0 - (N-1)s
    ocp.constraints.lbu = np.array(lbu)
    ocp.constraints.ubu = np.array(ubu)
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])

    ocp.constraints.lbx = np.array(lbx)
    ocp.constraints.ubx = np.array(ubx)
    ocp.constraints.idxbx = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12])#, 7, 8, 9, 10, 11, 12])
    print(f" DEBUG  bounds, lbx {ocp.constraints.lbx}, idx {ocp.constraints.idxbx}")

    # define constraints
    #ocp.constraints.lh = np.array([constraint.along_min])
    #ocp.constraints.uh = np.array([constraint.along_max ])

    # set QP solver and integration
    ocp.solver_options.tf = stepTime
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON" #"EXACT"
    ocp.solver_options.integrator_type = "IRK"
    # ocp.solver_options.nlp_solver_max_iter = 100
    # ocp.solver_options.tol = 1e-8  

    # create solver
    solve_json = "flight_ocp.json"
    acados_solver = AcadosOcpSolver(ocp, json_file = solve_json)
    acados_integrate = AcadosSimSolver(ocp, json_file = solve_json)

    return  ocp, acados_solver, acados_integrate

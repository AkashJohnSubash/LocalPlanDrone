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
    ny_e = nx  # No terminal cost

    ocp.dims.N = N

    # set cost
    ocp.cost.cost_type = "LINEAR_LS"                # TODO Implicitly LLS ?
    ocp.cost.cost_type_e = "LINEAR_LS"
    ocp.cost.W = scipy.linalg.block_diag(Q, R)
    ocp.cost.W_e = 50 * Q

    Vx = np.zeros((ny, nx))
    Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx = Vx

    Vu = np.zeros((ny, nu))
    Vu[13,0] = 1.0
    Vu[14,1] = 1.0
    Vu[15,2] = 1.0
    Vu[16,3] = 1.0
    ocp.cost.Vu = Vu

    Vx_e = np.zeros((ny_e, nx))
    Vx_e[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx_e = Vx_e

    # set intial references        # nx +  nu
    ocp.cost.yref = np.concatenate((targ_st, U_hov))
    ocp.cost.yref_e = np.array(targ_st) # No terminal cost


    # Bounds on decision variables
    
    # State bounds
    lbx = [0] * nx;         ubx = [0] * nx
    lbu = [0] * nu;         ubu = [0] * nu
    print(f" DEBUG init states constr, lbx {lbx}, ubx {ubx}")
    
    lbx[0] = 0;           ubx[0] = 100        # x lower, upper bounds
    lbx[1] = 0;           ubx[1] = 100        # y bounds
    lbx[2] = 0;           ubx[2] = 100          # z bounds
    lbx[3] = 0;           ubx[3] = 100        # qw bounds
    lbx[4] = 0;           ubx[4] = 100        # qx bounds
    lbx[5] = 0;           ubx[5] = 100        # qy bounds
    lbx[6] = 0;           ubx[6] = 100        # qz bounds
    lbx[7] = v_min;       ubx[7] = v_max      # u bounds
    lbx[8] = v_min;       ubx[8] = v_max      # v bounds
    lbx[9] = v_min;       ubx[9] = v_max      # w bounds
    lbx[10] = w_min;      ubx[10] = w_max     # p bounds
    lbx[11] = w_min;      ubx[11] = w_max     # q bounds
    lbx[12] = w_min;      ubx[12] = w_max     # r bounds
    
    # Control bounds
    lbu[0] = 0;         ubu[0] = max_rpm        # w1 bounds
    lbu[1] = 0;         ubu[1] = max_rpm        # w2 bounds
    lbu[2] = 0;         ubu[2] = max_rpm        # w3 bounds
    lbu[3] = 0;         ubu[3] = max_rpm        # w4 bounds
    
    ocp.constraints.x0  = model.x0
    
    # bounds on shooting nodes 0 - (N-1)s
    ocp.constraints.lbu = np.array(lbu)
    ocp.constraints.ubu = np.array(ubu)
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])

    ocp.constraints.lbx = np.array(lbx)
    ocp.constraints.ubx = np.array(ubx)
    ocp.constraints.idxbx = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12])

    # define constraints
    #ocp.constraints.lh = np.array([constraint.along_min])
    #ocp.constraints.uh = np.array([constraint.along_max ])

    # set QP solver and integration
    ocp.solver_options.tf = stepTime
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_max_iter = 100
    ocp.solver_options.tol = 1e-4   

    # create solver
    solve_json = "flight_ocp.json"
    acados_solver = AcadosOcpSolver(ocp, json_file = solve_json)
    acados_integrate = AcadosSimSolver(ocp, json_file = solve_json)

    return constraint, model, acados_solver, acados_integrate

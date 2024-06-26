# System overview
Drone : https://www.bitcraze.io/products/crazyflie-2-1/
Indoor Positioning : https://www.bitcraze.io/documentation/lighthouse/

# ControlLab

Problem Statement: Perform an overtake manoeuvre for a drone.(Deatailed MPC formulation in doc\report.tex)

Control :

    Setpoint tracking, with obstacle avoidance.
    Overtake trajectory points computed with MPC ( Python, Ipopt) online, offboard, closed loop. 
    Setpoints (RPYT) communicated (CrazyRadio) to onboard controller (EKF + PID) 

Obstacle handling : ( milestone based)

    Simulated, stationary, coordinates known at start of flight (statically generated)
    Simulated, stationary, added during flight (dynamically generated)
    Simulated, moving, dynamically generated.
    Ambitious milestones :
        Real, stationary (hanging object), dynamically detected.  (Multiranger, flow deck, local coordinates converted to global coordinates)
        Real, moving drone, dynamically detected during flight.

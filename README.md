# System overview
Drone : https://www.bitcraze.io/products/crazyflie-2-1/
Indoor Positioning : https://www.bitcraze.io/documentation/lighthouse/

# ControlLab
Problem Statement: Perform an overtake maneuver for an obstacle.

Control :

    Follow a straight trajectory, until an obstacle is generated/ detected.
    Overtake trajectory points generated offboard with MPC ( Python)
    Setpoints communicated (CrazyRadio) to onboard control loop (EKF + PID) 

Obstacle handling : ( milestone based)

    Simulated, stationary, coordinates (global from lighthouse) known at start of flight (statically generated)
    Simulated, stationary, added during flight (dynamically generated)
    Simulated, moving, dynamically generated.
    Ambitious milestones :
        Real, stationary (hanging object), dynamically detected.  (Multiranger, flow deck, local coordinates converted to global coordinates)
        Real, moving drone, dynamically detected during flight.

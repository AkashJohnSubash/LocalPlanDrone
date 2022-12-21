# ControlLab
Submodules cfClient, cfLib are updated for the speific problem

Problem Statement: Perform an overtake maneuver for an obstacle.

Control :

    Follow a straight trajectory, until an obstacle is generated/ detected.
    Overtake trajectory points generated offboard with MPC (MATLAB / Python)
    Setpoints communicated (CrazyRadio) to onboard control loop (EKF + PID) 

Obstacle handling : ( milestone based)

    Simulated, stationary, coordinates (global from lighthouse) known at start of flight (statically generated)
    Simulated, stationary, added during flight (dynamically generated)
    Simulated, moving, dynamically generated.
    Ambitious milestones :
        Real, stationary (hanging object), dynamically detected.  (Multiranger, flow deck, local coordinates converted to global coordinates)
        Real, moving drone, dynamically detected during flight.

Suggested TODOs (by Mohammad):
- [ ]  implement quadcopter dynamics
- [ ]  visualize dynamics  (input → output)
- [ ]  formulate MPC without constraints
- [ ]  add constraints to the MPC

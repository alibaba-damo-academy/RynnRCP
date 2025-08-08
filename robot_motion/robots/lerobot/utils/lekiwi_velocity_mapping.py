import numpy as np


def chassis_ikvelocity(chassis_vel):
    """
    Compute wheel velocities for LeKiwi omnidirectional chassis.
    
    Args:
        chassis_vel: Chassis velocity [vx, vy, omega] in chassis frame
                    - vx: forward velocity (m/s)
                    - vy: lateral velocity (m/s) 
                    - omega: angular velocity (rad/s)
    
    Returns:
        qdot: Wheel velocities [wheel1, wheel2, wheel3] (rad/s)
    """
    VEL_TO_WHEEL_COEFF = 20.0
    WHEEL_RADIUS = 0.1
    DEADBAND_THRESHOLD = 0.01
    
    jacobian = np.array([
        [0,                     1,    -WHEEL_RADIUS],  
        [-np.sqrt(3) * 0.5,    -0.5,  -WHEEL_RADIUS],  
        [ np.sqrt(3) * 0.5,    -0.5,  -WHEEL_RADIUS],  
    ])
    
    chassis_vel_reordered = np.array([chassis_vel[1], chassis_vel[2], chassis_vel[0]])
    qdot = VEL_TO_WHEEL_COEFF * (jacobian @ chassis_vel_reordered)
    qdot = np.where(
        np.abs(qdot) < DEADBAND_THRESHOLD, 
        0.0, 
        qdot
    )

    return qdot

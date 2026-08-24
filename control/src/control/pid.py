from __future__ import division
import numpy as np

from control.controller import BaseController
from control.controller import compute_position_in_frame


class PIDController(BaseController):
    def __init__(self, **kwargs):
        self.kp = kwargs.pop("kp")
        self.kd = kwargs.pop("kd")

        # Get the keyword args that we didn't consume with the above initialization
        super(PIDController, self).__init__(**kwargs)


    def get_error(self, pose, reference_xytv):
        """Compute the PD error.

        Args:
            pose: current state of the vehicle [x, y, heading]
            reference_xytv: reference state and speed

        Returns:
            error: across-track and cross-track error
        """
        return compute_position_in_frame(pose, reference_xytv[:3])

    def get_control(self, pose, reference_xytv, error):
        """Compute the PD control law.

        Args:
            pose: current state of the vehicle [x, y, heading]
            reference_xytv: reference state and speed
            error: error vector from get_error

        Returns:
            control: np.array of velocity and steering angle
                (velocity should be copied from reference velocity)
        """
        # BEGIN QUESTION 2.1
        # 1. Extract the reference velocity
        v_ref = reference_xytv[3]
        
        # 2. Extract the cross-track error (y-component of the error vector)
        e_y = error[1]
        
        # 3. Calculate the derivative of the cross-track error
        # Hint provided by prompt: v * sin(theta - theta_ref)
        theta = pose[2]
        theta_ref = reference_xytv[2]
        e_y_dot = v_ref * np.sin(theta - theta_ref)
        
        # 4. Compute the PD control law for the steering angle
        # We negate the terms because positive error (left of path) requires negative steering (turn right)
        steer = -self.kp * e_y - self.kd * e_y_dot
        
        # 5. Return the control vector [velocity, steering_angle]
        return np.array([v_ref, steer])
        # END QUESTION 2.1

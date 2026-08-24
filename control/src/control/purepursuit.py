from __future__ import division
import numpy as np

from control.controller import BaseController
from control.controller import compute_position_in_frame


class PurePursuitController(BaseController):
    def __init__(self, **kwargs):
        self.car_length = kwargs.pop("car_length")

        # Get the keyword args that we didn't consume with the above initialization
        super(PurePursuitController, self).__init__(**kwargs)


    def get_error(self, pose, reference_xytv):
        """Compute the Pure Pursuit error.

        Args:
            pose: current state of the vehicle [x, y, heading]
            reference_xytv: reference state and speed

        Returns:
            error: Pure Pursuit error
        """
        return compute_position_in_frame(reference_xytv[:3], pose)

    def get_control(self, pose, reference_xytv, error):
        """Compute the Pure Pursuit control law.

        Args:
            pose: current state of the vehicle [x, y, heading]
            reference_xytv: reference state and speed
            error: error vector from get_error

        Returns:
            control: np.array of velocity and steering angle
        """
        # BEGIN QUESTION 3.1
        # The reference velocity is the 4th element in reference_xytv
        v = reference_xytv[3]
        
        # According to the slide, [a, b]^T is the reference position 
        # transformed into the robot's local frame (which is our error vector)
        a = error[0]
        b = error[1]
        
        # Computing the Arc Radius (R_pp)
        # We add a check to prevent division by zero if the vehicle is 
        # perfectly aligned (b = 0)
        if abs(b) < 1e-6:
            delta = 0.0
        else:
            R_pp = (a**2 + b**2) / (2 * b)
            
            # Steering angle: delta = tan^-1(L / R)
            # Make sure to use your class's specific wheelbase variable (e.g., self.L)
            delta = np.arctan(self.car_length / R_pp)
            
        return np.array([v, delta])
        # END QUESTION 3.1

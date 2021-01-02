import numpy as np
from scipy.optimize import fsolve

class PathPlanner:

    @staticmethod
    def generate_pose_by_z_axis_while_constrain_x_axis_in_YZ_plane(z_axis):
        z = np.array(z_axis)
        '''
        R = [r1 r2 r3] =
        [0 c z1]
        [a d z2]
        [b e z3]
        '''
        def my_func(x):
            a = x[0]
            b = x[1]
            c = x[2]
            d = x[3]
            e = x[4]
            
            r1 = np.array([0, a, b])
            r2 = np.array([c, d, e])
            r3 = z / np.linalg.norm(z)
            
            F = np.empty((5))
            F[0] = np.linalg.norm(r1) - 1
            F[1] = np.linalg.norm(r2) - 1
            F[2:5] = np.cross(r1, r2) - r3
            
            return F

        x_guess = np.array([-1.0,0.1,-1.0,0.1,0.1]) # initial guess
        x = fsolve(my_func, x_guess)
        x = np.insert(x, 0, 0)
        x = x.tolist()

        # return the x-axis and y-axis
        return x[:3], x[3:]

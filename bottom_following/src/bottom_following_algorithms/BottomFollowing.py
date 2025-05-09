import numpy as np
import rospy
from math import pi, asin, sin, cos
from scipy.spatial.transform import Rotation as R
np.set_printoptions(precision=3, suppress=True)

class BottomFollowing:
    """
    This implements an estimator to obtain an estimate of the vector D.
    This is similar to a "sensor fusion" Kalman filter.
    """
    def __init__(self, h, attitude, alpha, kp, ki):
        self.alpha = alpha
        self.e=0
        # Gains for w0 = 0.05, ksi = 0.7
        self.kp = kp
        self.ki = ki
        self.int = 0

        # Initialize filter state and covariance
        self.xhat = self._compute_D(h, attitude)
        self.P = np.eye(3)

        # Filter parameters
        self.R = 10 * np.eye(3)
        self.Q = 0.01 * np.eye(3)

        #debug type shii
        self.n = np.array([None,None,None])
        self.D_B = np.array([None,None,None])
        

    def compute(self, h, body_velocity, attitude, Dt):
        """Perform the predict and update steps of the Kalman filter."""
        #####  ---  Predict Step  ---  #####
        # Rotate DVL velocity to body
        V = self._rot(attitude[0],attitude[1],attitude[2]) @ body_velocity
        # Project total inertial velocity on D
        D_dot = np.dot(V, self.xhat)/np.dot(self.xhat, self.xhat)*self.xhat

        # Propagate state forward
        self.xhat = self.xhat + Dt*D_dot
        self.P = self.P + self.Q


        #####  ---  Update Step  ---  #####
        # Compute noisy D from measurements
        D = self._compute_D(h, attitude)

        # Update step
        K = self.P @ np.linalg.inv(self.P + self.R)
        self.xhat = self.xhat + K @ (D - self.xhat)
        self.P = (np.eye(3) - K) @ self.P


    def controller(self, d_ref, u_ref, heading_ref, u_max):
        D = self.xhat.reshape((3,1))
        n = (D / np.linalg.norm(D))
        
        ##########  Attitude Control  #####
        z_B_des = n
        xc = np.array([cos(heading_ref), sin(heading_ref),0]).reshape((3,1))
        y_B_des = np.cross(n.T, xc.T).T
        y_B_des = y_B_des/np.linalg.norm(y_B_des)
        x_B_des = np.cross(y_B_des.T, z_B_des.T).T
        R_des = np.hstack((x_B_des, y_B_des, z_B_des))
        
        # Compute euler angles from rotation matrix
        attitude_ref = R.from_matrix(R_des).as_euler('xyz', degrees=False)
        
        
        #########  Velocity Control  #######
        # ___  Parallel velocity  ___ #
        
        # Desired velocity vector from the path following in inertial frame
        # V_I_ref = np.array([u_ref*cos(heading_ref),u_ref*sin(heading_ref),0]).reshape((3,1))
        # # Projection operator
        # P_D = np.eye(3) - (D*D.T)/(np.linalg.norm(D)**2)
        # # Project horizontal reference velocity on the plane tangent to the slope
        # V_I_ref_p  = P_D @ V_I_ref
        # # normalize velocity to lenght equal to surge ref
        # V_I_ref_p = V_I_ref_p / np.linalg.norm(V_I_ref_p)
        V_P = u_ref*x_B_des

        #####  ---  Normal velocity  ---  #####
        # Compute error
        self.e = d_ref - np.linalg.norm(D)
        # Velocity component normal to the plane tangent to the terrain
        V_D = - self.kp * self.e * (D / np.linalg.norm(D))

        # Total velocity
        V_T_ref = V_D + V_P
        # Saturate velocity norm
        if np.linalg.norm(V_T_ref) > u_max:
            V_T_ref = V_T_ref / np.linalg.norm(V_T_ref) *u_max
            
        # ____  Attitude Control  ____ #
        z_B_des = n
        xc = np.array([cos(heading_ref), sin(heading_ref),0]).reshape((3,1))
        y_B_des = np.cross(n.T, xc.T).T
        y_B_des = y_B_des/np.linalg.norm(y_B_des)
        x_B_des = np.cross(y_B_des.T, z_B_des.T).T
        R_des = np.hstack((x_B_des, y_B_des, z_B_des))
        
        # Compute euler angles from rotation matrix
        attitude_ref = R.from_matrix(R_des).as_euler('xyz', degrees=False) 

        return V_T_ref, attitude_ref

    #TODO: if using DVL in another orientation, this should be changed. should be implemented better but ...
    def _compute_D(self, h, attitude):
        # Compute matrix with all range measurements in the body frame
        H_b = np.zeros((3,5))
        H_b[:,0] = [sin(self.alpha)*h[0],0,cos(self.alpha)*h[0]]
        H_b[:,1] = [0,sin(-self.alpha)*h[1],cos(-self.alpha)*h[1]]
        H_b[:,2] = [sin(-self.alpha)*h[2],0,cos(-self.alpha)*h[2]]
        H_b[:,3] = [0,sin(self.alpha)*h[3],cos(self.alpha)*h[3]]
        H_b[:,4] = [0,0,h[4]]

        # Compute and remove centroid
        h_c = (np.sum(H_b, axis=1)/5).reshape((3,1))
        H_0 = H_b-h_c

        # Compute SVD and extract plane normal
        U, Σ, V = np.linalg.svd(H_0)
        n = U[:, -1]
        self.n = n

        # Compute the vector that points from the origin to the closest point on the plane
        D_B = abs(n.T @ h_c)/(np.linalg.norm(n)**3) * n
        self.D_B = D_B

        # Convert to inertial frame orientation
        D_I = self._rot(attitude[0],attitude[1],attitude[2]) @ D_B

        return D_I


    """Compute the 3D rotation matrix from roll, pitch, and yaw (in radians)."""
    @staticmethod
    def _rot(roll, pitch, yaw):
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        R = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,                cp * cr]
        ])

        return R


    @staticmethod
    def _sat(value):
        if value > 1:
            return 1
        if value <-1:
            return -1
        return value


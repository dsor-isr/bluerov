import numpy as np
import rospy 
from math import pi, asin

class BottomFollowing:
    """
    This implements an estimator to obtain an estimate of the vector D.
    This is similar to a "sensor fusion" Kalman filter.
    """
    
    def __init__(self, y0, alpha):
        self.alpha = alpha
        self.e=0
        
        # Compute D from initial measurement
        h1 = np.array([[0, y0[0]]]).T
        h2 = self._rot(pi/2-alpha) @ np.array([[y0[1],0]]).T
        S = h2 - h1
        Ps = np.eye(2) - np.outer(S, S) / np.linalg.norm(S)**2
        D = Ps @ h1
        
        # Initialize filter state and covariance
        self.xhat = np.vstack((D, np.zeros((2,1))))
        self.P = np.eye(4)
        
        # Filter parameters
        self.R = 10 * np.eye(4)
        self.Q = 0.01 * np.eye(4)
        
        self.debug = np.zeros(1)
        self.D_dot = np.zeros((2,1))
    
    def compute(self, u, y, pitch, Dt):
        """Perform the predict and update steps of the Kalman filter."""
        # Predict Step
        Ak = np.kron(np.array([[1, Dt], [0, 1]]), np.eye(2))
        self.xhat = Ak @ self.xhat
        self.P = Ak @ self.P @ Ak.T + self.Q
        
        #TODO: add pitch
        # Compute D
        h1 = self._rot(pitch) @ np.array([[0, y[0]]]).T
        h2 = self._rot(pitch) @ self._rot(pi/2-self.alpha) @ np.array([[y[1],0]]).T
        S = h2 - h1
       # Ps = np.eye(2) - np.outer(S, S) / np.linalg.norm(S)**2
        Ps = np.eye(2) - (S@S.T) / np.linalg.norm(S)**2
        D = Ps @ h2
        
        # Compute D_dot
        S1 = np.array([[self.xhat[1], -self.xhat[0]]]).T
        S1 /= np.linalg.norm(S1)
        Ps1 = np.eye(2) - np.outer(S1, S1) / np.linalg.norm(S1)**2
        Ddot = -Ps1 @ u
        
        # Build measurement vector
        y_meas = np.vstack((D, Ddot))   
        
        # Update step
        K = self.P @ np.linalg.inv(self.P + self.R)
        self.xhat = self.xhat + K @ (y_meas - self.xhat)
        self.P = (np.eye(4) - K) @ self.P
        
        # Store D_dot
        self.D_dot = self.xhat[2:4]
    
    def controller(self, d_ref, U, u_max):
        """Implements a simple nonlinear controller based on Lyapunov theory."""
        Kp = 0.1  # Controller gain
        D = self.xhat[:2]  # Extract D
        S = np.array([self.xhat[1], -self.xhat[0]])  # Rotate D 90º anti-clockwise
        S /= np.linalg.norm(S)  # Normalize
        e = d_ref - np.linalg.norm(D)  # Compute error
        

        V =  U * S - Kp * e * D / np.linalg.norm(D)
        if np.linalg.norm(V) > u_max:
            V = V / np.linalg.norm(V) *u_max
        print(V)
        self.e=e
        return V
    
    @staticmethod
    def _rot(angle):
        """Return a 2D rotation matrix for the given angle in radians."""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[c, -s], [s, c]])

    @staticmethod
    def _sat(value):
        if value > 1:
            return 1
        if value <-1:
            return -1
        return value
    

################################## MEMES #########################################
    
        #     gamma = asin(self._sat(-Kp*e))
        # V = self._rot(gamma)@S
        # print(V)
        
        
        
        
        #         ksi =  U * S - Kp * e * D  # Desired velocity
        # V = U * ksi / np.linalg.norm(ksi)  # Normalized velocity
import numpy as np

class BottomFollowing:
    """
    This implements an estimator to obtain an estimate of the vector D.
    This is similar to a "sensor fusion" Kalman filter.
    """
    
    def __init__(self, y0, alpha):
        self.alpha = alpha
        
        # Compute D from initial measurement
        h1 = np.array([0, y0[0]])
        h2 = self._rot(-alpha) @ np.array([0, y0[1]])
        S = h2 - h1
        Ps = np.eye(2) - np.outer(S, S) / np.linalg.norm(S)**2
        D = Ps @ h1
        
        # Initialize filter state and covariance
        self.xhat = np.hstack((D, [0, 0]))
        self.P = np.eye(4)
        
        # Filter parameters
        self.R = 10 * np.eye(4)
        self.Q = 0.01 * np.eye(4)
        
        self.debug = np.zeros(1)
        self.D_dot = np.zeros(2)
    
    def compute(self, u, y, Dt):
        """Perform the predict and update steps of the Kalman filter."""
        # Predict Step
        Ak = np.kron(np.array([[1, Dt], [0, 1]]), np.eye(2))
        self.xhat = Ak @ self.xhat
        self.P = Ak @ self.P @ Ak.T + self.Q
        
        # Update Step
        h1 = np.array([0, y[0]])
        h2 = self._rot(-self.alpha) @ np.array([0, y[1]])
        S = h2 - h1
        Ps = np.eye(2) - np.outer(S, S) / np.linalg.norm(S)**2
        D = Ps @ h1
        
        # Compute D_dot
        S1 = np.array([self.xhat[1], -self.xhat[0]])
        S1 /= np.linalg.norm(S1)
        Ps1 = np.eye(2) - np.outer(S1, S1) / np.linalg.norm(S1)**2
        Ddot = -Ps1 @ u
        
        # Build measurement vector
        y_meas = np.hstack((D, Ddot))
        
        # Update step
        K = self.P @ np.linalg.inv(self.P + self.R)
        self.xhat = self.xhat + K @ (y_meas - self.xhat)
        self.P = (np.eye(4) - K) @ self.P
        
        # Store D_dot
        self.D_dot = self.xhat[2:4]
    
    def controller(self, d_ref, U, X):
        """Implements a simple nonlinear controller based on Lyapunov theory."""
        Kp = 1  # Controller gain
        D = self.xhat[:2]  # Extract D
        S = np.array([self.xhat[1], -self.xhat[0]])  # Rotate D 90º anti-clockwise
        S /= np.linalg.norm(S)  # Normalize
        e = d_ref - np.linalg.norm(D)  # Compute error
        ksi = S - Kp * e * D  # Desired velocity
        V = U * ksi / np.linalg.norm(ksi)  # Normalized velocity

        return V
    
    @staticmethod
    def _rot(angle):
        """Return a 2D rotation matrix for the given angle in radians."""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[c, -s], [s, c]])

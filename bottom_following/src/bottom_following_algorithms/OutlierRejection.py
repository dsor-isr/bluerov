import numpy as np

class OutlierRejection:
    """
    Function to perform outlier rejection on a signal vector y of dimension n.
    This is implemented as a class to store the buffer.
    It works by applying a MAD criterion to the data on a rolling window.
    For reference, see Morgado et al., 2014.
    """
    
    def __init__(self, K, threshold, n):
        self.K = K
        self.buffer = np.full((n, K), np.nan)  # Initialize buffer for all channels
        self.idx = 0
        self.count = 0
        self.threshold = threshold
        self.debug = np.zeros(2)
    
    def compute(self, y):
        y_filtered = np.copy(y)
        n = y.shape[0]
        
        # Update buffer
        self.buffer[:, self.idx] = y
        self.idx = (self.idx + 1) % self.K
        self.count = min(self.count + 1, self.K)
        
        # Check if we have enough samples
        if self.count < self.K:
            return y_filtered
        
        # Compute median and MAD
        medians = np.nanmedian(self.buffer, axis=1)
        mad_values = np.nanmedian(np.abs(self.buffer - medians[:, None]), axis=1) / 0.6745
        
        # Identify outliers
        robust_z_scores = np.abs(y - medians) / mad_values
        self.debug = medians
        is_outliers = robust_z_scores > self.threshold
        
        if np.any(is_outliers):
            print("Outlier rejected")
        
        # Replace outliers with median
        y_filtered[is_outliers] = medians[is_outliers]
        
        return y_filtered

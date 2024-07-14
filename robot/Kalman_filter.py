class KalmanFilter:
    """
    Implements a simple Kalman filter for 1D data.
    
    Attributes:
        Q (float): Process noise covariance.
        R (float): Measurement noise covariance.
        P_k_k1 (float): Predicted estimate covariance.
        Kg (float): Kalman gain.
        P_k1_k1 (float): Updated estimate covariance.
        x_k_k1 (float): Predicted state.
        ADC_OLD_Value (float): Previous ADC value.
        Z_k (float): Current measurement.
        kalman_adc_old (float): Previous filtered value.
    """
    
    def __init__(self, Q, R):
        """
        Initializes the Kalman filter with the specified process and measurement noise covariances.
        
        Args:
            Q (float): Process noise covariance.
            R (float): Measurement noise covariance.
        """
        self.Q = Q
        self.R = R
        self.P_k_k1 = 1
        self.Kg = 0
        self.P_k1_k1 = 1
        self.x_k_k1 = 0
        self.ADC_OLD_Value = 0
        self.Z_k = 0
        self.kalman_adc_old = 0
        
    def kalman(self, ADC_Value):
        """
        Applies the Kalman filter to a new ADC measurement.
        
        Args:
            ADC_Value (float): The new ADC measurement.
        
        Returns:
            float: The filtered ADC value.
        """
        self.Z_k = ADC_Value
        
        # Check for significant change in ADC value
        if abs(self.kalman_adc_old - ADC_Value) >= 60:
            self.x_k1_k1 = ADC_Value * 0.382 + self.kalman_adc_old * 0.618
        else:
            self.x_k1_k1 = self.kalman_adc_old
    
        self.x_k_k1 = self.x_k1_k1
        self.P_k_k1 = self.P_k1_k1 + self.Q
        self.Kg = self.P_k_k1 / (self.P_k_k1 + self.R)
    
        kalman_adc = self.x_k_k1 + self.Kg * (self.Z_k - self.kalman_adc_old)
        self.P_k1_k1 = (1 - self.Kg) * self.P_k_k1
        self.P_k_k1 = self.P_k1_k1
    
        self.kalman_adc_old = kalman_adc
        
        return kalman_adc

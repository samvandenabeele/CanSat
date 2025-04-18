class KalmanFilter:
    def __init__(self, process_noise=0.01, measurement_noise=0.1, estimation_error=1.0):
        self.q = process_noise           # Process noise
        self.r = measurement_noise       # Measurement noise
        self.p = estimation_error        # Estimation error
        self.x = 0.0                     # Initial estimate

    def update(self, measurement):
        k = self.p / (self.p + self.r)               # Kalman gain
        self.x = self.x + k * (measurement - self.x) # Update estimate
        self.p = (1 - k) * self.p + self.q           # Update error covariance
        return self.x
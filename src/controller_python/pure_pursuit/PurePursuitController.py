import numpy as np

class PurePursuitController:
    def __init__(self, lookahead_distance=5):
        self.lookahead_distance = lookahead_distance
        self.xRef = None
        self.yRef = None
        self.IntegralError = 0
        self.previous_time = None

    def update_path(self, xRef, yRef):
        if not isinstance(xRef, np.ndarray) or not isinstance(yRef, np.ndarray):
            raise ValueError("xRef and yRef should be numpy arrays")
        self.xRef = np.array(xRef)
        self.yRef = np.array(yRef)

    def compute_target(self, X, Y, Theta, vel, clk):
        # Find the closest point on the path
        distances = np.sqrt((self.xRef - X)**2 + (self.yRef - Y)**2)
        closest_idx = np.argmin(distances)

        # Find the target point along the path based on the lookahead distance
        target_idx = closest_idx
        while target_idx < len(self.xRef) and np.linalg.norm([self.xRef[target_idx] - X, self.yRef[target_idx] - Y]) < self.lookahead_distance:
            target_idx += 1

        if target_idx >= len(self.xRef):
            target_idx = len(self.xRef) - 1

        # Calculate the angle to the target point
        target_x = self.xRef[target_idx]
        target_y = self.yRef[target_idx]
        target_angle = np.arctan2(target_y - Y, target_x - X)

        # Determine the steering direction
        heading_error = target_angle - Theta
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))  # Normalize the angle

        # Calculate crosstrack error and integrate if time has been initialized
        if self.previous_time is not None:
            time_diff = clk - self.previous_time
            self.IntegralError += (np.sin(heading_error) * vel * time_diff)
            self.IntegralError = np.clip(self.IntegralError, -0.1, 0.1)  # Limit the integral term to avoid windup

        self.previous_time = clk

        # Calculate the desired steering angle (Target Dir)
        Target_Dir = heading_error + 0.1 * self.IntegralError
        Target_Dir = np.clip(Target_Dir, -np.pi/4, np.pi/4)  # Clip the steering angle to reasonable limits

        return Target_Dir

    def reset_integral_error(self):
        self.IntegralError = 0
        self.previous_time = None

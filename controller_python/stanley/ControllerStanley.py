import numpy as np

class StanleyController:
    def __init__(self):
        self.startindex = 1
        self.angledir = 0
        self.IntegralError = 0
        self.time = [0]

    def update_path(self, xRef, yRef):
        self.xRef = xRef
        self.yRef = yRef

    def controller_stanley(self, X, Y, Theta, vel, clk):
        # Reference point for Stanley: front axle
        X += 0.75215 * np.cos(Theta)
        Y += 0.75215 * np.sin(Theta)
        
        i = self.startindex
        steering_angle = self.angledir
        heading_error = 0
        crosstrack_error = 0
        dist = 0

        while i < len(self.xRef) - 1:
            dist_prec = np.sqrt((X - self.xRef[i])**2 + (Y - self.yRef[i])**2)
            dist_succ = np.sqrt((X - self.xRef[i + 1])**2 + (Y - self.yRef[i + 1])**2)
            if dist_succ > dist_prec:
                dist = np.abs((self.xRef[i + 1] - self.xRef[i]) * (self.yRef[i] - Y) -
                              (self.xRef[i] - X) * (self.yRef[i + 1] - self.yRef[i])) / \
                       np.sqrt((self.xRef[i + 1] - self.xRef[i])**2 + (self.yRef[i + 1] - self.yRef[i])**2)
                
                anglePosRef = np.arctan2(Y - self.yRef[i], X - self.xRef[i])
                angpath = np.arctan2(self.yRef[i + 1] - self.yRef[i], self.xRef[i + 1] - self.xRef[i])
                diffangle = np.arctan2(np.sin(anglePosRef - angpath), np.cos(anglePosRef - angpath))
                
                if diffangle <= 0:
                    dist = abs(dist)
                else:
                    dist = -abs(dist)
                
                heading_error = np.arctan2(np.sin(angpath - Theta), np.cos(angpath - Theta))
                crosstrack_error = np.arctan(110 * dist / (0 + vel))

                self.time.append(clk)
                self.IntegralError += dist * (self.time[-1] - self.time[-2])
                self.IntegralError = np.clip(self.IntegralError, -0.02, 0.02)

                self.angledir = (1.2 * heading_error + crosstrack_error + 1.5 * self.IntegralError)
                self.angledir = np.clip(self.angledir, -3.14, 3.14)
                steering_angle = self.angledir

                self.startindex = max(1, i - 2)
                break
            i += 1

        return steering_angle
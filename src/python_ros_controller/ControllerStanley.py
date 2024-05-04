import numpy as np

class ControllerStanley:
    def __init__(self):
        self.startindex = 0
        self.angledir = 0
        self.IntegralError = 0
        self.time = [0]
        self.counter = 0
        self.xRef = None
        self.yRef = None

    def update_path(self, xRef, yRef):
        self.xRef = xRef
        self.yRef = yRef

    def controller_stanley(self, X, Y, Theta, vel, clk):
        X += 0.7675 * np.cos(Theta)
        Y += 0.7675 * np.sin(Theta)
        steering_angle = self.angledir
        i = self.startindex
        heading_error = 0
        cross_track_error = 0
        dist = 0
        Distance_from_Centerline = 0

        while i < (len(self.xRef) - 1):
            dist_prec = np.sqrt((X - self.xRef[i])**2 + (Y - self.yRef[i])**2)
            dist_succ = np.sqrt((X - self.xRef[i + 1])**2 + (Y - self.yRef[i + 1])**2)
            if dist_succ > dist_prec:
                # Point-Line Distance
                dist = np.abs((self.xRef[i + 1] - self.xRef[i]) * (self.yRef[i] - Y) - (self.xRef[i] - X) * (self.yRef[i + 1] - self.yRef[i])) / np.sqrt((self.xRef[i + 1] - self.xRef[i])**2 + (self.yRef[i + 1] - self.yRef[i])**2)
                
                # Reference point (intersection of reference and line segment)
                numerator = (-self.xRef[i + 1]*self.yRef[i]*self.yRef[i + 1] + self.xRef[i + 1]*self.yRef[i]**2 + self.xRef[i]*self.yRef[i + 1]**2 - self.xRef[i]*self.yRef[i]*self.yRef[i + 1] + self.xRef[i + 1]*Y*self.yRef[i + 1] - self.xRef[i]*Y*self.yRef[i + 1] - self.xRef[i + 1]*Y*self.yRef[i] + self.xRef[i]*Y*self.yRef[i] + self.xRef[i + 1]**2*X - 2*self.xRef[i + 1]*self.xRef[i]*X + self.xRef[i]**2*X)
                denominator = ((self.yRef[i + 1] - self.yRef[i])**2 + (self.xRef[i + 1] - self.xRef[i])**2)
                xpoint = numerator / denominator
                ypoint = ((self.yRef[i + 1] - self.yRef[i]) / (self.xRef[i + 1] - self.xRef[i])) * xpoint + self.yRef[i] - ((self.yRef[i + 1] - self.yRef[i]) / (self.xRef[i + 1] - self.xRef[i])) * self.xRef[i]

                anglePosRef = np.arctan2(Y - ypoint, X - xpoint)
                diff_angle = np.arctan2(np.sin(anglePosRef - np.arctan2(self.yRef[i + 1] - self.yRef[i], self.xRef[i + 1] - self.xRef[i])), np.cos(anglePosRef - np.arctan2(self.yRef[i + 1] - self.yRef[i], self.xRef[i + 1] - self.xRef[i])))
                if diff_angle <= 0:
                    dist = abs(dist)
                else:
                    dist = -abs(dist)

                Ke = 1
                KeT = 1
                Kv = 0
                Ki = 0

                ang_path = np.arctan2(self.yRef[i + 1] - self.yRef[i], self.xRef[i + 1] - self.xRef[i])
                heading_error = np.arctan2(np.sin(ang_path - Theta), np.cos(ang_path - Theta))
                cross_track_error = np.arctan(Ke * dist / (Kv + vel))

                self.time.append(clk)

                self.IntegralError += dist * (self.time[-1] - self.time[-2])
                self.IntegralError = max(min(self.IntegralError, 0.05), -0.05)
                self.angledir = KeT * heading_error + cross_track_error + Ki * self.IntegralError

                self.angledir = max(min(self.angledir, np.pi), -np.pi)

                steering_angle = self.angledir
                self.startindex = i
                break

            i += 1
        return steering_angle

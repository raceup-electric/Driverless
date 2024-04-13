import math
import csv
from time import sleep

class IMU:
        
    def __init__(self, t, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z):
        self.timestamp = float(t)
        self.accelerometer = (float(accel_x), float(accel_y), float(accel_z))
        self.gyroscope = (float(gyro_x), float(gyro_y), float(gyro_z))
        
        
    def get_acc(self):
        ax = self.accelerometer[0]
        ay = self.accelerometer[1] 
        az = self.accelerometer[2] 
        return [ax, ay, az]
        
    def get_acc_angles(self):
        [ax, ay, az] = self.get_acc()
        phi = math.atan2(ay, math.sqrt(ax ** 2.0 + az ** 2.0))
        theta = math.atan2(-ax, math.sqrt(ay ** 2.0 + az ** 2.0))
        return [phi, theta]
    
    def get_gyro(self):
        gx = self.gyroscope[0]
        gy = self.gyroscope[1]
        gz = self.gyroscope[2] 
        return [gx, gy, gz]        
    
    def from_csv(self, file_name):
        with open(file_name, mode='r') as file:
            csv_reader = csv.reader(file)
            next(csv_reader, None)
            for lines in csv_reader:
                self.timestamp = float(lines[0])
                self.accelerometer = (float(lines[1]), float(lines[2]), float(lines[3]))
                self.gyroscope = (float(lines[4]), float(lines[5]), float(lines[6]))               
        





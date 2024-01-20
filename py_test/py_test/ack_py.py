import os
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

# msg type to control car
from ackermann_msgs.msg import AckermannDriveStamped



class control_test(Node):

    def __init__(self):
        super().__init__('oppele')
        #self.node = rclpy.create_node("oppele")
        self._publisher = self.create_publisher(AckermannDriveStamped,'/cmd', 1)
        timer_period = 1.0  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0

        val = input("set velocity (now dummy): ")
        print(val)
        self._send_ackermann_drive_stamped(float(val), 0.)


    def _send_ackermann_drive_stamped(self, linear, angular):
            if self._publisher is None:
                return

            drive = AckermannDriveStamped()
            drive.header.stamp = self.get_clock().now().to_msg()

            drive.drive.acceleration = 0.0
            drive.drive.speed = 0.0
            #if self.command_mode == "velocity":
            drive.drive.speed = linear
            #elif self.command_mode == "acceleration":
            #drive.drive.acceleration = linear

            drive.drive.steering_angle = angular
            drive.drive.steering_angle_velocity = 0.0

            # Only send the zero command once so other devices can take control
            #if linear == 0 and angular == 0:
                #if not self.zero_cmd_sent:
                    #self.zero_cmd_sent = True
                    #self._publisher.publish(drive)
            #else:
            self.zero_cmd_sent = False
            self._publisher.publish(drive)



def main(args=None):
    #print('Hi from py_test.')
    rclpy.init(args=args)
    control_test()
    #rclpy.spin(node)
    


if __name__ == '__main__':
    main()

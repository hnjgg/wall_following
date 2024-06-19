# wall following node

import rclpy
from rclpy.node import Node
import serial
import math

import numpy as np
from sensor_msgs.msg import LaserScan
#from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # create laser topic subscriber
        self.subscriber = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.scan_callback,
            10)
        self.subscriber # prevent unused variable warning

        '''
        # create drive topic publisher
        self.publisher = self.create_publisher(
            AckermannDriveStamped, drive_topic, 10
        )
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.pid_control)
        self.publisher # prevent unused variable warning
        '''
        
        # create serial object
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        # set PID gains
        self.kp = 14
        self.kd = 0.09
        self.ki = 0

        # init history variables
        self.integral = 0
        self.prev_error = 0
        self.error = 0

        # init lidar variable
        self.frontDistLiDAR = 0.0   
        self.rightDistLiDAR = 0.0  
        self.leftDistLiDAR = 0.0

        # constants
        self.L_lookAhead = 1.5          # meters
        self.throttle_max = 60          # fastest throttle
        self.steering_max = 1           # rightest steering
        self.LiDAR_detect_max = 16      # meters
        self.refDistToRightWall = 0.2   # meters
        self.distPoint1 = -55           # degree
        self.distPoint2 = -90           # degree

    def toRadian(self, degree):
        """
        Convert angle to radian from degree
        """
        return float(math.pi * degree / 180.0)


    def get_range(self, msg, angle):
        """
        Get range[] data to desired angle[rad]
        """
        idx = int((angle - msg.angle_min) / msg.angle_increment)
        distance = msg.ranges[idx]

        # saturation
        if(math.isinf(distance) or math.isnan(distance)):
            distance = self.LiDAR_detect_max

        print("dist from ", round(angle/math.pi * 180), ": ", round(distance,2))
        return distance

    def get_error(self, a, b, theta):
        """
        Calculates the error to the wall. Follow the wall on the right
        """
        theta = self.toRadian(theta)
        alpha = math.atan((a * math.cos(theta) - b)/(a*math.sin(theta)))
        currDistToRightWall = b*math.cos(alpha)

        currErr = self.refDistToRightWall - currDistToRightWall
        print("theta: ", round(theta/math.pi * 180), " alpha: ", round(alpha/math.pi * 180))
        print("curr dist: ", currDistToRightWall, " dist err: ", currErr)

        # advanced
        aheadDistToRightWall = currDistToRightWall + self.L_lookAhead * math.sin(alpha)

        return currErr

    def pid_control(self, error):
        """
        Based on the calculated error, publish vehicle control
        """

        self.integral = self.integral + error

        # make a steering control input
        PIDdist = (self.kp * error + self.kd * (error - self.prev_error) + self.ki * self.integral)
        print("PID calculated dist: ", round(PIDdist,2))
        self.prev_error = error 

        max_PIDdist = self.kp * self.refDistToRightWall
        min_PIDdist = self.kp * (self.refDistToRightWall - self.LiDAR_detect_max*math.cos(math.atan((self.LiDAR_detect_max * math.cos(abs(self.distPoint1 - self.distPoint2)) - self.LiDAR_detect_max)/(self.LiDAR_detect_max*math.sin(abs(self.distPoint1 - self.distPoint2))))))     
            
        print("max PIDdist: ", max_PIDdist, " min PIDdist: ", min_PIDdist) 
        steeringPWM = self.calculate_pwm(PIDdist, max_PIDdist, min_PIDdist)
        print("steeringPWM: ", steeringPWM)
        
        # determine throttle speed depends on the amount of steering control input 
        if PIDdist >= abs(self.toRadian(0)) and PIDdist < abs(self.toRadian(10)):
            velocityPWM = 140 #self.throttle_max
        elif PIDdist >= abs(self.toRadian(10)) and PIDdist < abs(self.toRadian(20)):
            velocityPWM = 100 #self.throttle_max / 2
        else:
            velocityPWM = 60 #self.throttle_max / 4

        '''
        drive_msg = AckermannDriveStamped()
        drive_msg.steering_angle = PIDdist
        drive_msg.speed = velocity
        self.publisher.publish(drive_msg)
        '''

        return int(steeringPWM), int(0)
        
    def calculate_pwm(self, PIDdist, max_PIDdist, min_PIDdist):
        max_pwm = 140    # max turn right
        min_pwm = 60     # max turn left
        mid_pwm = 100    # go straight
         
        # go left
        if(PIDdist > 0):
            pwm = -((mid_pwm - min_pwm) / max_PIDdist) * PIDdist + mid_pwm      
            print("=== go left ===")  
        # go right
        elif(PIDdist <= 0):
            pwm = ((max_pwm - mid_pwm) / -max_PIDdist) * PIDdist + mid_pwm   
            print("=== go right ===")
        
        print("pwm w/o saturation: ", pwm)

        # saturation        
        if(pwm < 60): pwm = min_pwm
        elif(pwm > 140): pwm = max_pwm
        
        return int(pwm)
    
    def send_control_to_car(self, steering, throttle):
        self.ser.write(bytearray([throttle, steering])) # send throttle, steering throguth serial com.
        self.get_logger().info('Sending pwm: Throttle=%d, Steer=%d\n' % (throttle, steering))

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.
        """
        a = self.get_range(msg, self.toRadian(self.distPoint1))
        b = self.get_range(msg, self.toRadian(self.distPoint2))
        theta = abs(self.distPoint1 - self.distPoint2)

        # calc dist error
        error = self.get_error(a, b, theta)

        # make control input
        PIDdist, velocity = self.pid_control(error) 

        # drive the car 
        self.send_control_to_car(PIDdist, velocity)

        

def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
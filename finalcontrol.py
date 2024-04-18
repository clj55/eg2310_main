import rclpy
from rclpy.node import Node
import geometry_msgs.msg
import RPi.GPIO as GPIO
from time import sleep, time
import requests
from threading import Timer
from nav_msgs.msg import Odometry
import math
import cmath
import numpy as np
import datetime 
import ast
import time
from std_msgs.msg import String

# setting up all the GPIO for functionality
GPIO.setmode(GPIO.BCM)
ir1_pin = 27
ir2_pin = 17
servo1_pin = 21
servo2_pin = 25

GPIO.setup(ir1_pin, GPIO.IN)
GPIO.setup(ir2_pin, GPIO.IN)
GPIO.setup(servo1_pin, GPIO.OUT)
GPIO.setup(servo2_pin, GPIO.OUT)

s1 = GPIO.PWM(servo1_pin, 50)
s2 = GPIO.PWM(servo2_pin, 50)



def delay(seconds):
    start_time = datetime.datetime.now()
    target_time = start_time + datetime.timedelta(seconds=seconds)
    while datetime.datetime.now() < target_time:
        pass


def convert_ang_dc(angle):
    dc = 2.5 + angle / 18 
    return dc

def euler_from_quaternion(x, y, z, w):
    # """
    # Convert a quaternion into euler angles (roll, pitch, yaw)
    # roll is rotation around x in radians (counterclockwise)
    # pitch is rotation around y in radians (counterclockwise)
    # yaw is rotation around z in radians (counterclockwise)
    # """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians


# function to check if keyboard input is a number as
# isnumeric does not handle negative numbers
def isnumber(value):
    try:
        int(value)
        return True
    except ValueError:
        return False


# Function: Request HTTP
esp32_ip = '172.20.10.9'
turtleBot_ID = 43
header = {'Content-Type': 'application/json'}

# Class 1: Functionality of the turtleBot3 Burger
class Func(Node):
    # initialiser
    def __init__(self, ip_address, ID):
        super().__init__('moverotate')
        super().__init__('mover')
        self.publisher_ = self.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel', 10)
        self.modepublisher = self.create_publisher(String, 'mode', 10)
        self.ip_address = ip_address
        self.ID = ID
        self.subscription = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        # self.get_logger().info('Created subscriber')
        self.subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.activate = True
        

    def odom_callback(self, msg):
        # self.get_logger().info(msg)
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)

    # Function 1: Line_follower
    def line_follower(self, cross_line, cross_line_second, reduce_rate = 0.999, turning_rate = 1.3, cent_per_sec = -0.1, black_line_forward = -0.02):
        twist = geometry_msgs.msg.Twist()
        rad_per_sec = 0.2
        linear_increase_rate = 1.01
        linear_decrease_rate = 0.1
        counter11 = 0
        counter10 = 0
        counter01 = 0
        counter = 0
        buffer = 11

        try:
            while self.activate and counter < cross_line:
                ir1 = GPIO.input(ir1_pin)
                ir2 = GPIO.input(ir2_pin)
                print(f'forward: {twist.linear.x:10.5f} | turn: {twist.angular.z:10.5f} | ir1: {ir1} | ir2: {ir2}')
               # twist.linear.x = cent_per_sec
                self.publisher_.publish(twist)

                if ir1 == 1 and ir2 == 1:
                    counter10 = counter01 = 0
                    counter11 += 1
                    if counter11 >= buffer * 10:
                        #counter -= 1
                        counter11 = 0
                        twist.angular.z = 0.0
                        twist.linear.x = black_line_forward
                        self.publisher_.publish(twist)
                        delay(cross_line_second)
                        twist.linear.x = 0.0
                        self.publisher_.publish(twist)
                        counter += 1
           

                    else:
                        continue

                elif ir1 == 1 and ir2 == 0:
                    counter11 = counter01 = 0
                    counter10 += 1
                    if  counter10 >= buffer:
                        twist.linear.x = 0.0
                        self.publisher_.publish(twist)
                        delay(0.01)
                        twist.linear.x = -0.0001
                        twist.angular.z = rad_per_sec
                        if twist.angular.z < 0.5:
                            twist.angular.z *= turning_rate
                            if twist.linear.x > -0.02:
                                twist.linear.x *= linear_increase_rate
                                counter10 = 0
                        else:
                            pass
                    else:
                        continue

                elif ir1 == 0 and ir2 == 1:
                    counter11 = counter10 = 0
                    counter01 += 1
                    if counter01 >= buffer:
                        twist.linear.x = 0.0
                        self.publisher_.publish(twist)
                        delay(0.01)
                        twist.linear.x = -0.0001
                        twist.angular.z = -rad_per_sec
                        if twist.angular.z > -0.5:
                            twist.angular.z *= turning_rate
                            if twist.linear.x > -0.02:
                                twist.linear.x *= linear_increase_rate
                                counter01 = 0
                        else:
                            pass
                    else:
                        continue

                else:
                    counter11 = counter10 = counter01 = 0
                    # decelerates the turn
                    if twist.angular.z < 0.0:
                        if twist.angular.z <= -0.0001:
                            twist.angular.z = 0.0
                        print("i am spinning")
                        twist.angular.z *= reduce_rate
                    elif twist.angular.z > 0.0:
                        if twist.angular.z >= 0.0001:
                            twist.angular.z = 0.0
                        print("i am spinning")
                        twist.angular.z *= reduce_rate
                    else:
                        twist.angular.z = 0.0
                    
                    # decelerates the speed
                    if twist.linear.x < cent_per_sec:
                        twist.linear.x *= linear_decrease_rate
                        
                    elif twist.linear.x > cent_per_sec:
                        twist.linear.x = cent_per_sec
                        print("i am still moving")
                    else:
                        twist.linear.x = cent_per_sec
            
            self.stop_moving()

            if self.activate:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher_.publish(twist)


        except Exception as e:
            print(e)

        # Ctrl-c detected
        finally:
        	# stop moving
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

    # Function 2: stop_moving
    def stop_moving(self):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    # Function 3: Ball Dispenser
    def ball_dispenser(self):
        s1.start(11.5)
        s2.start(2.5)
        s1.ChangeDutyCycle(convert_ang_dc(180)) # default 180 #change to 60(max) for dispensing
        s2.ChangeDutyCycle(convert_ang_dc(0))  #default 0 # change to 120(max) for dispensing
        delay(1)
        angle_reset_s1 = 180
        angle_reset_s2 = 0
        while angle_reset_s1 > 80:
            angle_reset_s1 -= 1
            angle_reset_s2 += 1
            s1.ChangeDutyCycle(convert_ang_dc(angle_reset_s1))
            s2.ChangeDutyCycle(convert_ang_dc(angle_reset_s2))
            print(f's1: {angle_reset_s1} | s2: {angle_reset_s2}')
            delay(0.015)
        
        angle_throw_s1 = 80
        angle_throw_s2 = 100
        delay(3)
        while angle_throw_s1 < 180:
            angle_throw_s1 += 1
            angle_throw_s2 -= 1
            s1.ChangeDutyCycle(convert_ang_dc(angle_throw_s1))
            s2.ChangeDutyCycle(convert_ang_dc(angle_throw_s2))
            print(f's1: {angle_throw_s1} | s2: {angle_throw_s2}')
            delay(0.05)
        s1.stop()
        s2.stop()

    # Function 4: HTTP request to ESP32 --> needs to be tested as I didn't test it as a function
    def send_request(self):
        header = {'Content-Type': 'application/json'}
        endpoint = "http://" + self.ip_address + "/openDoor"
        data = {"action": "openDoor", "parameters": {"robotId": self.ID}}
        response = requests.post(url=endpoint, json=data, headers=header)
        response_dict = ast.literal_eval(response.text)
        return(response_dict['data']['message'])


    # Function 5: rotate Bot
    def rotatebot(self, rot_angle):
        rclpy.spin_once(self)
        error = 2
        rotatechange = 0.5
        self.get_logger().info('In rotatebot')
        twist = geometry_msgs.msg.Twist()
        curr_yaw = math.degrees(self.yaw)
        self.get_logger().info('Current: %f' % curr_yaw)
        delay(0.2)

        if rot_angle > 180:
            rot_angle -= 360
        elif rot_angle < -180:
            rot_angle += 360

        desired_angle = curr_yaw + rot_angle      
        if desired_angle < -180:
            desired_angle += 360
        elif desired_angle > 180:
            desired_angle -= 360     

        if rot_angle < 0:
            rotatechange *= -1 
        
        self.get_logger().info('Desired Yaw: %f' % desired_angle)
        
        while abs(math.degrees(self.yaw) - desired_angle) > error:
            twist.linear.x = 0.0
            twist.angular.z = rotatechange
            self.publisher_.publish(twist)
            rclpy.spin_once(self)
            self.get_logger().info('Current Yaw: %f' % math.degrees(self.yaw))
        self.stop_moving()
    
    def switchmode(self):
        msg = String()
        #maybe like make sure both programs communicating then publish maze
        msg.data = 'explore'
        self.modepublisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

        

def main(args=None):
    twist = geometry_msgs.msg.Twist()
    try:

        rclpy.init(args=args)
        TurtleBot = Func(esp32_ip, turtleBot_ID)
        TurtleBot.stop_moving()
        delay(1)
        TurtleBot.line_follower(1, 6, 0.9999, cent_per_sec=-0.08)

        while True:
            response = TurtleBot.send_request()
            print(f'response are {response}')
            if response == 'door1':
                TurtleBot.rotatebot(80)
                TurtleBot.line_follower(1, 3.5)
                TurtleBot.line_follower(1, 3.8, 0.99999999, turning_rate=1.9, cent_per_sec = -0.06, black_line_forward=-0.015)
                TurtleBot.ball_dispenser()
                TurtleBot.rotatebot(180)
                TurtleBot.line_follower(2, 3.1)
                TurtleBot.rotatebot(-89)
                break
            elif response == 'door2':
                TurtleBot.rotatebot(-80)
                TurtleBot.line_follower(1, 3.5)
                TurtleBot.line_follower(1, 3.8, 0.99999999, turning_rate=1.9, cent_per_sec = -0.06, black_line_forward=-0.015)
                TurtleBot.ball_dispenser()
                TurtleBot.rotatebot(180)
                TurtleBot.line_follower(2, 3.1)
                TurtleBot.rotatebot(89)
                break
        
        TurtleBot.switchmode()
        
        

    finally:  
        GPIO.cleanup()
    

if __name__ == '__main__':
    main()



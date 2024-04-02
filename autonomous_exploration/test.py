import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid , Odometry
from tf2_msgs.msg import TFMessage
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import heapq , math , random , yaml
import scipy.interpolate as si
import sys , threading , time
from .Frontier import *
from .Path import *
from .Move import *

goal_x = -0.1
goal_y = -1.2 #in m use publish point to get coordinates
goal_error = 0.15
front_angle = 20
front_angles = range(-front_angle,front_angle+1,1)
stop_distance = 0.3
sign = -1
v_speed = 0.07 * sign
w_speed = 0.15


def euler_from_quaternion(x,y,z,w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z


class navigationControl(Node):
    def __init__(self):
        super().__init__('Exploration')
        self.occ_subscription = self.create_subscription(OccupancyGrid,'map',self.map_callback,10)
        self.odom_subscription = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.scan_subscription = self.create_subscription(LaserScan,'scan',self.scan_callback,qos_profile_sensor_data)
        self.tf_subscription = self.create_subscription(TFMessage, 'tf', self.tf_callback, qos_profile_sensor_data)
        self.twist_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        threading.Thread(target=self.exp).start() #It runs the discovery function as a thread.
    
    def scan_callback(self,msg):
        self.scan_data = msg
        self.laser_range = np.array(msg.ranges)
        self.laser_range[self.laser_range==0] = np.nan
        np.savetxt('scan.txt', self.laser_range)

    def map_callback(self,msg):
        self.map_data = msg
        self.resolution = self.map_data.info.resolution # meter per cell 
        self.originX = self.map_data.info.origin.position.x
        self.originY = self.map_data.info.origin.position.y
        self.width = self.map_data.info.width
        self.height = self.map_data.info.height
        self.data = self.map_data.data

    def odom_callback(self,msg):
        self.odom_data = msg
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,msg.pose.pose.orientation.w) 
        # self.yaw PI to -PI , 0 is in row direction then anticlockwise increase to PI then goes to -PI 
    
    def tf_callback(self, msg):
        self.transform_data = msg
        self.odom_frame_x = msg.transforms[0].transform.translation.x
        self.odom_frame_y = msg.transforms[0].transform.translation.y
    
    def exp(self):
        twist = Twist()
        # GET COSTMAP
        # expand boundaries based on map 
        while True:
            if not hasattr(self,'map_data') or not hasattr(self,'odom_data') or not hasattr(self,'scan_data'):
                print("has data:", hasattr(self,'map_data'), hasattr(self,'odom_data'), hasattr(self,'scan_data') )
                time.sleep(0.1)
                continue
            self.x = self.odom_frame_x + self.odom_x #might have problems with the delay but fk it lah
            self.y = self.odom_frame_y + self.odom_y
            cmap= costmap(self.data, self.width, self.height)
            np.savetxt('cmap.csv', cmap, delimiter=',')
            # occ_grid = OccupancyGrid might publish cmap in the future 
            # self.cmap_publisher.publish()

            frontier_map = find_frontier_cells(cmap)
            np.savetxt('fmap.csv', frontier_map, delimiter=',')
            
             # group cells are close together 
            frontiers = assign_groups(frontier_map)
            # print(frontiers)
            frontiers = [grp for grp in frontiers.values() if len(grp) > 2] #make list of frontiers with > 2 frontier cells 
            if len(frontiers) == 0:
                print("Discovery Complete")
                return

            goal_x_cell = int((goal_x - self.originX)/self.resolution)
            goal_y_cell = len(cmap) - int((goal_y - self.originY)/self.resolution)
            chosen_x, chosen_y = findClosesttoGoal(frontiers, goal_x_cell, goal_y_cell) 
            print(chosen_x, chosen_y)
            chosen_x = chosen_x* self.resolution + self.originX
            chosen_y = (len(cmap)  - chosen_y) * self.resolution + self.originY

            while target_reached(self.x, self.y, (goal_x, goal_y), goal_error) == False:
                v, w = vroom(chosen_x, chosen_y, self.x, self.y, self.yaw, v_speed, w_speed)
                if self.laser_range.size != 0:
                    lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
                    self.get_logger().info('Distances: %s' % str(lri))
                    lri = np.array(lri)
                    print(lri)

                    # check distances in front of TurtleBot and find values less
                    # than stop_distance
                    if(len(lri[0])>0):
                        self.stopbot()
                        left_count = np.sum(lri<front_angle)
                        right_count = np.sum(lri>front_angle)
                        if left_count > right_count:
                            w = w_speed
                        else:
                            w = w_speed *-1
                        v = 0.0
                    
                twist.linear.x = v
                twist.angular.z = w
                self.get_logger().info('V: %s' % str(v))
                self.get_logger().info('W: %s' % str(w))
                self.twist_publisher.publish(twist)
                time.sleep(0.1)
                self.x = self.odom_frame_x + self.odom_x #might have problems with the delay but fk it lah
                self.y = self.odom_frame_y + self.odom_y

            #stop bot 
            v, w = 0.0, 0.0
            self.twist_publisher.publish(v,w)
            

    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        #time.sleep(1)
        self.twist_publisher.publish(twist)

    def test(self):
        twist = Twist()
        while True:
            if not hasattr(self,'map_data') or not hasattr(self,'odom_data') or not hasattr(self,'scan_data') or not hasattr(self, 'transform_data'):
                print("has data:", hasattr(self,'map_data'), hasattr(self,'odom_data'), hasattr(self,'scan_data'), hasattr(self, 'transform_data') )
                time.sleep(0.1)
                continue
            # self.x = self.odom_frame_x + self.odom_x #might have problems with the delay but fk it lah
            # self.y = self.odom_frame_y + self.odom_y
            # cmap= costmap(self.data, self.width, self.height)
            # np.savetxt('cmap.csv', cmap, delimiter=',')
            # index_x = int((self.x - self.originX)/self.resolution)
            # index_y = len(cmap) - int((self.y - self.originY)/self.resolution)
            # print(index_x, index_y)
            # twist.angular.z = math.pi/4
            # self.twist_publisher.publish(twist)
            time.sleep(10)

        # DETECT FRONTIERS
        

        # do a dfs to find the frontiers 
       
        # generate a list of average coordiantes (return list)

        # CHOOSE FRONTIERS
        # Based on coordinate of door calculate distance between each frontier and the door 
        # Choose frontier closest to the door (return coordinate)

        # PATH PLANNING using astar - i trust LOL
        # ASTAR
        # bfs - search surrounding nodes and calc distance to goal - this is the weight
        # arrange the nodes by weight in a queue 
        # go to node with lowest weight and bfs again 
        # recursion till reach the frontier cell 
        # store all the coordiantes it needs to visit as a list for the robot to follow 

        # OPTIONAL: BSPLINE PLANNING 

        # Get robot to move based on list of coordinates 
        # if coordinate is +1 +1 then diagonal right, rotate to angle and move 
        # use turks code move and calc angle every time change cell 

        # Reach frontier and detect frontiers again
        # If there are no more frontiers then stop exploration 
        
        # 2 modes: Goal mode and rando exploration
        # efeicient rando exploration - try to pass by the frontiers? 
        # But if can see the goal just straight line path towards it stop finding frontiers 
        
        


def main(args=None):
    rclpy.init(args=args)
    navigation_control = navigationControl()
    rclpy.spin(navigation_control)
    navigation_control.destroy_node()
    rclpy.shutdown()
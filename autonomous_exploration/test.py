import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid , Odometry
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import heapq , math , random , yaml
import scipy.interpolate as si
import sys , threading , time
from .Frontier import *
from .Path import *

goal = (5, 3) #in m
goal_error = 0.15


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
        self.twist_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        threading.Thread(target=self.exp).start() #It runs the discovery function as a thread.
    
    def scan_callback(self,msg):
        self.scan_data = msg
        self.scan = msg.ranges
        laserranges = np.array(msg.ranges)
        np.savetxt('scan.txt', laserranges)

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
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,msg.pose.pose.orientation.w) 
        # self.yaw PI to -PI , 0 is in row direction then anticlockwise increase to PI then goes to -PI 
    
    def exp(self):
        twist = Twist()
        # GET COSTMAP
        # expand boundaries based on map 
        while True:
            if not hasattr(self,'map_data') or not hasattr(self,'odom_data') or not hasattr(self,'scan_data'):
                print("has data:", hasattr(self,'map_data'), hasattr(self,'odom_data'), hasattr(self,'scan_data') )
                time.sleep(0.1)
                continue
            cmap= costmap(self.data, self.width, self.height, self.resolution)
            np.savetxt('cmap.csv', cmap, delimiter=',')
            # occ_grid = OccupancyGrid might publish cmap in the future 
            # self.cmap_publisher.publish()

            frontier_map = find_frontier_cells(cmap)
            np.savetxt('fmap.csv', frontier_map, delimiter=',')
            
             # group cells are close together 
            frontiers = assign_groups(frontier_map)
            print(frontiers)
            frontiers = [grp for grp in frontiers.values() if len(grp) > 2] #make list of frontiers with > 2 frontier cells 
            if len(frontiers) == 0:
                print("Discovery Complete")
                return
            target = tuple(m / 0.0521 for m in goal)
            chosen_frontier = findClosesttoGoal(frontiers, target)
            print(chosen_frontier)
            robot_x = int((self.x - self.originX)/self.resolution)
            robot_y = int((self.y - self.originY)/self.resolution)
            path = astar(cmap,(robot_x, robot_y),chosen_frontier) #list of coordinates for robot to follow

            reached = False
            
            while reached == False:
                # reached target (with a bit of error)
                robot_x = int((self.x - self.originX)/self.resolution)
                robot_y = int((self.y - self.originY)/self.resolution)
                target_error = goal_error/self.resolution
                if(abs(robot_x - path[-1][0]) < target_error and abs(robot_y - path[-1][1]) < target_error):
                    reached = True
                




            
            # take the average coordinate 

            

            time.sleep(10)
    def test(self):
        while True:
            if not hasattr(self,'map_data') or not hasattr(self,'odom_data') or not hasattr(self,'scan_data'):
                print("has data:", hasattr(self,'map_data'), hasattr(self,'odom_data'), hasattr(self,'scan_data') )
                time.sleep(0.1)
                continue
            print("self.x", self.x, "self.y", self.y)
            print("self.originX", self.originX, "self.originY", self.originY)
            robot_x = int((self.x - self.originX)/self.resolution)
            robot_y = int((self.y - self.originY)/self.resolution)
            print("robot_x", robot_x)
            print("robot_y", robot_y)
            time.sleep(3)

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
        


def main(args=None):
    rclpy.init(args=args)
    navigation_control = navigationControl()
    rclpy.spin(navigation_control)
    navigation_control.destroy_node()
    rclpy.shutdown()

target = tuple(m / 0.0521 for m in goal)
path = astar(cmap,(robot_x, robot_y),chosen_frontier) #list of coordinates for robot to follow
target_error = goal_error/self.resolution

reached = False
index = 0
# print(self.x, self.y, goal, goal_error)
print("Path:", path)
print("originX:", self.originX, "originY:", self.originY)
print("self.x:", self.x, "originy:", self.y)
print("LEN:", len(path))
while index < len(path):
    # reached target (with a bit of error)
    self.x = self.odom_frame_x + self.odom_x #might have problems with the delay but fk it lah
    self.y = self.odom_frame_y + self.odom_y
    robot_x = int((self.x - self.originX)/self.resolution)
    robot_y = len(cmap) - int((self.y - self.originY)/self.resolution)
    v, w, index = follow_path(robot_x, robot_y, self.yaw, path, index, target_error)

    twist.linear.x = v
    twist.angular.z = w
    self.twist_publisher.publish(twist)
    print("hehe")
    time.sleep(0.1)
twist.linear.x = 0.0
twist.angular.z = 0.0
self.twist_publisher.publish(twist)
print("Reached end of path")

# take the average coordinate 



def pure_pursuit(current_x, current_y, current_heading, path, index):
    global lookahead_distance
    closest_point = None
    v = v_speed
    for i in range(index,len(path)):
        x = path[i][0]
        y = path[i][1]
        distance = math.hypot(current_x - x, current_y - y)
        if lookahead_distance < distance:
            closest_point = (x, y)
            index = i
            break
    if closest_point is not None:
        target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
        desired_steering_angle = target_heading - current_heading
    else:
        target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
        desired_steering_angle = target_heading - current_heading
        index = len(path)-1
    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi
    if desired_steering_angle > math.pi/6 or desired_steering_angle < -math.pi/6:
        sign = 1 if desired_steering_angle > 0 else -1
        desired_steering_angle = sign * math.pi/4
        v = 0.0
    return v,desired_steering_angle,index

#when generating path need to take into account radius of robot?

def follow_path(curr_x, curr_y, curr_heading, path, index, target_error):
    closest_point = path[index]
    
    if target_reached(curr_x, curr_y, closest_point, target_error):
        v = 0.0
        w = 0.0
        index += 1
        print("index:", index)
        return v,w, index
    
    target_heading = math.atan2(closest_point[1] - curr_y, closest_point[0] - curr_x)
    desired_steering_angle = target_heading - curr_heading
   

    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi
    
    w = w_speed
    if abs(desired_steering_angle) < math.pi/6:
        w = 0
    elif desired_steering_angle < 0:
        w *= -1

    return v_speed, w, index
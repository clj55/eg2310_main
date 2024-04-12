import math
import numpy as np

target_error = 0.1
factor = 0.3

TURTLEBOT_RADIUS = 0.25


# tell robot to keep moving to next cell
# reach next cell then move on to next cell
# v and w can js keep changing ig
    
#generate v and w based on next cell 
# target heading relative to horizontal 
# target heading = atan(nextcell[1]- currcell[1], nextcell[0] - currcell[0])
# target heading - current heading = desired steering angle in radian 
# then do the turn shorter angle stuff 
# turn till abs(desired_steering angle) < math.pi/6 then dont turn 
def target_reached(curr_x, curr_y, target, target_error):
    return (abs(curr_x - target[0]) < target_error and abs(curr_y - target[1]) < target_error)

def avg(list):
    return sum(list)/len(list)

def scale_angle(laser_angle, laserrange):
    return laser_angle * 360 / laserrange

def find_opening(laserrange):
    RightFound = False
    LeftFound = False
    np.savetxt('laserrange.txt', laserrange)
    for i in range(5, int(len(laserrange)/2) - 5):

        ##### SPKIKE EXTRACTION ######
        if not RightFound:
            #check right side
            right_inner_dist = avg(laserrange[i - 5: i])
            right_outer_dist = avg(laserrange[i:i+5])
            # print('Right wall dist: ', right_wall_dist)
            print((right_outer_dist - right_inner_dist) )

            if (right_outer_dist - right_inner_dist) > 0.2: # if wall is infront of robot
                # opening right outer
                right_angle = i
                RightFound = True
                Outer = True
                print("Right outer opening Found")
            elif (right_inner_dist - right_outer_dist) > 0.2:  #if the wall is like beside robot
                right_angle = i
                RightFound = True
                Outer = False
                print("Right inner opening Found ")
        
        if not LeftFound: 
            #check left side 
            left_inner_dist = avg(laserrange[-i-5:-i])
            left_outer_dist = avg(laserrange[-i-10: -i-5]) 
            # print('Left wall dist: ', left_wall_dist)
            if not LeftFound and (left_outer_dist- left_inner_dist) > 0.2:
                left_angle = i 
                LeftFound = True
                Outer = True
                print("LeftFound")
            elif (left_inner_dist - left_outer_dist) > 0.2:  #if the wall is like beside robot
                left_angle = i
                LeftFound = True
                Outer = False
                print("Right inner opening Found ")
        


    if LeftFound and RightFound:
        if left_angle > right_angle: # GO RIGHT
            laser_angle = right_angle
            if Outer:
                wall_dist = right_outer_dist
            else:
                wall_dist = right_inner_dist
        else: #GO RIGHT
            laser_angle = len(laserrange) - left_angle
            if Outer:
                wall_dist = left_outer_dist
            else:
                wall_dist = left_inner_dist
    
    elif LeftFound:
        laser_angle = len(laserrange) - left_angle
        if Outer:
            wall_dist = left_outer_dist
        else:
            wall_dist = left_inner_dist
    
    elif RightFound:
        laser_angle = right_angle
        if Outer:
            wall_dist = right_outer_dist
        else:
            wall_dist = right_inner_dist

    else:
        print("knn find opening also cannot")
        print("opening straight ahead?")
        laser_angle = 0
        wall_dist = laserrange[0]
    
    actl_angle = scale_angle(laser_angle, len(laserrange))
    add_angle = math.atan(TURTLEBOT_RADIUS/wall_dist)
    

    if actl_angle > 180:
        rotangle = actl_angle + add_angle
    else:
        rotangle = actl_angle - add_angle
    
    # if rotangle 

    return rotangle

    

    
     
    


def pick_direction(target_x, target_y, curr_x, curr_y, curr_heading):
    target_heading =  math.atan2(target_y - curr_y, target_x - curr_x)
    desired_steering_angle = target_heading - curr_heading
    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi
    return desired_steering_angle
    
    v = v_speed
    w = desired_steering_angle * factor
    if abs(desired_steering_angle) > math.pi/2:
        v = 0.0
    elif desired_steering_angle > math.pi/4:
        w = w_speed
    elif desired_steering_angle < -math.pi/4:
        w = w_speed * -1
    elif desired_steering_angle < 0:
        w *= -1 
    return v, w

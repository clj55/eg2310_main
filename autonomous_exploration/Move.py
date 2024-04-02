import math

target_error = 0.1




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


def vroom(target_x, target_y, curr_x, curr_y, curr_heading,v_speed, w_speed):
    target_heading =  math.atan2(target_y - curr_y, target_x - curr_x)
    desired_steering_angle = target_heading - curr_heading
    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi
    
    v = v_speed
    w = desired_steering_angle
    if abs(desired_steering_angle) > math.pi/2:
        v = 0.0
    elif desired_steering_angle > math.pi/4:
        w = w_speed
    elif desired_steering_angle < -math.pi/4:
        w = w_speed * -1
    elif desired_steering_angle < 0:
        w *= -1 
    return v, w

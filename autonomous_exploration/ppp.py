obstacleMid = (leftTurnMin + rightTurnMin) / 2
cut = laser_range[obstacleMid:] #cut 
remaining = laser_range[:obstacleMid] #remaining
laser_range = cut + remaining # shift array so that middle of obstacle starts at index 0
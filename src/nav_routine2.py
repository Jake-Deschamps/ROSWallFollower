#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def nav_logic(my_scan):
    # The goal of this version will be to identify the closest point
    # on the right hand side of the robot
    # If this distance is within 0.2-0.3, then we will act to
    # keep it in position 180 (+/- some amnt) (if it's below that we'll turn away, if above, we'll turn toward)
    # If this distance is >0.3, we'll turn to put it
    # in a position above 180 (240 +/- means approaching at 30 deg)
    # If this distance is <0.2, we'll turn to put it in
    # a position below 180 (120 +/- means moving away at 30 deg)
    
    # Lastly, we'll need a behavior for latching to a new wall,
    # If the minimum is very far away from 180, we'll
    # halt and turn

    # Note that we *should* be able to determine the margin we need
    # to search for by our turn angle and error margins
    # 0(back) - (turn_angle + margin)
    # to end
    # beginning to
    # 540 + (turn_angle + margin)

    # Settings
    turn_angle = 30 * 2 # Approach/retreat angle in deg (*2 since) the scan is 720 over 360 deg
    margin = 5 * 2 # Heading can be off by margin in either direciton
    right_target = 180 # the index of the right-hand side of the robot
    turn_speed = 0.5 # base speed for rotation 0.1 works
    march_speed = 0.1 # base speed for forward progression 0.02 works (0.1, 0.03)
    # (0.5, 0.05)
    # (0.8, 0.05)
    # (0.8, 0.1) seems alright, turns a little rapidly
    # (0.9, 0.15) makes him lose the wall/bonk occasionally

    # Get critical info    
    wall_dist = min(my_scan.ranges) # the distance to the wall
    wall_index = my_scan.ranges.index(wall_dist) # the location of the nearest wall
    wall_str = f"Wall at index {wall_index}, Distance {wall_dist:.2f} m"

    # Use the wall index to determine our wall target
    if wall_dist < 0.2:                         # if we are too close to the wall
        wall_target = right_target - turn_angle # -> set target to turn away
        target_str = f"Too close, target: {wall_target}"
    elif wall_dist > 0.3:                       # if we are too far from the wall
        wall_target = right_target + turn_angle # -> set target to turn to
        target_str = f"Too far, target: {wall_target}"
    else:                                       # if we are in the sweet spot
        wall_target = right_target              # -> set target to right-hand side
        target_str = f"Sweet spot, target: {wall_target}"

    # Adjust turn speed to put wall into target
    if wall_index < wall_target - margin:   # if the wall is CW of the target
        nav_logic.set_angular = -turn_speed # -> turn CW
        turn_str = "Turning CW"
    elif wall_index > wall_target + margin: # if the wall is CCW of the target
        nav_logic.set_angular = +turn_speed # -> turn CCW
        turn_str = "Turning CCW"
    else:                                   # if the wall is in our margin
        nav_logic.set_angular = 0           # -> do not turn
        turn_str = "Turning 0"

    # emergency setup: too close and off by too much
    if (wall_dist < 0.20) and (abs(wall_target - wall_index) > 90):
        # originally wall_dist < 0.15
        nav_logic.set_linear = 0
        print("HALTING" + " " + wall_str + " " + target_str + " " + turn_str)
    else:
        nav_logic.set_linear = march_speed
        print(wall_str + " " + target_str + " " + turn_str)



rospy.init_node('wall_follow_nav')
sub = rospy.Subscriber('/scan', LaserScan, nav_logic)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

rate = rospy.Rate(2)

setattr(nav_logic, 'set_linear', 0.01)
setattr(nav_logic, 'set_angular', 0)

set_twist = Twist()
set_twist.linear.x = 0
set_twist.angular.z = 0

print("setup complete")

while not rospy.is_shutdown():
    set_twist.linear.x = nav_logic.set_linear
    set_twist.angular.z = nav_logic.set_angular
    pub.publish(set_twist)
    rate.sleep()
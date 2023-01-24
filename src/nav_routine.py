#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def nav_logic(my_scan):
    f_scan = my_scan.ranges[360]
    l_scan = my_scan.ranges[540]
    r_scan = my_scan.ranges[180]
    read_str = f'L:{l_scan:.2f} F:{f_scan:.2f} R:{r_scan:.2f}'
    #nav_logic.set_linear = 0.1
    #nav_logic.set_angular = 0
    turn_speed = 0.05
    march_speed = 0.02

    # if approaching wall in front
    if f_scan < 0.2:
        nav_logic.set_angular = 3*turn_speed
        nav_logic.set_linear = 0
        # turn behavior to catch new wall
        behav_str = "Approaching forward wall"
    else:
        nav_logic.set_linear = march_speed
        # if right side is bigger than 0.3
        if r_scan > 0.3:
            nav_logic.set_angular = -turn_speed/2
            # get closer
            behav_str = "Approaching current wall"
        # if right side is smaller than 0.2
        elif r_scan < 0.2:
            nav_logic.set_angular = turn_speed
            # get further
            behav_str = "Distancing from current wall"
        # if between
        else:
            nav_logic.set_angular = 0
            # remove turn
            behav_str = "Distance good, progressing"
    set_str = f"set_linear: {nav_logic.set_linear}   set_angular: {nav_logic.set_angular}"
    print(read_str +" " + behav_str + " " + set_str)

rospy.init_node('wall_follow_nav')
sub = rospy.Subscriber('/scan', LaserScan, nav_logic)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

rate = rospy.Rate(2)

setattr(nav_logic, 'set_linear', 0.1)
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
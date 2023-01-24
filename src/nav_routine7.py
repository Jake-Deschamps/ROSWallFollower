#! /usr/bin/env python

import numpy as np
import warnings
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from wall_follow.srv import FindWall, FindWallRequest
import actionlib
from wall_follow.msg import OdomRecordAction, OdomRecordGoal, OdomRecordResult, OdomRecordFeedback

# Version 7: Cleaning up control loop

# Version 6: Now with odom_record action server!

# Version 5: Switch to simple control algorithms. Maybe remove complex algorithms from the scan callback

# Version 4: Now with find_wall service!


def callback(my_scan):
    callback.last_scan = my_scan.ranges


def feedback_callback(feedback):
    rospy.loginfo(f"Travelled {feedback.current_total:.3f} meters.")


def nav_logic(my_scan):
    # The goal of version 3 is to implement some mechanism to smooth out the turn onto the
    # new wall. Something like looking at the 30 degree window [330-390] at the front of the robot
    # and seeing if a wall is within some new range. If this is the case, the nearest-wall
    # info will be overwritten with this value

    # The goal of version2 was to identify the closest point
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
    k_p_x = 5  # Halved to resolve units10       # Proportional gain for distance error -> set angle
    k_p_t = 1  # Doubled to resolve units.5    # Proportional gain for theta error -> set angular
    v = 0.15        # Forward velocity for general use
    rhs = 180  # the index of the right-hand side of the robot
    e_dist = 0.1  # the distance from the wall where we will halt and emergency behavior kicks in
    front_window = 5*2  # 15*2 # distance in degrees away from front to define front wall window
    front_wall_distance = 0.55  # distance to use front wall behavior
    f_wall_w = 0.49
    f_wall_v = 0.1
    # note that these can be solved for:
    # assuming we are in the sweetspot and heading straight, we will want to complete
    # a 90-deg turn in the time it takes for us to traverse from front_wall_distance to 0.25
    # t = (pi/2)/f_wall_turn_speed,   t = (front_wall_distance - 0.25)/f_wall_march_speed
    # d = pi*v/(2w) + 0.25
    # for v = 0.07 and w = 0.45, d = 0.494 (nice!)
    # for v = 0.07 and w = 0.49, d = 0.474
    # for v = 0.10 and w = 0.45, d = 0.599
    # for v =  0.19 and w = 0.49, d = 0.859 (which is almost half of the wall)
    # At top speed, we traverse a wall in ~10 seconds
    # At top angular speed, we complete a rotation in 12.8 seconds

    # Get critical info
    abs_min_dist = min(my_scan)
    wall_dist = min(my_scan[0:360])  # the distance to the wall
    # the location of the nearest wall
    wall_index = my_scan.index(wall_dist)
    wall_str = f"Wall at index {wall_index}, Distance {wall_dist:.2f} m"
    # See if front wall should overwrite this
    f_min = 360 - front_window
    f_max = 360 + front_window
    front_wall = min(my_scan[f_min:f_max])

    if front_wall < front_wall_distance:
        print("New wall!")
        nav_logic.set_linear = f_wall_v  # maybe modulate v wrt to x_error
        nav_logic.set_angular = f_wall_w
        return

    # If we are approaching the front wall
    # AND (the other wall distance isn't close enough for halting behavior)
    #       OR (the front wall distance IS close enough for halting behavior)
    # (front_wall < front_wall_distance) and (not wall_dist < e_dist):

    # if False:
    #     wall_dist = front_wall
    #     wall_index = my_scan.ranges[f_min:f_max].index(wall_dist) + f_min
    #     wall_str = f"NEW wall at index {wall_index}, Distance {wall_dist:.2f} m"
    #     turn_speed = f_wall_turn_speed
    #     march_speed = f_wall_march_speed

    x_error = 0.25 - wall_dist
    theta_set = k_p_x * x_error

    theta_error = theta_set - (1/2)*(rhs - wall_index)*np.pi/180
    w = k_p_t * theta_error

    # # emergency setup: too close and off by too much
    # if False:  # (wall_dist < 0.20) and (abs(wall_target - wall_index) > 90):
    #     # originally wall_dist < 0.15
    #     nav_logic.set_linear = 0
    #     print("HALTING" + " " + wall_str + " " + target_str + " " + turn_str)
    # else:
    #     nav_logic.set_linear = v
    #     nav_logic.set_angular = w
    #     print(wall_str + " " + target_str + " " + turn_str)

    nav_logic.set_linear = v  # maybe modulate v wrt to x_error
    nav_logic.set_angular = w
    print(wall_str + " " + f"setting angular velocity {w:.3f}")


rospy.init_node('wall_follow_nav')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

rate = rospy.Rate(2)

setattr(nav_logic, 'set_linear', 0.01)
setattr(nav_logic, 'set_angular', 0)

setattr(callback, 'last_scan', [0]*720)

set_twist = Twist()
set_twist.linear.x = 0
set_twist.angular.z = 0

# Call find wall service
rospy.loginfo("Setup complete, finding wall")
rospy.wait_for_service('/find_wall')
find_wall_service = rospy.ServiceProxy('/find_wall', FindWall)
find_wall_object = FindWallRequest()
find_wall_service(find_wall_object)

rospy.loginfo("Find wall complete. Initializing call to odom_record")

# Call odom recorder
action_server_name = '/odom_recorder_as'
client = actionlib.SimpleActionClient(action_server_name, OdomRecordAction)
# waits until the action server is up and running
rospy.loginfo('Waiting for action Server '+action_server_name)
client.wait_for_server()
rospy.loginfo('Action Server Found...'+action_server_name)
goal = OdomRecordGoal()
client.send_goal(goal, feedback_cb=feedback_callback)

sub = rospy.Subscriber('/scan', LaserScan, callback)

while not rospy.is_shutdown():
    if client.get_state() < 2:
        nav_logic(callback.last_scan)
        set_twist.linear.x = nav_logic.set_linear
        set_twist.angular.z = nav_logic.set_angular
        # DO NOT EXCEED linear speed of 0.19 or angular of 0.49
        # The node will be terminated
        if abs(set_twist.linear.x) > 0.19:
            set_twist.linear.x = 0.19*np.sign(set_twist.linear.x)
            rospy.logwarn(
                f"Linear speed {nav_logic.set_linear:.2f} exceeds robot safety max 0.19. Set to {set_twist.linear.x}.")
        if abs(set_twist.angular.z) > 0.49:
            set_twist.angular.z = 0.49*np.sign(set_twist.angular.z)
            rospy.logwarn(
                f"Angular speed {nav_logic.set_angular:.2f} exceeds robot safety max 0.49. Set to {set_twist.angular.z}.")
        print(
            f"Publishing lin speed {set_twist.linear.x}, angular speed {set_twist.angular.z}")
        pub.publish(set_twist)
    else:
        set_twist.linear.x = 0
        set_twist.angular.z = 0
        rospy.loginfo("Finished a lap. Halting.")
        pub.publish(set_twist)
        rospy.spin()
    rate.sleep()

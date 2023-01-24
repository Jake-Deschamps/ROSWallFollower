#! /usr/bin/env python

# Did some math. Removed integral control from turning and set Kp = 2, which seems to be optimal

# This newest version will implement some actual control algorithms to attempt to
# improve turning to the wall
# To test this, some things are required first:
#       1: my calcs say it should take the robot 31 seconds to complete a half turn (max turn) at 0.1 commanded
#          this needs to be validated. aside, we have 30.5 sec to get from max error to window
#           -> 103 sec for a full turn @ -0.1 (on laptop)
#           -> 2min 9s for two turns @ 0.1 (on real robot) (32.25 sec for 180 deg)
#       2: Make sure that the error determined stops being >180. Error of 355 deg should just be -5 deg!
#           -> think I've got it!
#       3: what is the publishing rate of odom?
#           -> 30 Hz
#       4: implement proportional control with our determined max. sim says we can get max time down to 23 sec.
#           -> implemented. testing:
#       5: there was a point where the angle error determining had a domain error when running on real robot
#           -> this was when using acos. hopefully atan2 is more robust.
#       6: redo error handling when commanding too extreme velocities. issue a warning and command the max legal velocity
#           -> done, hopefully it works nicely.
#       7: try the PI control!
#           -> 177deg : 13.5s
#           -> 172.5deg in 15s
#           -> 71.5 deg: 12.5s
#           -> 52 deg: 12s
#           -> 18.5 deg: 9.5s
# This may also be used to
# There is also currently an issue where the robot will see walls between a narrow gap between obstacles
# Minimum distance behavior? Even with a successfully identified obstacle we might get caught when trying to turn.

import timeit

from math import pi, atan2, sqrt
import warnings
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from wall_follow.srv import FindWall, FindWallResponse
from tf.transformations import quaternion_multiply, quaternion_from_euler, quaternion_inverse


class FindWallServerObject:
    def __init__(self):
        # self.latest_scan = [0]*720 # Initialize latest scan
        self.seize_control = False  # This turns on when the service is called
        # self.routine_complete = False
        self.my_twist = Twist()
        self.wall_dist = 0
        self.wall_index = 0
        self.abs_min_dist = 0
        self.abs_min_index = 0
        self.q_set = quaternion_from_euler(0, 0, 0)
        self.recent_scan = [0]*720
        self.has_odom = False
        self.has_laser = False

    def publish_velocities(self):
        if self.seize_control:
            # DO NOT EXCEED linear speed of 0.19 or angular of 0.49
            # You will lose the real robot
            if abs(self.my_twist.linear.x) > 0.19:
                rospy.logwarn(
                    f"Linear speed {self.my_twist.linear.x:.2f} exceeds robot safety max 0.19")
                self.my_twist.linear.x = np.sign(self.my_twist.linear.x)*0.19
                # self.my_twist.angular.z = 0
            if abs(self.my_twist.angular.z) > 0.49:
                rospy.logwarn(
                    f"Angular speed {self.my_twist.angular.z:.2f} exceeds robot safety max 0.49")
                #self.my_twist.linear.x = 0
                self.my_twist.angular.z = np.sign(self.my_twist.angular.z)*0.49
            rospy.logdebug(
                f"Published linear {self.my_twist.linear.x:.3f}, angular {self.my_twist.angular.z:.3f}")
            pub.publish(self.my_twist)

    def identify_walls(self, latest_scan):
        wall_jump = 0.1  # Down from 0.3
        wall_span = 2*40  # The number of readings we must see without a jump

        nranges = len(latest_scan)

        # There is definitely some super clever way to do this with numpy by shifting a copy of the ranges and subtracting the whole lists
        jumps = []
        for i, r in enumerate(latest_scan):
            if abs(r - latest_scan[i-1]) > wall_jump:
                jumps.append(i-1)
                jumps.append(i)

        # print((jumps))

        WallIndices = []
        ObstacleIndices = []
        # This is a hot mess. Also, check for fenceposting

        for i, j in enumerate(jumps):
            if i == 0:  # get the span for the first wall in a weird way
                my_span_len = j + nranges - jumps[i-1]
                my_span = []
                my_span.extend(range(j+1))
                last_span = range(jumps[i-1], nranges)
                if my_span_len > wall_span:
                    last_span_judge = 'W'
                else:
                    last_span_judge = 'O'
            else:
                my_span_len = j - jumps[i-1]  # + 1
                my_span = []
                my_span.extend(range(jumps[i-1], j+1, 1))

            if my_span_len > wall_span:
                WallIndices.extend(my_span)
            else:
                ObstacleIndices.extend(my_span)

        if last_span_judge == 'W':
            WallIndices.extend(last_span)
        else:
            ObstacleIndices.extend(last_span)

        # thetas = np.arange(0, 2*np.pi, 2*np.pi/720)
        # wall_thetas = []
        wall_ranges = []
        for wi in WallIndices:
            # wall_thetas.append(thetas[wi])
            wall_ranges.append(latest_scan[wi])

        self.wall_dist = min(wall_ranges)
        self.wall_index = WallIndices[wall_ranges.index(self.wall_dist)]

    def receive_scan(self, my_scan):
        self.recent_scan = my_scan.ranges

        if not self.has_laser:
            self.identify_walls(my_scan.ranges)

        # Identify which laser ray is the shortest
        # self.wall_dist = min(filtered_scan) # the distance to the wall
        # self.wall_index = filtered_scan.index(self.wall_dist) # the location of the nearest wall

        # Identify which laser ray is the shortest
        self.abs_min_dist = min(my_scan.ranges)  # the distance to the wall
        self.abs_min_index = my_scan.ranges.index(
            self.abs_min_dist)  # the location of the nearest wall
        self.has_laser = True

    def receive_odom(self, new_odom):
        self.current_quat = [new_odom.pose.pose.orientation.x, new_odom.pose.pose.orientation.y,
                             new_odom.pose.pose.orientation.z, new_odom.pose.pose.orientation.w]
        # https://stackoverflow.com/questions/22157435/difference-between-the-two-quaternions
        q_error = quaternion_multiply(
            self.q_set, quaternion_inverse(self.current_quat))
        #    self.current_quat, quaternion_inverse(self.q_set))
        # https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Recovering_the_axis-angle_representation
        q_error_v_norm = sqrt(q_error[0]**2 + q_error[1]**2 + q_error[2]**2)
        error_axis = [q_error[0]/q_error_v_norm,
                      q_error[1]/q_error_v_norm, q_error[2]/q_error_v_norm]
        temp_error_angle = 2*atan2(q_error_v_norm, q_error[3])
        # Could we just use modulo here?
        while temp_error_angle > pi:
            temp_error_angle -= 2*pi
        while temp_error_angle < -pi:
            temp_error_angle += 2*pi
        self.error_angle = temp_error_angle*error_axis[2]
        # if self.turn_error > 180:
        #     self.turn_error = abs(360 - self.turn_error)
        self.has_odom = True

    def my_callback(self, _):
        # Turn flags and begin publishing velocities
        self.seize_control = True
        #self.rountine_complete = False
        margin = 2*pi/180  # acceptable margin of error in degrees

        # Check to make sure we have received a processed scan
        self.has_laser = False
        while not self.has_laser:
            rate.sleep()

        # Maybe add something here to break out if its at a minimal disance
        rospy.loginfo(
            f"Turning to wall at {self.wall_index}, {self.recent_scan[self.wall_index]} m away.")
        turn_angle = (self.wall_index - 360)/2

        q_rot = quaternion_from_euler(0, 0, turn_angle * pi/180)
        q_init = self.current_quat
        self.q_set = quaternion_multiply(q_rot, q_init)

        # self.turn_error = 99999
        # Initiate Turn
        # While ORIENTATION NOT ACCEPTABLE
        self.has_odom = False
        while not self.has_odom:
            rate.sleep()

        #print(f"{self.error_angle}")
        dt = 1/2
        k_p = 2  # 0.15
        k_i = 0  # 0.008 #0.004 has worked before
        # error_integral = 0
        start = timeit.default_timer()
        while abs(self.error_angle) > margin:
            # error_integral += self.error_angle*dt
            self.my_twist.angular.z = k_p*self.error_angle  # + k_i*error_integral
            rospy.logdebug(
                f"Error: {self.error_angle*180/pi:.2f} Setting Velocity: {self.my_twist.angular.z}")
            self.publish_velocities()
            rate.sleep()

        self.my_twist.angular.z = 0
        self.publish_velocities()

        stop = timeit.default_timer()
        rospy.loginfo(f'Turn to time: {stop - start}')

        # Phase 2: approach until distance is good
        dt = 1/2
        k_p = 0.19  # should be around the max k_p for purely P to not exceed max commandable v
        k_i = 0.008  # 0.004 gave 18s approach
        #self.my_twist.linear.x = 0.04
        # self.abs_min_dist > 0.30:
        error_integral = 0
        # min(self.recent_scan[350:370]) > 0.30:
        start = timeit.default_timer()
        while abs(min(self.recent_scan[350:370]) - 0.30) > 0.02:
            # Not sure I understand the sign here
            error = min(self.recent_scan[350:370]) - 0.30
            error_integral += error*dt
            self.my_twist.linear.x = k_p*error + k_i*error_integral
            rospy.logdebug(
                f"Error: {error:.3f} Setting Velocity: {self.my_twist.linear.x}")
            self.publish_velocities()
            rate.sleep()

        self.my_twist.linear.x = 0
        self.publish_velocities()

        stop = timeit.default_timer()
        rospy.loginfo(f'Approach time: {stop - start}')

        # Initiate the final turn to put wall at 270
        while abs(self.abs_min_index - 270) > 5:
            if 270 - self.abs_min_index > 0:
                self.my_twist.angular.z = -0.2
            else:
                self.my_twist.angular.z = 0.2
            self.publish_velocities()

        self.my_twist.linear.x = 0
        self.my_twist.angular.z = 0
        self.publish_velocities()
        rospy.sleep(1)
        # This stuff may not be needed
        # while not self.routine_complete:
        #    rate.sleep()
        # Return True
        self.seize_control = False
        my_response = FindWallResponse()
        my_response.wallfound = True
        rospy.loginfo("End")
        return my_response


if __name__ == '__main__':
    my_ServerObject = FindWallServerObject()

    rospy.init_node('find_wall_server')
    sub = rospy.Subscriber('/scan', LaserScan, my_ServerObject.receive_scan)
    sub_odom = rospy.Subscriber(
        '/odom', Odometry, my_ServerObject.receive_odom)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    rate = rospy.Rate(2)

    find_wall_service = rospy.Service(
        '/find_wall', FindWall, my_ServerObject.my_callback)

    rospy.spin()

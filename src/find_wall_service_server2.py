#! /usr/bin/env python

from math import pi, acos
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from wall_follow.srv import FindWall, FindWallResponse
from tf.transformations import quaternion_multiply, quaternion_from_euler

# Current suspected issues:
# Minimum distance behavior
# identifying series of obstacles as walls?


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
                self.my_twist.linear.x = 0
                self.my_twist.angular.z = 0
                pub.publish(self.my_twist)
                raise Exception(
                    f"Linear speed {self.my_twist.linear.x:.2f} exceeds robot safety max 0.19")
            elif abs(self.my_twist.angular.z) > 0.49:
                self.my_twist.linear.x = 0
                self.my_twist.angular.z = 0
                pub.publish(self.my_twist)
                raise Exception(
                    f"Angular speed {self.my_twist.angular.z:.2f} exceeds robot safety max 0.49")
            pub.publish(self.my_twist)

    def identify_walls(self, latest_scan):
        wall_jump = 0.1 # Down from 0.3
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
        self.turn_error = (180/pi) * 2 * acos(self.q_set[0]*self.current_quat[0] +
                                              self.q_set[1]*self.current_quat[1] +
                                              self.q_set[2]*self.current_quat[2] +
                                              self.q_set[3]*self.current_quat[3])
        if self.turn_error > 180:
            self.turn_error = abs(360 - self.turn_error)
        self.has_odom = True

    def my_callback(self, _):
        # Turn flags and begin publishing velocities
        self.seize_control = True
        #self.rountine_complete = False
        margin = 5  # acceptable margin of error in degrees

        self.has_odom = False
        self.has_laser = False
        while not(self.has_odom and self.has_laser):
            rate.sleep()

        # Maybe add something here to break out if its at a minimal disance
        print(f"Turning to wall at {self.wall_index}.")
        turn_angle = (self.wall_index - 360)/2
        if self.wall_index < 360:
            self.my_twist.angular.z = -0.1 # -0.2
        elif self.wall_index > 360:
            self.my_twist.angular.z = 0.1 # 0.2

        # print(f"Turning to wall at {self.abs_min_index}.")
        # turn_angle = (self.abs_min_index - 360)/2
        # if self.abs_min_index < 360:
        #     self.my_twist.angular.z = -0.1
        # elif self.abs_min_index > 360:
        #     self.my_twist.angular.z = 0.1

        q_rot = quaternion_from_euler(0, 0, turn_angle * pi/180)
        q_init = self.current_quat
        self.q_set = quaternion_multiply(q_rot, q_init)

        # self.turn_error = 99999
        # Initiate Turn
        # While ORIENTATION NOT ACCEPTABLE

        while abs(self.turn_error) > margin:
            self.publish_velocities()
            print(self.turn_error)
            rate.sleep()

        self.my_twist.angular.z = 0
        self.publish_velocities()

        # Phase 2: approach until distance is good
        self.my_twist.linear.x = 0.04
        # self.abs_min_dist > 0.30:
        while min(self.recent_scan[350:370]) > 0.30:
            self.publish_velocities()
            rate.sleep()

        self.my_twist.linear.x = 0
        self.publish_velocities()

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
        # This stuff may not be needed
        # while not self.routine_complete:
        #    rate.sleep()
        # Return True
        self.seize_control = False
        my_response = FindWallResponse()
        my_response.wallfound = True
        print("End")
        return my_response


my_ServerObject = FindWallServerObject()

rospy.init_node('find_wall_server')
sub = rospy.Subscriber('/scan', LaserScan, my_ServerObject.receive_scan)
sub_odom = rospy.Subscriber('/odom', Odometry, my_ServerObject.receive_odom)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

rate = rospy.Rate(2)

find_wall_service = rospy.Service(
    '/find_wall', FindWall, my_ServerObject.my_callback)

rospy.spin()

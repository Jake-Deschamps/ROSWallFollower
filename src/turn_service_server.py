#! /usr/bin/env python

from math import pi, acos
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from wall_follow.srv import Turn, TurnResponse
from tf.transformations import quaternion_multiply, quaternion_from_euler


class TurnServerObject:
    def __init__(self):
        #self.latest_scan = [0]*720 # Initialize latest scan
        self.seize_control = False # This turns on when the service is called
        # self.routine_complete = False
        self.my_twist = Twist()
        self.my_odom = Odometry()
        self.q_set = quaternion_from_euler(0,0,0)

    def publish_velocities(self):
        # DO NOT EXCEED linear speed of 0.19 or angular of 0.49
        # You will lose the real robot
        if abs(self.my_twist.linear.x) > 0.19:
            self.my_twist.linear.x = 0
            self.my_twist.angular.z = 0
            pub.publish(self.my_twist)
            raise Exception(f"Linear speed {self.my_twist.linear.x:.2f} exceeds robot safety max 0.19")
        elif abs(self.my_twist.angular.z) > 0.49:
            self.my_twist.linear.x = 0
            self.my_twist.angular.z = 0
            pub.publish(self.my_twist)
            raise Exception(f"Angular speed {self.my_twist.angular.z:.2f} exceeds robot safety max 0.49")
        pub.publish(self.my_twist)

    def receive_odom(self, new_odom):
        self.current_quat = [new_odom.pose.pose.orientation.x, new_odom.pose.pose.orientation.y, new_odom.pose.pose.orientation.z, new_odom.pose.pose.orientation.w]
        self.turn_error = (180/pi) * 2 * acos(self.q_set[0]*self.current_quat[0] + \
                                        self.q_set[1]*self.current_quat[1] + \
                                        self.q_set[2]*self.current_quat[2] + \
                                        self.q_set[3]*self.current_quat[3])

    def my_callback(self, request):
        margin = 5 # acceptable margin of error in degrees
        q_rot = quaternion_from_euler(0, 0, request.degrees * pi/180)
        q_init = self.current_quat
        self.q_set = quaternion_multiply(q_rot, q_init)
        
        self.my_twist.angular.z = 0.2
        self.turn_error = 99999
        # Initiate Turn
        # While ORIENTATION NOT ACCEPTABLE
        
        while abs(self.turn_error) > margin:
            self.publish_velocities()
            print(self.turn_error)
            rate.sleep()

        self.my_twist.angular.z = 0
        self.publish_velocities()

        my_response = TurnResponse()
        my_response.success = True
        print("End")
        return my_response


my_ServerObject = TurnServerObject()

rospy.init_node('turn_server')
sub = rospy.Subscriber('/odom', Odometry, my_ServerObject.receive_odom)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

rate = rospy.Rate(2)

find_wall_service = rospy.Service('/turn', Turn, my_ServerObject.my_callback)

rospy.spin()
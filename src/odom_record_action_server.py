#! /usr/bin/env python

# To do:
# Implement into the main program!!
# Test lap recognition with corners/real robot and such
# determine if the result deletion on preempt is needed


from math import sqrt, pi, sin, cos
import rospy
import actionlib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32
from wall_follow.msg import OdomRecordAction, OdomRecordFeedback, OdomRecordResult
from tf.transformations import euler_from_quaternion

class OdomRecorderServerClass(object):
    _feedback = OdomRecordFeedback()
    _result = OdomRecordResult()
    _lastOdom = Odometry()

    def __init__(self):
        self._as = actionlib.SimpleActionServer("odom_recorder_as", OdomRecordAction, self.goal_callback, False)
        self._as.start()
        self._odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.loginfo("Started odom_recorder_as action server and its subscriber to odom")

    def odom_callback(self, odom):
        self._lastOdom = odom

    def goal_callback(self, goal):
        rospy.loginfo("Goal called")
        r = rospy.Rate(1)
        success = True
        distance = 0 # initialized distance travelled
        Lap_State = 1

        # Take initial position values
        x_i = self._lastOdom.pose.pose.position.x
        y_i = self._lastOdom.pose.pose.position.y
        quatlist = self._lastOdom.pose.pose.orientation.x, self._lastOdom.pose.pose.orientation.y, self._lastOdom.pose.pose.orientation.z, self._lastOdom.pose.pose.orientation.w
        [_, _, theta_i] = euler_from_quaternion(quatlist) # only need yaw. note that quatlist is in order xyzw
        
        # Generate constants for the lap equations. All of these are lines/line inequalities
        # in standard from. i.e. Ax + By = C

        # Goal (_g) line eq: theta_g is 45 deg ccw from theta_i (since robot starts at a 45 deg angle)
        # Note that A_g*x + B_g*y >= C_g is true for positions in FRONT of the goal line
        theta_g = theta_i - pi/4
        A_g = -sin(theta_g)
        B_g = cos(theta_g)
        C_g = -x_i*sin(theta_g) + y_i*cos(theta_g)

        # Road (_r) line eqs. Ensures robot is close to its original wall by requiring it to be
        # between two lines that *should* be parallel to the original wall. These lines will
        # be offset from the robots position by a distance d. It may be better to use two
        # values of d for each side.
        # The left edge inequality A_r*x + B_r*y <= C_r_left
        # The right edge inequality A_r*x + B_r*y >= C_r_right
        d = 0.3 # Since the initial distance from the wall should be 0.3 meters
        theta_r = theta_i + pi/4 # defining an angle *hopefully* parallel to the wall
        A_r = -sin(theta_r)
        B_r = cos(theta_r)
        C_r = -x_i*sin(theta_r) + y_i*cos(theta_r)
        C_r_left = C_r + d
        C_r_right = C_r - d


        last_x = self._lastOdom.pose.pose.position.x
        last_y = self._lastOdom.pose.pose.position.y
        while True:
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                success = False

                # Perform any necessary cleanup? Flush the result since its a big o' list
                # do a test to see if this works?
                _feedback = OdomRecordFeedback()
                _result = OdomRecordResult()
                break

            # Determine angle
            # Let's just determine the angle using tf.transformations euler_from_quaternion
            # as described in https://www.theconstructsim.com/ros-qa-how-to-convert-quaternions-to-euler-angles/
            quatlist = self._lastOdom.pose.pose.orientation.x, self._lastOdom.pose.pose.orientation.y, self._lastOdom.pose.pose.orientation.z, self._lastOdom.pose.pose.orientation.w
            [_, _, new_angle] = euler_from_quaternion(quatlist) # only need yaw. note that quatlist is in order xyzw
            
            newpoint = Point32()
            x, y = self._lastOdom.pose.pose.position.x, self._lastOdom.pose.pose.position.y
            newpoint.x = x
            newpoint.y = y
            newpoint.z = new_angle*180/pi
            self._result.list_of_odoms.append(newpoint)
            # rospy.logdebug(f"Logging point no. {len(self._result.list_of_odoms)} x: {newpoint.x:.2f}, y: {newpoint.y:.2f}, theta[deg]: {newpoint.z:.2f}")
            
            # note that this should be very similar to taking the speed (I think its in the odom) and multiplying by the rate of refresh
            dx = x - last_x
            dy = y - last_y
            distance += sqrt(dx**2 + dy**2) # Note that 1 lap in a quick test was around 5.7 meters
            self._feedback.current_total = distance
            self._as.publish_feedback(self._feedback)
            last_x = x
            last_y = y

            # Check for lap completion
            #   This first method seems very risky since we could easily miss the
            #   lap moment with our slow refresh rate
            # If -distance > [some threshold, will test for this]
            #    -and distance from startpos is small
            #       Say we're done, send the result
            #if (distance > 1) and (sqrt((last_x-x_i)**2 + (last_y-y_i)**2)):
            #    pass

            # Simplest approach: just do repeat tests to see how long a lap is on average.

            # Alternatively, since we know the four walls are aligned with +x, +y, -x, -y we can use the initial heading

            # Some simple test printing
            # Are we on the road?
            ON_ROAD = ((A_r*x + B_r*y <= C_r_left) and (A_r*x + B_r*y >= C_r_right))
            
            # Are we in front of or behind the goal line?
            FRONT_OF_GOAL = (A_g*x + B_g*y >= C_g)

            # rospy.logdebug(f"On the road: {ON_ROAD}. In front of goal: {FRONT_OF_GOAL}")

            # State machine for lap tracking
            # Do we need the *on the road*? Can I just look for crossing the goal line?
            
            if Lap_State == 1: # Waiting to achieve minimum distance, be off road, and behind goal
                if (distance >= 1) and (not ON_ROAD) and (not FRONT_OF_GOAL):
                    Lap_State = 2
                    rospy.loginfo("Initial lap sequence complete. Waiting to be on road in front of goal.")
            elif Lap_State == 2: # Waiting to be on road and in front of goal
                if ON_ROAD and FRONT_OF_GOAL:
                    break

            r.sleep()
        
        if success:
            rospy.loginfo("Lap complete!")
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node("odom_recorder", log_level=rospy.DEBUG)
    myServer = OdomRecorderServerClass()

    rospy.spin()
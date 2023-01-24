#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from wall_follow.srv import FindWall, FindWallResponse

class FindWallServerObject:
    def __init__(self):
        #self.latest_scan = [0]*720 # Initialize latest scan
        self.seize_control = False # This turns on when the service is called
        # self.routine_complete = False
        self.my_twist = Twist()

    def publish_velocities(self):
        while self.seize_control:
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
            rate.sleep()
    
    def receive_scan(self, my_scan):
        if self.seize_control:

            # it has trouble when it's nearly equidistant between two walls as rotationg changes dist.
            # perhaps we get really wide for our approach margin and just say that it's in the front 1/2
            margin = 10 # margin for the wall to be in an acceptable position
            forward_speed = 0.09
            turn_speed = 0.4# 0.4 # getting odd results, likely need to sim on pc
            Front = 360
            Final = 270

            # # Is this a wall?
            # # Start from the shortest laser
            # # As we go ccw from the shortest laser, we should see small changes to the value
            # # If we see a large dropoff, that marks the end of an obstacle, and should be ignored
            # wall_dropoff = 0.1
            # # Sudden increases can mark the beginning of an obstacle
            # currently = '?'
            # wall_determination = []
            # for j, w in enumerate(my_scan.ranges):
            #     i = j-1
            #     #i = (j-1) % 720
            #     #k = (j+1) % 720

            #     d = w - my_scan.ranges[i]
            #     if abs(d) > wall_dropoff:
            #         # This is either the beginning or end of an obstacle
            #         if d > 0: # This is the end of an obstacle, our position has a much smaller distance
            #             currently = 'W'
            #         else: # This is the end of a wall
            #             currently = 'O'

            #     wall_determination.append(currently)
            # # At end
            # if wall_determination[-1] == '?': # if we got to the end with all '?'s, just call it all a wall
            #     wall_determination = ['W']*720
            # elif wall_determination[0] == '?': # if we have a '?' at the beginnning, copy the last entry over to fill the '?'s
            #     for i, wd in enumerate(wall_determination):
            #         if wd == '?':
            #             wall_determination[i] = wall_determination[-1]
            # #print(wall_determination)
            # print(f"Back {wall_determination[0]}, Right {wall_determination[180]}, Front {wall_determination[360]}, Left {wall_determination[540]}.")
            
            # filtered_scan = []
            # for wd, dist in zip(wall_determination, my_scan.ranges):
            #     if wd == 'W':
            #         filtered_scan.append(dist)
            #     else:
            #         filtered_scan.append(float('inf'))

            # # Identify which laser ray is the shortest
            # wall_dist = min(filtered_scan) # the distance to the wall
            # wall_index = filtered_scan.index(wall_dist) # the location of the nearest wall


            # Identify which laser ray is the shortest
            wall_dist = min(my_scan.ranges) # the distance to the wall
            wall_index = my_scan.ranges.index(wall_dist) # the location of the nearest wall

            infomessage = f"{wall_dist:.2f} at {wall_index} "
            if wall_dist <= 0.30: # Is the wall_distance acceptable?
                if abs(wall_index - Final) < margin: # Is it in the right spot to wrap this routine up?
                    infomessage += "Phase 4: Success! Ending routine."
                    self.my_twist.linear.x = 0 # Wrap it up.
                    self.my_twist.angular.z = 0
                    pub.publish(self.my_twist)
                    #self.routine_complete = True
                    self.seize_control = False
                else: # Else
                    infomessage += "Phase 3: Distance good, turning to put wall at requested index."
                    # Turn put the wall in the right spot.
                    self.my_twist.linear.x = 0
                    if Final - wall_index > 0:
                        self.my_twist.angular.z = -turn_speed
                    else:
                        self.my_twist.angular.z = turn_speed

            # changed from comparing with margin to comparing with 180.
            elif abs(wall_index - Front) < 180: # Else, is the wall_index good for approaching?
                # Approach
                infomessage += "Phase 2: Distance too far, wall in front. Approaching."
                self.my_twist.linear.x = forward_speed
                self.my_twist.angular.z = 0
            else: # Else
                # Turn to put the wall_index in the right spot

                # Rotate the robot until it is facing the wall
                infomessage += "Phase 1: Distance too far and wall not in front. Turning."
                self.my_twist.linear.x = 0
                if 360 - wall_index > 0:
                    self.my_twist.angular.z = -turn_speed
                else:
                    self.my_twist.angular.z = turn_speed
            print(infomessage)       
            # Move the robot forward until the front ray is smaller than 30cm
            # Rotate the robot until ray #270 is the wall



    def my_callback(self, _):
        # Turn flags and begin publishing velocities
        self.seize_control = True
        #self.rountine_complete = False
        self.publish_velocities()
        # This stuff may not be needed
        #while not self.routine_complete:
        #    rate.sleep()
        # Return True
        my_response = FindWallResponse()
        my_response.wallfound = True
        print("End")
        return my_response


my_ServerObject = FindWallServerObject()

rospy.init_node('find_wall_server')
sub = rospy.Subscriber('/scan', LaserScan, my_ServerObject.receive_scan)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

rate = rospy.Rate(2)

find_wall_service = rospy.Service('/find_wall', FindWall, my_ServerObject.my_callback)

rospy.spin()
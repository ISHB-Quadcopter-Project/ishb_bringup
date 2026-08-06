#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
import threading
import math

GOAL_TOL = 1.0 #Tolerance for checking whether odom reading match waypoint4

##
# @brief This node publishes waypoints to the /super/goal topic for the drone to follow
# @details Also, it includes a watchdog mechanism to republish the current waypoint if the drone is not moving towards it.
class ourNode:
    def __init__(self): 
        self.waypts = [
            {'x': 15.0, 'y': -3.0, 'z': 2.0},
            {'x': 0.0, 'y': -6.0, 'z': 2.0},
            {'x': 15.0, 'y': -9.0, 'z': 2.0},
            {'x': 0.0, 'y': -12.0, 'z': 2.0},
            {'x': 15.0, 'y': -15.0, 'z': 2.0},
            {'x': 0.0, 'y': -15.0, 'z': 2.0}
        ]

        # self.waypts = [
        #     {'x': 2.0, 'y': -2.5, 'z': 2.0},
        #     {'x': 4.0, 'y': -5.0, 'z': 2.0},
        #     {'x': 6.0, 'y': -7.5, 'z': 2.0},
        #     {'x': 8.0, 'y': -10.0, 'z': 2.0},
        #     {'x': 10.0, 'y': -12.5, 'z': 2.0},
        #     {'x': 12.0, 'y': -15.0, 'z': 2.0},
        #     {'x': 14.0, 'y': -12.167, 'z': 2.0},
        #     {'x': 16.0, 'y': -9.333, 'z': 2.0},
        #     {'x': 18.0, 'y': -6.5, 'z': 2.0},
        #     {'x': 20.0, 'y': -3.667, 'z': 2.0},
        #     {'x': 22.0, 'y': -0.833, 'z': 2.0},
        #     {'x': 24.0, 'y': 2.0, 'z': 2.0},
        #     {'x': 26.0, 'y': -0.833, 'z': 2.0},
        #     {'x': 28.0, 'y': -3.667, 'z': 2.0},
        #     {'x': 30.0, 'y': -6.5, 'z': 2.0},
        #     {'x': 32.0, 'y': -9.333, 'z': 2.0},
        #     {'x': 34.0, 'y': -12.167, 'z': 2.0},
        #     {'x': 36.0, 'y': -15.0, 'z': 2.0},
        #     {'x': 38.0, 'y': -12.167, 'z': 2.0},
        #     {'x': 40.0, 'y': -9.333, 'z': 2.0},
        #     {'x': 42.0, 'y': -6.5, 'z': 2.0},
        #     {'x': 44.0, 'y': -3.667, 'z': 2.0},
        #     {'x': 46.0, 'y': -0.833, 'z': 2.0},
        #     {'x': 48.0, 'y': 2.0, 'z': 2.0},
        # ]

        self.waypt_index = 0 

        self.lock = threading.Lock()

        #Odom var to hold the x,y,z odom data
        self.latest_pos = None

        self.odom_list = []

        # Topic super takes
        self.pub = rospy.Publisher("/super/goal", PoseStamped, queue_size = 10) 

        self.sub = rospy.Subscriber("/Odometry", Odometry, self.odom_cb, queue_size = 10)

        #Flag to see if there is available odom data to check dist_to_goal
        self.is_odom = False

    def publ(self):
        """!@brief Publishes the current waypoint to the /super/goal topic
        @details Currently, waypoints are hardcoded in the constructor"""

        # print("Publishing now")
        #Creating message to publish
        self.msg = PoseStamped() 
        self.msg.header.stamp = rospy.Time.now() #time stamp for msg
        self.msg.header.frame_id = "world" #frame of message

        # print(self.waypt_index)
        self.msg.pose.position.x = self.waypts[self.waypt_index]["x"]
        self.msg.pose.position.y = self.waypts[self.waypt_index]["y"]
        self.msg.pose.position.z = self.waypts[self.waypt_index]["z"]
        self.msg.pose.orientation.w = 1.0

        self.pub.publish(self.msg)

    def odom_cb(self, msg):
        """!@brief Callback function for the /Odometry topic
            @details Updates the latest odometry position and sets the is_odom flag to True. Odom data is stored in a list for the odom_watchdog to check if the drone is moving.
            @note self.lock is used to ensure other areas of code using odom data don't get partial data, as this callback is in a separate thread
            @param msg The Odometry message received from the /Odometry topic"""
        
        with self.lock:
            self.latest_pos = msg.pose.pose.position
            self.odom_list.append(self.latest_pos) #Add odom data to list for odom_watchdog
            #print("INSIDE HERE IS P: ", self.latest_pos)
            self.is_odom = True # latest_pos odom should be set by now

    def dist_to_goal(self, odom):
        """!@brief Calculates the distance from the current odometry position to the goal waypoint
            @details This function is called by the run function to check if the drone has reached the current waypoint
            @param odom The current odometry position
            @see publ"""
        # print("DIST ODOM: ", odom, "\n")

        Odomx = odom.x
        Odomy = odom.y
        Odomz = odom.z

        dist_x = Odomx - self.waypts[self.waypt_index]["x"]
        dist_y = Odomy - self.waypts[self.waypt_index]["y"]
        dist_z = Odomz - self.waypts[self.waypt_index]["z"]

        squared_sum = pow(dist_x, 2) + pow(dist_y, 2) + pow(dist_z, 2)

        distance = math.sqrt(squared_sum)
        # print("D: ", distance)
        if distance < GOAL_TOL:
            print("waypt reached")

            if self.waypt_index < len(self.waypts)-1:
                self.waypt_index += 1
            
            self.publ() #Once reached waypoint, publish next one, instead of spamming in run

            self.is_odom = False #Set back to false, so can do this func until have odom data
            
    def odom_watchdog(self):
        """!@brief Checks if the drone is moving towards the current waypoint
            @details If the drone is not moving towards the waypoint, it republishes the current waypoint to the /super/goal topic. This function is called by the run function."""
        
        #Wating until 5 secs of odom data, to see if drone moving
        if len(self.odom_list) > 50:
            delta_x = self.odom_list[-1].x - self.odom_list[0].x #last odom x - first odom x, to see if drone moved in x direction
            delta_y = self.odom_list[-1].y - self.odom_list[0].y

            if abs(delta_x) < 0.5 and abs(delta_y) < 0.5: #Checking if odom x and y changed, if so then publish goal again so drone move
                print("Delta x: ", delta_x)
                print("Delta y: ", delta_y)
                self.publ()

            self.odom_list.clear()

    def run(self):
        """!@brief Main loop of the node
            @details This function runs continuously until the node is shut down. It checks if odometry data is available, retrieves it with self.lock and calls the dist_to_goal function
            @see odom_watchdog
            @see dist_to_goal"""
        
        while(not rospy.is_shutdown()): #TODO add smt when do FSM

            self.odom_watchdog() #watchdog here to run to republish if not moving, and checks length of list

            with self.lock:
                if self.is_odom == True:
                    # print("NOW HERE")
                    odom = self.latest_pos
                    self.dist_to_goal(odom) # only runs when odom is set


def main():
    rospy.init_node("ourNode")

    ourNode().run()

    rospy.spin()

if __name__ =="__main__":
    main()

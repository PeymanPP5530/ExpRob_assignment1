#!/usr/bin/env python3

import rospy
import time
from threading import Thread
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import Twist , Point
from aruco_ros.msg import marker_data


class robot_controller:
    def __init__(self):
        
        # Initialize the node
        rospy.init_node('robot_controller')

        # Variables
        self.marker_list = [11, 12, 13, 15]
        self.marker_id = 0
        self.threshold = 10
        self.reached_ack = False
        self.camera_center = Point()
        self.marker_center = Point()
        self.vel = Twist()
        
        # Publishers
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/marker_data', marker_data, self.marker_data_callback, queue_size=1)
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_center_callback, queue_size=1)
        

    # Get the center of the camera
    def camera_center_callback(self, msg : CameraInfo):
        self.camera_center.x = msg.width / 2
        self.camera_center.y = msg.height / 2

    # Reached the marker
    def reached(self):
        self.reached_ack = False
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)
        self.marker_list.pop(0)
        print("I reached to the marker number: ", self.marker_id)
        time.sleep(2)

    # Move forward to the marker
    def approche(self):
        self.vel.linear.x = 0.4
        self.vel.angular.z = 0
        print("approaching token number: ", self.marker_list[0])

        
    # Allign the robot with the marker
    def allign(self):
        self.vel.linear.x = 0.1
        if self.camera_center.x > self.marker_center.x:
            self.vel.angular.z = 0.2
        else:
            self.vel.angular.z = -0.2
        print("allining token number: ", self.marker_list[0])

    # Rotate to find the marker
    def search(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0.6
        print("Searching for token number: ", self.marker_list[0])

    # Callback to get the marker data
    def marker_data_callback(self, msg : marker_data):
        self.marker_id = msg.marker_id.data
        self.marker_center.x= msg.marker_center.x
        self.marker_center.y= msg.marker_center.y
        if (msg.marker_perimeter.data/4) > 180 :
            self.reached_ack =True
        else:
            self.reached_ack = False


    # Control loop
    def control_loop(self):
        # loop until all markers are reached
        while  self.marker_list:
            if self.marker_id == self.marker_list[0]:
                if self.reached_ack:
                    self.reached()
                elif (self.camera_center.x < (self.marker_center.x + self.threshold)) and (self.camera_center.x > (self.marker_center.x - self.threshold)):
                    self.approche()
                else:
                    self.allign()
            else:
                self.search()
            self.vel_pub.publish(self.vel)
        
        print("I reached to all markers !")
        time.sleep(10)
        rospy.signal_shutdown("")


def main():


    # Wait for other nodes to initialize properly
    time.sleep(6)

    # Create and spin the controller node
    logic = robot_controller()

    # Spinning thread to ensure that ROS callbacks are executed
    spin_thread = Thread(target=rospy.spin)
    spin_thread.start()

    # Start the control loop
    time.sleep(1)
    logic.control_loop()


if __name__ == '__main__':
    main()

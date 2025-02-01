#!/usr/bin/python3
# -*- coding: utf-8 -*-

'''
*****************************************************************************************
*
*        		===============================================
*           		    Logistic coBot (LB) Theme (eYRC 2024-25)
*        		===============================================
*
*  This script should be used to implement Task 1B of Logistic coBot (LB) Theme (eYRC 2024-25).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ 1624 ]
# Author List:		[Subhrajit]
# Filename:		    task1b_boiler_plate.py
# Functions:
#			        [ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /etc... ]


################### IMPORT MODULES #######################

import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32MultiArray
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


focalX = 931.1829833984375 
focalY = 931.1829833984375

##################### FUNCTION DEFINITIONS #######################

def calculate_rectangle_area(coordinates):
    '''
    Description:    Function to calculate area or detected aruco

    Args:
        msg = Int32MultiArray()
        msg.data = self.detected_marker_ids  # Publish the list of
        coordinates (list):     coordinates of detected aruco (4 set of (x,y) coordinates)

    Returns:
        area        (float):    area of detected aruco
        width       (float):    width of detected aruco
    '''

    ############ Function VARIABLES ############

    # You can remove these variables after reading the instructions. These are just for sample.

    ############ ADD YOUR CODE HERE ############

    p1 = [coordinates[0][0], coordinates[0][1]]
    p2 = [coordinates[1][0], coordinates[1][1]]
    p3 = [coordinates[2][0], coordinates[2][1]]
    p4 = [coordinates[3][0], coordinates[3][1]]

    height = math.sqrt(((p2[1] - p3[1])**2) + ((p2[0] - p3[0])**2))
    width = math.sqrt(((p1[1] - p2[1])**2) + ((p1[0] - p2[0])**2))
    area = height * width

    # INSTRUCTIONS & HELP : 
    #	->  Recevice coordiantes from 'detectMarkers' using cv2.aruco library 
    #       and use these coordinates to calculate area and width of aruco detected.
    #	->  Extract values from input set of 4 (x,y) coordinates 
    #       and formulate width and height of aruco detected to return 'area' and 'width'.

    ############################################

    return area, width


def detect_aruco(image):
    '''
    Description:    Function to perform aruco detection and return each detail of aruco detected 
                    such as marker ID, distance, angle, width, center point location, etc.

    Args:
        image                   (Image):    Input image frame received from respective camera topic

    Returns:
        center_aruco_list       (list):     Center points of all aruco markers detected
        distance_from_rgb_list  (list):     Distance value of each aruco markers detected from RGB camera
        angle_aruco_list        (list):     Angle of all pose estimated for aruco marker
        width_aruco_list        (list):     Width of all detected aruco markers
        ids                     (list):     List of all aruco marker IDs detected in a single frame 
    '''

    # variable as a threshold value to detect aruco markers of certain size.
    aruco_area_threshold = 1500

    # The camera matrix is defined as per camera info loaded from the plugin used. 
    camera_matrix = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])

    # The distortion matrix is currently set to 0. 
    distance_matrix = np.array([0.0,0.0,0.0,0.0,0.0])

    # We are using 150x150 aruco marker size
    marker_size = 0.15

    ids = []
    aruco_list = []    
    axis_length = 0.1

    # Convert input BGR image to GRAYSCALE for aruco detection
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # defining aruco dictionary and parameters
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    arucoParams = cv2.aruco.DetectorParameters_create()

    # finding corners and aruco ids
    corners, ids, _ = cv2.aruco.detectMarkers(gray_image, arucoDict, parameters=arucoParams)	

    if len(corners) > 0:
        
        # flatten the ArUco IDs list
        ids = ids.flatten()
        
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
                        
            # extract the marker corners (which are always returned in top-left, top-right, bottom-right, and bottom-left order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            
            # convert each of the (x,y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
                        
            # draw the bounding box of the ArUCo detection
            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
            
            # draw the ArUco marker ID on the frame
            cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
            area, width = calculate_rectangle_area(corners)
                            
            if(area > aruco_area_threshold):
   
                # compute and draw the center (x, y)-coordinates of the ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                                
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorner, marker_size, camera_matrix, distance_matrix)
                                
                cv2.drawFrameAxes(image, camera_matrix, distance_matrix, rvecs, tvecs, axis_length)
                
                # Store ArUco marker information in the list
                aruco_list.append({'ID': markerID,
                                        'Center': (cX, cY),
                                        'Angle' : rvecs,
                                        'Distance': tvecs,
                                        'Width': width})
    
    return aruco_list


##################### CLASS DEFINITION #######################

class aruco_tf(Node):
    '''
    ___CLASS___

    Description:    Class which servers purpose to define process for detecting aruco marker and publishing tf on pose estimated.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class aruco_tf
                        All classes have a function called __init__(), which is always executed when the class is being initiated.
                        The __init__() function is called automatically every time the class is being used to create a new object.
                        You can find more on this topic here -> https://www.w3schools.com/python/python_classes.asp
        '''

        super().__init__('aruco_tf_publisher')                                          # registering node

        ############ Topic SUBSCRIPTIONS ############

        qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_ALL,
            )

        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)
        # self.marker_pub = self.create_publisher(String, '/detected_markers',10)
        self.aruco_ids_pub = self.create_publisher(Int32MultiArray, '/marker_ids', 10)



        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)

        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())
        print("Wassup")

    def depthimagecb(self, data):
        '''
        Description:    Callback function for aligned depth camera topic. 
                        Use this function to receive image depth data and convert to CV2 image

        Args:
            data (Image):    Input depth image frame received from aligned depth camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############
        # bridge = CvBridge()

        self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        print("Hello depthimagecb")

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT: You may use CvBridge to do the same

        ############################################


    def colorimagecb(self, data):
        '''
        Description:    Callback function for colour camera raw topic.
                        Use this function to receive raw image data and convert to CV2 image

        Args:
            data (Image):    Input coloured raw image frame received from image_raw camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        print("Hello depthimagecb")


        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT:   You may use CvBridge to do the same
        #               Check if you need any rotation or flipping image as input data maybe different than what you expect to be.
        #               You may use cv2 functions such as 'flip' and 'rotate' to do the same

        ############################################


    def process_image(self):
        '''
        Description:    Timer function used to detect aruco markers and publish tf on estimated poses.

        Args:
        Returns:
        '''

        ############ Function VARIABLES ############

        # These are the variables defined from camera info topic such as image pixel size, focalX, focalY, etc.
        # Make sure you verify these variable values once. As it may affect your result.
        # You can find more on these variables here -> http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        cam_mat = np.array([[focalX, 0.0, 640.0], [0.0, focalY, 360.0], [0.0, 0.0, 1.0]])
        dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

        
        aruco_list = []
        detected_markers = {}

        if self.cv_image is None:
            self.get_logger().warn("No color image received for processing!")
            return
        
        # Get aruco center, distance from rgb, angle, width and ids list from 'detect_aruco_center'
        aruco_list = detect_aruco(self.cv_image)

        # corners, ids, rejected = cv2.aruco.detectMarkers(self.cv_image, cv2.aruco_dict, parameters=aruco_params)
        
        if not aruco_list:
            self.get_logger().info("No ArUco markers detected.")
            cv2.imshow("rgb_image", self.cv_image)
            cv2.waitKey(1)

        tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.detected_marker_ids = []
        print('marker_ids initialised with list')

        # Iterate through all detected ArUco markers
        for aruco_info in aruco_list:
            
            print("Entered loop!!!!")
            marker_id = aruco_info['ID']
            center = aruco_info['Center']
            angle = aruco_info['Angle']
            distance = aruco_info['Distance']
            width = aruco_info['Width']


            rotation_matrix, _ = cv2.Rodrigues(angle)

            # Adjust fixed rotations for correct orientation
            theta_x = np.pi / 2  # Rotate around X to align coordinate system
            theta_y = 0         # No rotation about Y
            theta_z = np.pi / 2  # Rotate around Z

            # Define rotation matrices
            rot_x = np.array([
                [1, 0, 0],
                [0, np.cos(theta_x), -np.sin(theta_x)],
                [0, np.sin(theta_x), np.cos(theta_x)]
            ])

            rot_y = np.array([
                [np.cos(theta_y), 0, np.sin(theta_y)],
                [0, 1, 0],
                [-np.sin(theta_y), 0, np.cos(theta_y)]
            ])

            rot_z = np.array([
                [np.cos(theta_z), -np.sin(theta_z), 0],
                [np.sin(theta_z), np.cos(theta_z), 0],
                [0, 0, 1]
            ])

            # Additional 180-degree rotation around X-axis to flip Z-axis downward
            flip_y_90 = np.array([
                [np.cos(-np.pi/2), 0, np.sin(-np.pi/2)],
                [0, 1, 0],
                [-np.sin(-np.pi/2), 0, np.cos(-np.pi/2)]
            ])

            # Combine rotations: Apply Flip_X -> Z -> Y -> X order
            rot_correction = np.dot(flip_y_90, np.dot(rot_z, np.dot(rot_y, rot_x)))

            # Final transformation matrix
            transform_matrix = np.dot(rot_correction, rotation_matrix)

            # Convert the rotation matrix to a Rotation object
            r_euler = Rotation.from_matrix(transform_matrix)

            # Get the Euler angles in radians (XYZ order by default)
            euler_angles = r_euler.as_euler('xyz')

            # Convert adjusted Euler angles to quaternion
            r_quat = Rotation.from_euler('xyz', [np.pi / 2, -np.pi / 18, euler_angles[2]])
            quat = r_euler.as_quat()
                                                
            # Use center_aruco_list to get realsense depth and log them down.
            depth_value = self.depth_image[center[1], center[0]]  # Assuming (y, x) coordinates

            # Convert depth_value to meters (assuming depth image is in millimeters)
            depth_meters = depth_value/1000.0
            
            # Use this formula to rectify x, y, z based on focal length, center value and size of image
            x = depth_meters * (sizeCamX - center[0] - centerCamX) / focalX
            y = depth_meters * (sizeCamY - center[1] - centerCamY) / focalY
            z = depth_meters
                        
            # Create a TransformStamped message
            transform_cam = TransformStamped()
            transform_cam.header.stamp = rclpy.time.Time().to_msg()
            transform_cam.header.frame_id = 'camera_link'  
            transform_cam.child_frame_id = f'cam_{marker_id}'           
        
            # Set the translation
            transform_cam.transform.translation.x = z
            transform_cam.transform.translation.y = x
            transform_cam.transform.translation.z = y
                        
            # Set the rotation (quaternion) based on the angle
            transform_cam.transform.rotation.x = quat[0]
            transform_cam.transform.rotation.y = quat[1]
            transform_cam.transform.rotation.z = quat[2] 
            transform_cam.transform.rotation.w = quat[3]

            tf_broadcaster.sendTransform(transform_cam)
            print("Transform done!")
            
            if self.tf_buffer.can_transform('base_link', f'cam_{marker_id}', rclpy.time.Time()):

                # Lookup transform between base_link and cam_<marker_id>
                try:
                    transform_lookup = self.tf_buffer.lookup_transform('base_link', f'cam_{marker_id}', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2))
                    
                    # Publish transform from obj_<marker_id> to base_link
                    transform_base = TransformStamped()
                    transform_base.header.stamp = rclpy.time.Time().to_msg()
                    transform_base.header.frame_id = 'base_link'
                    transform_base.child_frame_id = f'obj_{marker_id}'
                    transform_base.transform = transform_lookup.transform
                    tf_broadcaster.sendTransform(transform_base)

                    translation = transform_lookup.transform.translation
                    # coords_text = f"x: {translation.x}, y: {translation.y}, z: {translation.z}"

                    print("NOICE!")

                    if marker_id not in self.detected_marker_ids:
                        self.detected_marker_ids.append(marker_id)

                    '''place for publishing coordinates to any topic'''

                    ## Overlay the text on the image at the marker's center position
                    #cv2.putText(self.cv_image, coords_text, (int(center[0]), int(center[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)


                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    self.get_logger().warn(f"Failed to lookup transform for marker ID {marker_id}")


        cv2.imshow("rgb_image", self.cv_image)
        cv2.waitKey(1)

        msg = Int32MultiArray()
        msg.data = list(map(int, self.detected_marker_ids))  # Publish the list of marker IDs
        self.aruco_ids_pub.publish(msg)
        self.get_logger().info(f"Published Aruco IDs: {self.detected_marker_ids}")



##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node

    node.get_logger().info('Node created: Aruco tf process')        # logging information

    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

    rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    '''
    Description:    If the python interpreter is running that module (the source file) as the main program, 
                    it sets the special __name__ variable to have a value “__main__”. 
                    If this file is being imported from another module, __name__ will be set to the module’s name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    '''

    main()

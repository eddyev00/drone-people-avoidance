#! /usr/bin/env python
# Import ROS.
import rospy
from yolov8_msgs.msg import DepthResult
import time
from djitellopy import tello
from cv_bridge import CvBridge
import cv2
from yolov8_msgs.msg import DepthResult #Reused message already created to send a string
from sensor_msgs.msg import Image

"""
Potential Tello ROS driver
"""
class Tello():
    def __init__(self):
        ########### Essentials for code ################
        # Initialise ROS node
        rospy.init_node("driver_tello_ros_")
        #Publisher of camera frames
        self.img_pub = rospy.Publisher ("/Tellocam/image_raw", Image, queue_size=1)
        self.bridge = CvBridge() #Transforms messages from ROS format to OpenCV format
        # Register a callback to handle ROS shutdown
        #rospy.on_shutdown(self.shutdown_callback)
        # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
        rate = rospy.Rate(10)
        
        ############ Important variables ###############
        self.speed_drone = 20
        print('speed of drone set to 20 cm/s')
        self.start_flag= False #allows drone to move after being set up.   
        self.initial_wait_done = False # waits 3 seconds for camera to connect streaming

        ######## Drone Initialization ################
        # Create an object for the API.
        self.drone = tello.Tello()
        # Wait for FCU connection.
        self.drone.connect()
        #print battery status
        print(self.drone.get_battery())
        time.sleep(3)
        #start video streaming to get frames
        self.drone.streamon()
        time.sleep(1)
        # Request takeoff with an altitude of 1.7m.
        self.drone.takeoff()
        time.sleep(5)
        #define the speed in cm/s
        #self.drone.set_speed(20)
        #time.sleep(2)
        #set height in cm. Drone usually takesoff between 70 and 80 cm.
        self.drone.move_up(75) 
        time.sleep(4)

        #Creating a subscriber for the drone control topic
        rospy.Subscriber('/command_tello', DepthResult, self.execute_command) 


        while not rospy.is_shutdown():
            img = self.drone.get_frame_read().frame
            img = cv2.resize(img, (360, 240))
            img= self.bridge.cv2_to_imgmsg(img, "bgr8") 
            self.img_pub.publish(img)
            #img = cv2.resize(img, (360, 240))
            #cv2.imshow("Image", img)
            #cv2.waitKey(1)
 


    def execute_command(self, input):

        
        # Wait for 3 seconds at the beginning
        if not self.initial_wait_done:
            time.sleep(5)#reduce if drone is landing unexpectedly (if done doesnt recive commands after 7 secs it turns off)
            self.initial_wait_done= True 
            self.start_flag = True
        

        if self.start_flag == True and not rospy.is_shutdown(): # means drone has been set up already
            command= input.depth_result #string message type basically, ignore "depth result" name.
            
            if command == 'forward':
                self.drone.send_rc_control(0, self.speed_drone, 0, 0)
                print('forward')
            elif command == 'up':
                self.drone.send_rc_control(0, 0, self.speed_drone, 0)
                print('up')
            elif command == 'down':
                self.drone.send_rc_control(0, 0, -self.speed_drone, 0)
                print('up')
            elif command == 'hover':
                self.drone.send_rc_control(0,0,0,0)
                print('hover')
            elif command == 'rotate':
                self.drone.rotate_clockwise(90)
                print('rotate')
            elif command == 'land':
                self.drone.land()
                print('land')
            else:
                print('No command received')


        return
'''    
    def shutdown_callback(self):
        # This callback will be called when the ROS node is shutting down
        # Add code here to gracefully stop the drone and release resources
        print("Shutting down...")

        # Stop video stream
        self.drone.streamoff()

        time.sleep(2)
        # Land the drone
        self.drone.land()

        # Give some time for the drone to land
        time.sleep(2)

        # Close the connection
        self.drone.end()
'''
# Driver code.
if __name__ == '__main__':

    tello = Tello()
    rospy.spin()



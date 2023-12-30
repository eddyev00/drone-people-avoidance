#! /usr/bin/env python
# Import ROS.
import rospy
# Import PoseStamped
from geometry_msgs.msg import PoseStamped  
# To print colours (optional).
from colorama import init, Fore, Style
from yolov8_msgs.msg import DepthResult
import time
from djitellopy import tello
import cv2
from sensor_msgs.msg import Image # Image message
from cv_bridge import CvBridge 


#creo que voy a tener que mandar las imagenes desde este script.
#Optitrack-> prendes vrpn -> te suscribes al topic, sacas las coordenadas.


class drone_control():

    def __init__(self):
        
        # Initialise ROS node
        rospy.init_node("drone_path_control")
        
        #Creating a subscriber for the Optitrack topic for live coordinate's reception
        #Change topic according to created object's name
        rospy.Subscriber('/vrpn_client_node/Tello/pose', PoseStamped, self.optitrack_cb,queue_size=1) 
        
        #Publisher of command messages
        self.drone_controller = rospy.Publisher ("/command_tello", DepthResult, queue_size=1)

        #string message for commands
        self.command = DepthResult()
        
        # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
        self.rate = rospy.Rate(10)
        ############## Important Variables #################################
        #avoids errors when callback function is called before we get any waypoints from the flight.
        self.start_flag=True #Camera from tello driver publishes images just before tello starts moving.

        #Control Manouver Mode flag
        self.control_mode = False
        # True - Near person, so distance is around 1 m to the person. Safety Mechanism -> wait pesrson to move or go up to an altitude of 2.1 meters 
        # False - Far from person -> distancedistance > 1.5 m

        #Safety Measure Flag -> makes safety flight after person does not move out of frame of view.
        self.safety_manouver_on=False

        #Person present and close
        self.person = True

        #time threshold for the global manouver in seconds
        self.manouver_time_threshold = 7


        #time threshold for the loop activated after no people is detected. 
        #after this time, if still no person is detected we go back to normal
        self.loop_time_threshold = 5
        #Standard altitude (altitude for takeoff and which the drone will try to maintain)
        self.standard_altitude = 1.7
        #Emergency altitude (In optitrack I decided for 2.1 meters)
        self.emergency_altitude = 2.1

        #initiate tracking coordinates
        self.current_position= [0,0,0,0]

        ######## Specify some waypoints ###########
        #square path -> ignore psi (not used here as path was predetermined) -> drone must start at specific point and orientation.
        #[point 0, point 1, point 2, back to origin -1] -> inside list
        #[x,y,normal_Z, emergency_z]-> inside coordinate list

        #initial testing wps 
        #self.wps = [[0.28,-0.95,1.70,2.10], [1.28,-0.95,1.70,2.10], [1.28,-3.45,1.70,2.10],[0.28, -3.45, 1.70, 2.10]]
        
        #expanded square wps
        self.wps = [[-0.13,-0.61,1.55,2.10], [1.72,-0.61,1.55,2.10], [1.72,-3.81,1.55,2.10],[-0.13, -3.81, 1.55, 2.10]]

        # Creating a subscriber for the topic sending qualitative depth information.
        rospy.Subscriber(name="/depth_inference_result",
                        data_class=DepthResult,
                        callback=self.control_cb,
                        queue_size=1)
        
        # Execute planned path after takeoff.
        self.execute_flight()


    #Optitrack callback function
    def optitrack_cb(self, optitrack_msg):
        # Access x, y, and z coordinates from the PoseStamped message and updates the variables.
        # global variables, so we access them directly in other functions.
        self.current_position= [optitrack_msg.pose.position.x,optitrack_msg.pose.position.y,
                                optitrack_msg.pose.position.z]
        
        # Do something with x, y, and z
        return


    #Changes control mode flag status depending on the depth estimation it receives.
    def control_cb(self,msg):

        if (self.start_flag):
            # Callback function of the subscriber.
            # Assigning bounding_boxes of the message to bbox variable.
            depth_result = msg.depth_result
            print('message received')
            # Printing the detected object with its probability in percentage.
            #rospy.loginfo("Received depth result: {}".format(depth_result))
            if depth_result == "SAFE":
                self.control_mode = False
                print(Fore.YELLOW + "SAFE" + Style.RESET_ALL)



            if depth_result == "nothing":
                self.control_mode = False
                print(Fore.BLUE + "NO PEOPLE" + Style.RESET_ALL)

            if depth_result == "NEAR": #Too close to a person
                # Setting mode_g to True to state we have to begin our safety control mechanism.
                self.control_mode = True
                print('Received depth result: TOO CLOSE')
                print(Fore.RED + "Drone is too close to a worker. Starting safety control flight." + Style.RESET_ALL)
   
    """
    #Stops drone and evaluates emergency manouver to execute.
    #Option A: Person moves and the drone can continue
    #Option B: Person doesn't move and the drone has to fly on top of the person.
    """

    def avoidance_manouver(self,i):
        
        current_side = i # saves which current side of the square we are in.
        self.command.depth_result = 'hover' #stop midair
        self.drone_controller.publish(self.command.depth_result)
        time.sleep(1) #fine tune  depending on how fast the drone needs to break.
        
        starting_time = time.time()
        manouver_time = 0 #total manouver time
        
        while manouver_time<self.manouver_time_threshold: #Checks whether totaltime set beforehand is trespassed.
            self.command.depth_result = 'hover' #stop midair
            self.drone_controller.publish(self.command.depth_result)
            
            end_time = time.time()
            manouver_time = end_time-starting_time

            if self.control_mode == True:
                print("Person is still too close")
            else:#->person moved -> checks whether person really moved away (avoids detection flickering or person coming back).
                loop_start=time.time()
                
                #if false, it means no person was detected closeby
                while self.control_mode == False:
                    self.command.depth_result = 'hover' #stop midair
                    self.drone_controller.publish(self.command.depth_result)
                    print("Person moved")
                    loop_end = time.time()
                    total_loop_time= loop_end-loop_start
                     
                    #safety measure in case this was triggered by a flickering in the values
                    if total_loop_time >self.loop_time_threshold:
                        print("No more people near")
                        #maybe we have to trigger a function with the optitrack to make it move further.
                        return  #if we break the function here is because no person is nearby after some time
                    #if loop is broken befor loop time condition, means self.control_mode == True (a person was detected again)
                    


        # threshold time has passed and therefore we do a safety measure by flying on a stated safety altitude.
        if manouver_time > self.manouver_time_threshold:
            
            self.safety_manouver_on= True

            # Alert Message
            print(Fore.GREEN + "Executing object avoidance manouver" + Style.RESET_ALL)
            
            #Flag for stop_to_safety loop
            stop_to_satefy_on = True
            #Flag for fly_above_on loop -> flying above the person
            fly_above_on= True
            #Falg for return_to_altitude loop
            return_to_altitude = True

            #To fly to safety altitude on current position
            while(stop_to_satefy_on):
                if self.check_waypoint_reached(i=current_side,j=2):
                    print (str(self.check_waypoint_reached(i=current_side,j=2)))
                    self.command.depth_result = 'hover' #stop midair.
                    self.drone_controller.publish(self.command.depth_result) #tell the drone to stop.
                    time.sleep(2)
                    stop_to_satefy_on=False#same effect as break.
                    break#same effect as flag.
                else:
                    print (str(self.check_waypoint_reached(i=current_side,j=2)))
                    self.command.depth_result = 'up'
                    self.drone_controller.publish(self.command.depth_result)
                    
                self.rate.sleep() 

            #To maintain avoidance manouver until we reach the point.
            while(fly_above_on):
                if self.check_waypoint_reached(current_side): #j is our previous i(side for the square) from the pre-avoidance commands
                    print (str(self.check_waypoint_reached(current_side)))
                    self.command.depth_result = 'hover' #stop midair.
                    self.drone_controller.publish(self.command.depth_result) #tell the drone to stop.
                    time.sleep(2)
                    fly_above_on=False
                    break
                else:
                    print (str(self.check_waypoint_reached(current_side)))
                    self.command.depth_result = 'forward'
                    self.drone_controller.publish(self.command.depth_result)
                                    
                self.rate.sleep()

            #Return to standard altitude after waypoint reached
            while(return_to_altitude):
                if self.check_waypoint_reached(i=current_side,j=1): #j is our previous i(side fo square) from the pre-avoidance commands
                    print (str(self.check_waypoint_reached(i=current_side,j=1)))
                    self.command.depth_result = 'hover' #stop midair.
                    self.drone_controller.publish(self.command.depth_result) #tell the drone to stop.
                    time.sleep(2)
                    return_to_altitude=False
                    break
                else:
                    print (str(self.check_waypoint_reached(i=current_side,j=1)))  
                    self.command.depth_result = 'down'
                    self.drone_controller.publish(self.command.depth_result)
                                  
                self.rate.sleep()

            self.command.depth_result = 'rotate'
            self.drone_controller.publish(self.command.depth_result)
            time.sleep(2)

        #avoidance method is finished. We can go back to normal mode and continue with the Waypoints.
        self.control_mode = False
        
    #function checks error between actual coordinate and the expected waypoint.
    def check_waypoint_reached(self,i,j=0):
        #i means side of square.
        #j means to check altitude from emergency manouver.By default = 0, so not checking this.

        if j==0:
            # Calculate the differences in x and y.
            dx = abs(self.current_position[0] - self.wps[i][0])
            dy = abs(self.current_position[1] - self.wps[i][1])



            if i == 0 or i == 2:
                # Check if the distance is within +- 10 cm
                if dy <= 0.1:  # 0.1 meters = 10 cm -> moving in y-direction
                    return True  # Waypoint reached
                
            if i == 1 or i == 3:
                # Check if the distance is within +- 10 cm
                if dx <= 0.1:# 0.1 meters = 10 cm -> moving in x-direction
                    return True  # Waypoint reached

        else:
            # Calculate the differences z
            dz_normal = abs(self.current_position[2] - self.wps[i][2])
            dz_safety = abs(self.current_position[2] - self.wps[i][3])
            if j== 1: #i=5 checks for 'z' or up direction (back to normal altitude)
                if dz_normal <= 0.1:  # 0.1 meters = 10 cm -> moving in z-direction
                    return True  # Waypoint reached
                
            if j== 2: #i=6 checks for 'z' or up direction (safety altitude) 
                if dz_safety <= 0.1:  # 0.1 meters = 10 cm -> moving in z-direction
                    return True  # Waypoint reached           
            

        return False
    


    def execute_flight(self):
        
        #starts process and allows callback function to work
        self.start_flag=True #erasable
        

        i = 0 # has to be reset in every beginning of the square

        #for j in range(2): -> to make the square in a loop.
        i = 0 # has to be reset in every beginning of the square
        while i < len(self.wps) and not rospy.is_shutdown(): #len gives as result 4 elements not 3 (0,1,2,3) -> check if ctl+c has not been clicked.
            if not self.control_mode:
                
                if self.check_waypoint_reached(i):
                    print (str(self.check_waypoint_reached(i)))
 
                    self.command.depth_result = 'hover'
                    self.drone_controller.publish(self.command.depth_result)
                    time.sleep(2)

                    self.command.depth_result = 'rotate'
                    self.drone_controller.publish(self.command.depth_result)
                    time.sleep(2)
                    i += 1 #next waypoint
                else:
                    self.command.depth_result = 'forward'
                    self.drone_controller.publish(self.command.depth_result)
                    print (str(self.check_waypoint_reached(i)))
    
            elif self.control_mode:
                self.avoidance_manouver(i)
                if(self.safety_manouver_on):
                    i += 1 #Waypoint should be reached already after this, so we go to the next point.
                    self.safety_manouver_on=False #safety manouver is done
                else: #person appeared but removed from fram so safety manouver did not activate.
                    continue

            self.rate.sleep() 

        # Land after all waypoints are reached.
        print(Fore.GREEN + "Navigation Successfully Finished. Proceed to Landing" + Style.RESET_ALL)
        #self.drone.land() #Landing command.
        self.command.depth_result = 'land'
        self.drone_controller.publish(self.command.depth_result)


        time.sleep(3) 
        
        # After execution of flying path, we want to shutdown node. 
        rospy.signal_shutdown('Program completed successfully.')

# Driver code.
if __name__ == '__main__':

    drone_control = drone_control()
    rospy.spin()

    
    # try:
    #     main()
    #     # Used to keep the node running.
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     rospy.signal_shutdown("KeyboardInterrupt")
    #     exit()



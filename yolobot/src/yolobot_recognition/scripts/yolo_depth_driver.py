#!/usr/bin/env python3

from ultralytics import YOLO
import rospy
import cv2
import torch
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import numpy as np
import statistics as st #Library to find the "mode" in an array
import pandas as pd
import csv
import os
from collections import deque
import datetime
from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference
from yolov8_msgs.msg import DepthResult


#setting important variables
global counter_conf
global counter_average
global counter_sum
global counter_median
global counter_mode
global upper_thr_median
global lower_thr_median
global median_depth
global length_deque
global median_deque
#counter_average= []
#counter_sum = []
counter_median = []
counter_filtered_median = []
#counter_mode = []
#counter_conf = []
median_depth=0

#fine-tune -> may depend on speed of detection as well
length_deque=50

#fine-tune to specific use
#lower_thr_median = 229

#upper_thr_median = 229
lower_thr_median = 213 #for simulation in gazebo

median_deque = deque(maxlen=length_deque)

# def toExcel():          

#     if len(counter) == 100:
#         df = pd.DataFrame(counter, columns=['Array Values'])
#         df.to_excel('/home/eddye/yolobot/src/yolobot_recognition/scripts/test_array.xlsx', index=False)
#     else:
#         ida=0


#acts like "low-pass" filter

def deque_median(median_depth):
    median_deque.append(median_depth)
    filtered_median= int(np.median(median_deque))
    #print(deque)
    return filtered_median


def depth_to_distance(median_received):

    near_or_safe =''
    print("Length of deque " + str(len(median_deque)))
    if(median_received > lower_thr_median):# and median_received<upper_thr_median)
        near_or_safe = 'NEAR'
        print('CLOSE')
        return(near_or_safe)
        
    else:
        print('SAFE')
        near_or_safe = 'SAFE'
        return(near_or_safe)
        

def toCSV():
    # Get current timestamp
    current_timestamp = datetime.datetime.now()

    # Format timestamp to show only minutes, seconds and milliseconds
    formatted_timestamp = current_timestamp.strftime('%M:%S.%f')[:-3]
    
    if len(counter_median) == 2000:
        os.system('spd-say "Lets go Bicho"')  # Linux

        with open('/home/eddye/yolobot/src/yolobot_recognition/scripts/csv_files/test_array.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            # Only write the header if the file is empty.
            if file.tell() == 0:
                writer.writerow(["Time", "Median", "Filtered Median"])
                #writer.writerow(["Average", "Median", "Mode", "Sum"])
                #writer.writerow(["Time", "Confidence Score"])
            for i in range(len(counter_median)):
                writer.writerow([formatted_timestamp, counter_median[i], counter_filtered_median[i]])
                #writer.writerow([counter_average[i], counter_median[conf], counter_mode[i], counter_sum[i]])
                #writer.writerow([formatted_timestamp, counter_conf[i]])


class Camera_subscriber:

    def __init__(self):
        #Essentials for code
        rospy.init_node('camera_subscriber') #Initiate Node
        self.bridge = CvBridge() #Transforms messages from ROS format to OpenCV format

        #Yolo initialization
        self.model = YOLO('~/yolobot/src/yolobot_recognition/scripts/yolov8n.pt') #Import Yolov8 nano model
        self.yolov8_inference = Yolov8Inference() # Create Instance of the Yolov8Inference Message Class

        ############# MiDaS (Depth Estimation) Initialization #############
        # Load the model
        self.midas = torch.hub.load('intel-isl/MiDaS', 'MiDaS_small')#Initialize MiDaS model
        self.midas.to('cuda') #use the GPU
        self.midas.eval()
        # Input transformation pipeline
        transforms = torch.hub.load('intel-isl/MiDaS', 'transforms')
        self.transform = transforms.small_transform # sets evaluation mode (no dropout or batch normalization)


        ###########Subscribers and Publishers##############
        self.subscription = rospy.Subscriber( #Create subscriber to drone's camera
            "/Tellocam/image_raw", #topic
            Image, #type
            self.camera_callback, #cb function
            queue_size=2)#play with this. og was 10
         
        # rospy.Subscriber(name="/depth_inference_result",
        #                 data_class=DepthResult,
        #                 callback=self.control_cb,
        #                 queue_size=1)


        #Visualizatiion with written information on the image.
        self.mock_pub = rospy.Publisher ("/mock_image", Image, queue_size=1)  #published results from node's code as Image to a topic for visualization

        self.img_pub = rospy.Publisher ("/image_inference_result", Image, queue_size=1)  #published results from node's code as Image to a topic for visualization
        #self.yolov8_pub = rospy.Publisher("/yolov8_inference_array", Yolov8Inference,  queue_size=1) #published results as array
        # Depth "Near" or "Far" inference.
        self.depth_pub = rospy.Publisher ("/depth_inference_result", DepthResult, queue_size=1)
        


        #Variables
        global multi_bb_buffer
        multi_bb_buffer = [] #serves to save boxes coordinates to be able to discern which box we have to take care of 
        
    """
        #video saving features
        self.frame_width = 640
        self.frame_height = 480
        self.size = (self.frame_width, self.frame_height)

        self.saved_video = cv2.VideoWriter('/home/jay/yolobot/src/yolobot_recognition/scripts/trial.mp4', 
                                cv2.VideoWriter_fourcc(*'mp4v'), 20, self.size)
    """
    
    
    
    def camera_callback(self, data):
        #to know if it's running
        #print("Callback Function has been called")

        #raw Image transformed to cv2 format.
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")

         #start time to count the FPS
        start = time.time()

        #Create black image where bb wil be located. 
        black_img=np.zeros(img.shape[:2], dtype=np.uint8)

        #Working with GPU
        imgbatch = self.transform(img).to('cuda')

        # Make a prediction
        with torch.no_grad(): #we don't need to keep track of the gradients (used for backpropagation), saving memory and makes things computation efficient
            prediction = self.midas(imgbatch) # depth prediction
            prediction = torch.nn.functional.interpolate(#rescales images
                prediction.unsqueeze(1),#adds extra dimension, as interpoalte requires it.
                size = img.shape[:2], # desired output size (same as input image)
                mode='bicubic', #interpolation method
                align_corners=False #slightly changes interpolation method
            ).squeeze() #deletes the extra dimension

        depth_map = prediction.cpu().numpy() #moves tensor to cpu and transforms it to Numpy (makes things more)
        
        #Higher precision
        depth_map = cv2.normalize(depth_map, None, 0, 1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_64F)
        depth_map = (depth_map*255).astype(np.uint8) #converting to grayscale values

        #depth_map copy to put info (text to it) such that in calculations is not affecting the final depth values.
        depth_map_copy = depth_map.copy()
        #Here the inference yolo is done
        results = self.model(
            img, #image to be evaluated
            classes= 0, #only people class detected
            conf= 0.55) #Confidence score allowed

        #print(depth_map)

        
        """ 
        #Nexts loop Gets bb Coordinates -> can be used fo control algorithm
        #We get top-left and bottom-right coordinates to be able to draw a rectangle.
        #If you want to send this as a message, you need the inference result
        message type (imported at beginnin) to be able to pass the messages!
        """ 
        #identifier (label) to denote inference of yolov8 model
        self.yolov8_inference.header.frame_id = "inference"
        #timestamp
        self.yolov8_inference.header.stamp = self.yolov8_inference.header.stamp = rospy.Time.now()


        # Iterates through each detection result from the YOLO model.
        # Each r represents a detected object.

        for r in results:
            #print(len(results))
            #print(results[0])

            #print("Resutls obtained")
            #get the al bb of all objects detected
            boxes = r.boxes
            
            #If there are no boxes, then there are no people detected.
            if not boxes:
                print("Status: No Person detected")
                person = "nothing"
                self.depth_result = DepthResult()
                self.depth_result.depth_result= person
                self.depth_pub.publish(self.depth_result.depth_result)
                
            # Analyze for each individual bb
            else:
                for box in boxes:
                    #Initializes Inference Result message Object
                    self.inference_result = InferenceResult()
                    
                    # get box coordinates in (top, left, bottom, right) format
                    b = box.xyxy[0].to('cpu').detach().numpy().copy()  
                    #object's class
                    c = box.cls
                    class_name = self.model.names[int(c)] #assigns class name
                    print(class_name)

    #############################################################3
                    bb= results
                    #conf = int(bb[4])
                    #print(str(float(box.conf)*100)+ "%")
                    
                    #counter_conf.append(float(box.conf))

                    

                    #print(formatted_timestamp)

    #############################################################3                
                    #self.inference_result.class_name = self.model.names[int(c)] # Assigns the name of the class to the class_name field of the InferenceResult.
                    
                    # Sets the coordinates of the bounding box.
                    left = int(b[0])
                    top = int(b[1])
                    right = int(b[2])
                    bottom = int(b[3])
                    # self.inference_result.top = int(b[0])
                    # self.inference_result.left = int(b[1])
                    # self.inference_result.bottom = int(b[2])
                    # self.inference_result.right = int(b[3])
                    



                    #display depth on black image (If needed)
                    #black_img[top:bottom, left:right] = depth_map[top:bottom, left:right]
                    
                    #draw bounding box using opencv
                    #cv2.rectangle(depth_map, (left,top), (right, bottom), (0, 255, 0), 2) # For RGB
                    cv2.rectangle(depth_map_copy, (left,top), (right, bottom), 255, 2) # For Grayscale. Better to have a black bb fro the analysis
                    
                    """The following statistical modes are tested """
                    #Average of depth values -> the closer you are, the higher the average (the closer to white = 256)
                    # avg_depth = np.mean(depth_map[top:bottom, left:right])
                    # print("Average: " + str(int(avg_depth))) # For visualization
                    
                    #We can also use the median
                    median_depth = np.median(depth_map[top:bottom, left:right])
                    print("Median: " + str(int(median_depth))) # For visualization

                    #We can also sum up
                    # sum_depth = np.sum(depth_map[top:bottom, left:right])
                    # print("Sum: " + str(int(sum_depth))) # For visualization

                    #We can also do the mode
                    # mode_depth = st.mode(np.ravel(depth_map[top:bottom, left:right]))
                    # print("Mode: " + str(int(mode_depth))) # For visualization

                    ########### Lists to add to csv file ##########################
                    #counter_average.append(int(avg_depth))
                    counter_median.append(int(median_depth))
                    # counter_sum.append(int(sum_depth))
                    # counter_mode.append(int(mode_depth))
                    print(len(counter_median))
                    
                    

                    ##################################
                    # =average(A3:A1001)
                    # =stdev(A3:A1001)
                    ##################################
                    
                    bb_area = (right - left) * (bottom - top)  # Calculate the area of the bounding box

                    median_depth = np.median(depth_map[top:bottom, left:right])
                    multi_bb_buffer.append((bb_area, median_depth))  # Append the area and median depth

                    # Larges area -> sort by area (lambda)
                    biggest_bb = max(multi_bb_buffer, key=lambda x: x[0])

                    #from max area, get median value and send it to get the median from the deque
                    filtered_median = deque_median(biggest_bb[1])

                    #filtered_median = deque_median(median_depth)
                    print("Filtered Median: " + str(filtered_median))
                    #cv2.putText(depth_map_copy, str(filtered_median), (right-200,bottom-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, 0, 2)
                    distance = depth_to_distance(filtered_median)
                    
                    if distance == "NEAR":
                        cv2.putText(depth_map_copy, distance, (right-100,bottom-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, 0, 2)

                    self.depth_result = DepthResult()
                    self.depth_result.depth_result= depth_to_distance (filtered_median)
                    self.depth_pub.publish(self.depth_result.depth_result)


                    ########### Lists to add to csv file ##########################
                    #counter_average.append(int(avg_depth))
                    counter_median.append(int(median_depth))
                    counter_filtered_median.append(int(filtered_median))
                    # counter_sum.append(int(sum_depth))
                    # counter_mode.append(int(mode_depth))
                    #print('Number of points: '+ str(len(counter_median)))
                    #toCSV()

                    #Threshold value to know if either far or near. This threshhold can be fine-tuned.
                    #avg_depth_threshold = 118
                    #sum_depth_threshold = 30000000


                    # if(int(avg_depth_threshold)>118): 
                    # #if(int(median_depth)>220): 
                    # #if(int(sum_depth)>30000000): 
                    # #if(int(mode_depth)>220): 
                    #     distance ='Near!'
                    #     cv2.putText(depth_map_copy, distance, (right-100,bottom-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, 255, 2)
                        
                    # else:
                    #     distance='Far'
                    #     cv2.putText(depth_map_copy, distance, (right-100,bottom-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, 255, 2)                
                    
                    #Appends this InferenceResult to the list of all results. 
                    #This last part is supposed to be sending this to another script, which draws the same boxes in opencv.
                    #Therefore we are not using this
                    #self.yolov8_inference.yolov8_inference.append(self.inference_result)  
                    
                    
        #     #rospy.loginfo()
    
        # make yolo drawing bounding boxes and labels on the detected objects in the image.  
        #annotated_frame = results[0].plot()`

        multi_bb_buffer.clear()
        #final time of process for FPS
        end = time.time()

        #how much time the processing took
        totaltime = end-start 
        fps = int((1 / totaltime)/2)   
        cv2.putText(depth_map_copy, f'FPS: {int(fps)}', (20,70), cv2.FONT_HERSHEY_SIMPLEX, 1.5, 255 ,2)
        print("FPS : " + str(int(fps)))
        # Convert the OpenCV image (depth_map) back to a ROS Image message -> we are publishing original depth image (without annotations)
        img_msg = self.bridge.cv2_to_imgmsg(depth_map) 
        depth_map_msg= self.bridge.cv2_to_imgmsg(depth_map_copy)  

        #Publish annotated image
        self.img_pub.publish(img_msg)
        self.mock_pub.publish(depth_map_msg)

        
        #Publish inference results (coordinates + object class). Not needed here
        #self.yolov8_pub.publish(self.yolov8_inference)

        #We need to clear the array every time a new message is received
        self.yolov8_inference.yolov8_inference.clear()
        #We need to clear the deque every time a new person is detected, such that previous detections don't affect.
        #median_deque.clear()



    """

    #Destructor to avoid file corruptions for not closing correctly
    def __del__(self):
        self.saved_video.release()
    """
if __name__ == '__main__':

    camera_subscriber = Camera_subscriber() #Create Instance of this class
    rospy.spin()

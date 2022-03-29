# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from geometry_msgs.msg import Point
import numpy as np
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
# from http.server import BaseHTTPRequestHandler, HTTPServer
# import  pyshine as ps
import time
import base64
import os
import argparse
# hostName = "10.2.9.32"
# serverPort = 9000


# HTML="""
# <html>
# <head>
# <title>PyShine Live Streaming</title>
# </head>

# <body>
# <center><h1> PyShine Live Streaming using OpenCV </h1></center>
# <center><img src="stream.mjpg" width='640' height='480' autoplay playsinline></center>
# </body>
# </html>
# """


class ImageObjectDetection(Node):
  """
  Create an ImageObjectDetection class, which is a subclass of the Node class.
  """
  def __init__(self):
      """
      Class constructor to set up the node
      """
      # Initiate the Node class's constructor and give it a name
      super().__init__('image_object_detection')
        
      # Create the subscriber. This subscriber will receive an Image
      # from the video_frames topic. The queue size is 10 messages.
      self.subscription = self.create_subscription(
        Image, 
        'video_frames', 
        self.listener_callback, 
        10)
      self.subscription # prevent unused variable warning
      self.publisher_=self.create_publisher(Point, 'center_pos', 10)
      # Used to convert between ROS and OpenCV images
      self.br = CvBridge()
      yolo_dir= '/home/pi/Downloads/yolov3'
      weightsPath=os.path.join(yolo_dir,'yolov3.weights')
      configPath=os.path.join(yolo_dir,'yolov3.cfg')
      labelsPath=os.path.join(yolo_dir,'coco.names')
      self.CONFIDENCE=0.5 
      self.THRESHOLD=0.4
      with open(labelsPath, 'rt') as f:
          self.labels=f.read().rstrip('\n').split('\n')
      self.COLORS=np.random.randint(0,255,size=(len(self.labels),3),dtype="uint8")
      self.net = cv2.dnn.readNet(weightsPath,configPath)
      self.display_image=False

  def getOutputsNames(self):
      # Get the names of all the layers in the network
      layersNames = self.net.getLayerNames()
     # Get the names of the output layers, i.e. the layers with unconnected outputs
      return [layersNames[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

  def yolo_v3_object_detection(self,img):

      blobImg=cv2.dnn.blobFromImage(img,1.0/255.0,(416,416),None,True,False)
      self.net.setInput(blobImg)
      # outInfo=net.getUnconnetedOutLayersNames()
      outInfo=self.getOutputsNames()
      layerOutPuts=self.net.forward(outInfo)
      (H,W)=img.shape[:2]

      boxes=[]
      confidences=[]
      classIDs=[]

      for out in layerOutPuts:
          for detection in out:
              scores=detection[5:]
              classID=np.argmax(scores)
              confidence=scores[classID]
              if confidence > self.CONFIDENCE:
                  box=detection[0:4]*np.array([W,H,W,H])
                  (centerX,centerY,width,height)=box.astype("int")
                  x=int(centerX-(width/2))
                  y=int(centerY-(height/2))
                  boxes.append([x,y,int(width),int(height)])
                  confidences.append(float(confidence))
                  classIDs.append(classID)
      idxs=cv2.dnn.NMSBoxes(boxes,confidences,self.CONFIDENCE,self.THRESHOLD)
      return idxs,boxes,confidences,classIDs



  def listener_callback(self, data):
      """
      Callback function.
      """
      # Display the message on the console
      self.get_logger().info('Receiving video frame')
  
      # Convert ROS Image message to OpenCV image
      # print(data)
      # print(type(data))
      current_frame = self.br.imgmsg_to_cv2(data)
      # print(current_frame)
      # print(type(current_frame))
      t1=time.time()
      indexs,boxes,confidences,classIDs=self.yolo_v3_object_detection(current_frame)
      
      (H,W)=current_frame.shape[:2]
      t2=time.time()
      self.get_logger().info('object detection time: {}'.format(t2-t1))
      if len(boxes) > 0:
        pos=Point()
        pos.x=(boxes[0][0]+boxes[0][2]/2.0)/W
        pos.y=(boxes[0][1]+boxes[0][3]/2.0)/H
        pos.z=0.0
        self.publisher_.publish(pos)
      if self.display_image:
        # Display image
        if len(indexs) > 0:
            for i in indexs.flatten():
                (x,y)=(boxes[i][0],boxes[i][1])
                (w,h)=(boxes[i][2],boxes[i][3])
                color=[int(c) for c in self.COLORS[classIDs[i]]]
                cv2.rectangle(current_frame,(x,y),(x+w,y+h), color,2) # width of the line is 2px
                text="{}: {:.4f}".format(self.labels[classIDs[i]],confidences[i])
                cv2.putText(current_frame,text,(x,y-5),cv2.FONT_HERSHEY_SIMPLEX,0.5,color,2)# size of the text 0.5
        cv2.imshow('object detection result',current_frame)
        
        cv2.waitKey(1)
      else:
        self.get_logger().info('no object detected')
def main(args=None):
    
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create the node
    image_object_detection = ImageObjectDetection()
    
    # Spin the node so the callback function is called.
    rclpy.spin(image_object_detection)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_object_detection.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
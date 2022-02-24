#!/usr/bin/env python3
from importlib.resources import path
from os import pread
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import torch
import numpy as np
from detetion_pkg.msg import BoundingBoxes,BoundingBox
import sys
model = torch.hub.load('ultralytics/yolov5', 'custom', path='yolov5m.pt',source='local')  # local model
# model = torch.hub.load('ultralytics/yolov5', 'yolov5s' ,pretrained=True)
# model.load_state_dict(torch.load('yolov5s.pt')['model'])
# model.load_state_dict(torch.load('yolov5s.pt')['model'].state_dict())
# model.eval()
image_pub = rospy.Publisher('/image_converter', Image, queue_size=1000)
bounding_boxes_pub = rospy.Publisher('/bounding_boxes', BoundingBoxes, queue_size=1000)

 
def callback(data):
  prev_frame_time = 0
  new_frame_time = 0
  probability=[]
  xmin=[]
  
  ymin=[]
  xmax=[]
  ymax=[]
  id_list=[]
  Class=[]
  bounding_boxes = BoundingBoxes()
  
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
 
  # Output debugging information to the terminal
  rospy.loginfo("receiving video frame")
  
  # Convert ROS Image message to OpenCV image
  current_frame = br.imgmsg_to_cv2(data)
  results = model(current_frame)
  xy_values=results.pandas().xyxy[0]
  print('xy_values',xy_values)
  id_list = xy_values.index.tolist()
  Class=xy_values["name"].values.tolist()
  probability=xy_values["confidence"].values.tolist()
  xmin=xy_values["xmin"].values.tolist()
  xmax=xy_values["xmax"].values.tolist()
  ymin=xy_values["ymin"].values.tolist()
  ymax=xy_values["ymax"].values.tolist()
  bounding_boxes.header.stamp = rospy.Time.now()
  bounding_boxes.header.frame_id = "Boson_yolo_v5"
  bounding_boxes.header.seq = 0
  bounding_boxes.image_header.stamp = rospy.Time.now()
  bounding_boxes.image_header.frame_id = "zed2_left_camera_optical_frame"
  
  for i in range(len(xmin)):
    bounding_box = BoundingBox()
    bounding_box.id = int(id_list[i])
    bounding_box.Class=str(Class[i])
    bounding_box.probability=float(probability[i])
    bounding_box.xmin=int(xmin[i])
    bounding_box.xmax=int(xmax[i])
    bounding_box.ymin=int(ymin[i])
    bounding_box.ymax=int(ymax[i])
    bounding_boxes.bounding_boxes.append(bounding_box)
  bounding_boxes_pub.publish(bounding_boxes)
  # xy_values=xy_values.astype(int)
  # print(xy_values)
  # results.show()
#   print('results',type(results.render()))
#   # Display image
  cv2.imshow("camera", np.squeeze(results.render()))
  image_pub.publish(br.cv2_to_imgmsg(np.squeeze(results.render()), "bgr8"))
  
   
  cv2.waitKey(1)
      
def receive_message():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('video_sub_py', anonymous=True)
   
  # Node is subscribing to the video_frames topic
  rospy.Subscriber('/zed2/zed_node/left/image_rect_color', Image, callback)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  receive_message()

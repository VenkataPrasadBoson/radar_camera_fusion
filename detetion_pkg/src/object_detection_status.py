from xmlrpc.client import Boolean
import rospy
from geometry_msgs.msg import PoseArray
import numpy as np


class object_detection_status:
    def __init__(self):
        rospy.Subscriber('/obstacle_pose_array', PoseArray, self.depth_callback)
        self.object_detection_status_pub = rospy.Publisher('/object_detection_status', Boolean, queue_size=10)
        self.pose_array = PoseArray()
        self.x_min = 0
        self.x_max = 3
        self.y_min = -1.5
        self.y_max = 1.5

        
    def depth_callback(self, depth_data):
        self.internal_depth_x=[]
        self.internal_depth_y=[]
        self.internal_depth_Status_append=[]
        self.pose_array = depth_data
        for let_it in range(len(self.pose_array.poses)):
            self.internal_depth_x.append(self.pose_array.poses[let_it].position.z)
            self.internal_depth_y.append(self.pose_array.poses[let_it].position.x)
        print(self.internal_depth_x,self.internal_depth_y)
        for i in range(len(self.internal_depth_x)):
            self.internal_depth_Status=None
            if self.internal_depth_x[i] < self.x_max and self.y_min < self.internal_depth_y[i] < self.y_max:
                self.internal_depth_Status=True
            else:
                self.internal_depth_Status=False
            self.internal_depth_Status_append.append(self.internal_depth_Status)
        print(self.internal_depth_Status_append)
        if any(self.internal_depth_Status_append):
            self.object_detection_status_pub.publish(True)
        else:
            self.object_detection_status_pub.publish(False)
                
                        
if __name__ == '__main__':
    rospy.init_node('object_detection_status', anonymous=True)
    object_detection_status()
    rospy.spin()
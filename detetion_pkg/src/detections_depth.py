#!/usr/bin/env python3
from math import inf, nan
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
import ros_numpy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import statistics

"""
Header header
Header image_header
BoundingBox[] bounding_boxes

bounding_boxes
    float64 probability
    int64 xmin
    int64 ymin
    int64 xmax
    int64 ymax
    int16 id
    string Class


# Intrinsic camera matrix for the raw (distorted) images.
#     [fx  0 cx]
# K = [ 0 fy cy]
#     [ 0  0  1]

K: [528.7377319335938, 0.0, 639.3441162109375, 0.0, 528.7377319335938, 367.45574951171875, 0.0, 0.0, 1.0]



points[i, 0] = (c - self.cx) / self.fx * z;
points[i, 1] = (r - self.cy) / self.fy * z;
points[i, 2] = z
"""

class ZedObjectsDepth:
    def __init__(self) -> None:
        rospy.Subscriber('/zed2/zed_node/depth/depth_registered',Image, self.depth_callback)
        rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes, self.bounding_boxes_callback )
        rospy.Subscriber('/zed2/zed_node/left/image_rect_color', Image , self.image_callback)
        # rospy.Subscriber('/zed2i/zed_node/point_cloud/cloud_registered', PointCloud2, self.pointcloud_callback)
        rospy.Subscriber('/zed2/zed_node/depth/camera_info', CameraInfo, self.camerainfo_callback)
        self.pose_array_pub = rospy.Publisher('/obstacle_pose_array', PoseArray, queue_size= 100)
        self.detection_image_pub = rospy.Publisher('detections_image', Image, queue_size=100)
        self.bridge = CvBridge()
        self.pose_array = PoseArray()
        self.pose = Pose()


        self.area_div = 2
    def depth_callback(self, depth_data):
        try:
            self.depth = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")  # depth_image = bridge.imgmsg_to_cv2(depth_data, "32FC1")
            # print(self.depth_image)

        except  Exception as  e:
            print('Error in rendeing depth image ', e)
        pass
    def camerainfo_callback(self, camera_info):
        K = camera_info.K
        self.cx, self.cy = K[2], K[5]
        self.fx, self.fy = K[0], K[4]
        # print('center: -',self.cx, self.cy)
        # print('focal : -',self.fx, self.fy)
        

        pass
    def bounding_boxes_callback(self, data):
        self.pose_array.poses = []
        for i in range(len(data.bounding_boxes)):
            bounds = data.bounding_boxes[i]

            # bounds = [  data.bounding_boxes[0].xmin, 
            #             data.bounding_boxes[0].ymin,
            #             data.bounding_boxes[0].xmax,
            #             data.bounding_boxes[0].ymax]
            # depth_val = self.get_object_depth(bounds)
            # print(depth_val)
            depth_val = self.center_depth(bounds)
            
            print('--------------')
            print('obj : ', data.bounding_boxes[i].Class)
            # print('dis : ',depth_val)
            # prin'obj(depth_val)
            start_point = bounds.xmin,bounds.ymin
            end_point = bounds.xmax, bounds.ymax
            label = data.bounding_boxes[i].Class
            color = (255, 0, 0)
            thickness = 1
            print('depth_val',depth_val)
            if depth_val[0] != -1:
                distance = "{:.2f}".format(depth_val[2])
                image = cv2.rectangle(self.img_data, start_point, end_point, color, thickness) 
                r= cv2.putText(self.img_data, label + " " +  str(distance)+" m" ,
                        (start_point[0] + (thickness ), start_point[1] + (10 + thickness )),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
            
                self.pose.position.x = depth_val[0]
                self.pose.position.y = depth_val[1]
                self.pose.position.z = depth_val[2]
                self.pose.orientation.y = 1

                self.pose_array.poses.append(self.pose)
            
        self.pose_array_pub.publish(self.pose_array)

        self.detection_image_pub.publish(self.bridge.cv2_to_imgmsg(self.img_data))
        # cv2.imshow("Show",self.img_data)

        # cv2.waitKey(3)  
            # cv2.destroyAllWindows()

            # cv2.rectangle(self.img_data,(x,y),(x+w,y+h),(0,255,0),2) 


            # distance = "{:.2f}".format(distance)
            # thickness = 1
            # cv2.rectangle(image, (x_coord - thickness, y_coord - thickness),
            #                 (x_coord + x_extent + thickness, y_coord + (18 + thickness*4)),
            #                 color_array[detection[3]], -1)
            # cv2.putText(image, label + " " +  (str(distance) + " m"),
            #             (x_coord + (thickness * 4), y_coord + (10 + thickness * 4)),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            # cv2.rectangle(image, (x_coord - thickness, y_coord - thickness),
            #                 (x_coord + x_extent + thickness, y_coord + y_extent + thickness),
            #                 color_array[detection[3]], int(thickness*2))
        

    def image_callback(self, image_data):
        try:
            self.img_data = np.frombuffer(image_data.data, dtype=np.uint8).reshape(image_data.height, image_data.width, -1)
            # self.img_data = self.bridge.imgmsg_to_cv2(image_data,  encoding="passthrough")  # depth_image = bridge.imgmsg_to_cv2(depth_data, "32FC1")
            # print(self.depth_image)

        except  Exception as  e:
            print('Error in rendeing depth image ', e)
        pass
    def pointcloud_callback(self, pc_msg):
        pc_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc_msg, remove_nans=True)
        # pc_pcl = pcl.PointCloud(np.array(pc_np, dtype=np.float32))
        print(pc_np.shape)
        return pc_np

    def center_depth(self,bounds):
        # try:
            # print(bounds)
            x_mid = (bounds.xmin + bounds.xmax)//2
            y_mid = (bounds.ymin + bounds.ymax)//2

            x_vect = []
            y_vect = []
            z_vect = []

            for i in range(int(x_mid - self.area_div), int(x_mid + self.area_div)):
                for j in range(int(y_mid - self.area_div), int(y_mid + self.area_div)):
                    z = self.depth[j, i]
                    if not np.isnan(z) and not np.isinf(z):
                        x = (i - self.cx) / self.fx * z
                        x_vect.append(x)
                        y = (j - self.cy) / self.fy * z
                        y_vect.append(y)
                        z_vect.append(z)
            try:
                x_median = statistics.median(x_vect)
                y_median = statistics.median(y_vect)
                z_median = statistics.median(z_vect)
            except Exception:
                x_median = -1
                y_median = -1
                z_median = -1
                pass

            return x_median, y_median, z_median





            dis = self.depth[x_mid,y_mid]
            x = (x_mid - self.cx) / self.fx * dis
            y = (y_mid - self.cy) / self.fy * dis


            print(x,y,dis)
            return x,y,dis

            # (c - self.cx) / self.fx, 0)

            # return dis
        # except Exception as e:
        #     print(e)
        #     return nan,nan,nan
        # print('dis',dis)

    def convert_pc_msg_to_np(self,pc_msg):
        # Conversion from PointCloud2 msg to np array.
        pc_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc_msg, remove_nans=True)
        # pc_pcl = pcl.PointCloud(np.array(pc_np, dtype=np.float32))
        return pc_np  # point cloud in numpy and pcl format


  
     


    def get_object_depth(self, bounds):
        '''
        Calculates the median x, y, z position of top slice(area_div) of point cloud
        in camera frame.
        Arguments:
            depth: Point cloud data of whole frame.
            bounds: Bounding box for object in pixels.
                bounds[0]: x-center
                bounds[1]: y-center
                bounds[2]: width of bounding box.
                bounds[3]: height of bounding box.

        Return:
            x, y, z: Location of object in meters.
        '''
        area_div = 2

        x_vect = []
        y_vect = []
        z_vect = []

        for j in range(int(bounds[0] - area_div), int(bounds[0] + area_div)):
            for i in range(int(bounds[1] - area_div), int(bounds[1] + area_div)):
                z = self.depth[i, j, 2]
                if not np.isnan(z) and not np.isinf(z):
                    x_vect.append(self.depth[i, j, 0])
                    y_vect.append(self.depth[i, j, 1])
                    z_vect.append(z)
        try:
            x_median = statistics.median(x_vect)
            y_median = statistics.median(y_vect)
            z_median = statistics.median(z_vect)
        except Exception:
            x_median = -1
            y_median = -1
            z_median = -1
            pass

        return x_median, y_median, z_median



if __name__ == "__main__":
    rospy.init_node('zed_objects_depth', anonymous=True)
    z=ZedObjectsDepth()
    rospy.loginfo('Node started')
    rospy.spin()
    
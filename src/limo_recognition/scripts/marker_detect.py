#!/usr/bin/env python
# -*- coding: utf-8 -*-

# failed because of python version
import rospy
import cv2
import sys
print(sys.version)
print(cv2.__version__)
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image,CameraInfo
import numpy as np
from math import *
from geometry_msgs.msg import Pose,PointStamped
import tf2_ros
import tf2_geometry_msgs
import random

HUE_LOW   = 0
HUE_HIGH  = 10
SATURATION_LOW  = 120
SATURATION_HIGH = 255
VALUE_LOW    = 70
VALUE_HIGH   = 255

class image_converter:
    def __init__(self):    
    # 创建cv_bridge，声明图像的发布者和订阅者
      self.bridge=CvBridge()	#ROS图像和OpenCV图像信息的转换
      self.image_sub=rospy.Subscriber("/camera/color/image_raw", Image, self.visual_callback)	#订阅Image，Camera的话题
      self.depth_sub=rospy.Subscriber("/camera/depth/image_raw",Image,self.depth_callback)    # 获取深度信息
      self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info",CameraInfo,self.camera_info_callback)   # 获取相机内参
      self.image_pub=rospy.Publisher("object_detect_image", Image, queue_size=1)	#发布识别结果
      self.target_pub=rospy.Publisher("/object_detect_pose", PointStamped, queue_size=1)	#发布target的Pose信息
      self.tf_buffer = tf2_ros.Buffer()
      self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def visual_callback(self,data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            image_input = self.bridge.imgmsg_to_cv2(data, "bgr8")	#将ROS中拿到的数据转换成OpenCV能够使用的数据
        except CvBridgeError as e:
            print(e)

        gray = cv2.cvtColor(image_input,cv2.COLOR_BGR2GRAY)  #将获得的bgr图转化为hsv图，这样更利于我们在真实环境中识别物体
        # print("Size of image:", cv_image.shape) #(480,640,3)

        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)   
        results = []
        if ids is not None:
            aruco.drawDetectedMarkers(image_input, corners, ids)	    
            camera_matrix = np.array([[self.camera_info.K[0], 0, self.camera_info.K[2]], [0, self.camera_info.K[4], self.camera_info.K[5]], [0, 0, 1]])
            dist_coeffs = np.zeros((4,1))
            marker_length = 0.1
            for i, corner in enumerate(corners):
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corner, marker_length, camera_matrix, dist_coeffs)
                aruro.drawAxis(image_input, camera_matrix, dist_coeffs, rvec, tvec, marker_length*0.5)
		
                origin_3d = np.array([[0,0,0]])
                image_points, _ = cv2.projectPoints(origin_3d, rvec, tvec, camera_matrix, dist_coeffs)
                origin_pi = image_points[0][0]
                cX, cY = int(origin_pi[0]), int(origin_pi[1])
                cX = cX if cX < 400 else 399
                cY = cY if cY < 640 else 639
                print(cX, cY)
                cv2.circle(image_input, (cX, cY), 3, (255, 0, 0), -1)


                depth = self.depth_image[cX,cY]

                # 计算物体在相机坐标系中的位置,像素坐标系-->相机坐标系
                point_camera = PointStamped()
                point_camera.header.frame_id = data.header.frame_id
                point_camera.point.x = (cX - self.camera_info.K[2]) * depth / self.camera_info.K[0]
                point_camera.point.y = (cY - self.camera_info.K[5]) * depth / self.camera_info.K[4]
                point_camera.point.z = depth

                # 使用tf2将点从相机坐标系转换到世界坐标系
                point_world = self.tf_buffer.transform(point_camera,"odom")
                results.append([depth, point_world])
	
            max_depth_index = 0
            for i in range(len(results)):
                if result[i][0] > results[max_depth_index][0]:
                    max_depth_index = i
            self.target_pub.publish(results[max_depth_index][1])
            

        # 显示Opencv格式的图像
        cv2.imshow("Image window", image_input)
        # cv2.imshow("Image window", mask)
        cv2.waitKey(3)

        # 再将opencv格式额数据转换成ros image格式的数据发布
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "rgb8"))
        except CvBridgeError as e:
            print(e)

    def depth_callback(self,data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")  # "32FC1"表示32位浮点数单通道，常用于表示深度图像

    def camera_info_callback(self,data):
        self.camera_info = data



if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("object_detect")
        rospy.loginfo("Starting detect object")
        image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down object_detect node.")
        cv2.destroyAllWindows()

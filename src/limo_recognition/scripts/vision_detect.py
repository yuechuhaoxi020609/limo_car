#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs

class YOLODetector:
    def __init__(self):
        # 创建 cv_bridge，声明图像的订阅者和目标位置的发布者
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)  # 深度图订阅
        self.image_sub=rospy.Subscriber("/camera/color/image_raw", Image, self.visual_callback)	#订阅Image，Camera的话题
        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)  # 相机内参订阅
        self.yolo_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.yolo_callback)  # YOLO 检测结果订阅
        self.target_pub = rospy.Publisher("/object_detect_pose", PointStamped, queue_size=1)  # 发布目标位置
        self.image_pub = rospy.Publisher("object_detect_image", Image, queue_size=1)  # 发布处理后的图像
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.depth_image = None
        self.camera_info = None

    def depth_callback(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")  # 深度图像为 32 位浮点单通道


    def camera_info_callback(self, data):
        self.camera_info = data

    def visual_callback(self,data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            self.image_input = self.bridge.imgmsg_to_cv2(data, "bgr8")	#将ROS中拿到的数据转换成OpenCV能够使用的数据
        except CvBridgeError as e:
            print(e)

    def yolo_callback(self, data):
        if self.depth_image is None or self.camera_info is None:
            print("Depth image or camera info not available yet.")
            return

        # 处理每个检测框
        results = []
        for box in data.bounding_boxes:
            if box.Class != "bottle":  # 只处理目标类别为 "cola can"
                continue

            # 计算目标框中心
            cX = int((box.xmin + box.xmax) / 2)
            cY = int((box.ymin + box.ymax) / 2)
            print(cX, cY)

            if not (0 <= cX < self.depth_image.shape[1] and 0 <= cY < self.depth_image.shape[0]):
                rospy.logwarn("Bounding box center out of depth image range.")
                continue

            # 获取目标深度
            depth = self.depth_image[cY, cX]
            print(cY, cX, depth)

            # 计算相机坐标系下的 3D 坐标
            point_camera = PointStamped()
            point_camera.header.frame_id = self.camera_info.header.frame_id
            point_camera.point.x = (cX - self.camera_info.K[2]) * depth / self.camera_info.K[0]
            point_camera.point.y = (cY - self.camera_info.K[5]) * depth / self.camera_info.K[4]
            point_camera.point.z = depth

            # 转换到世界坐标系
            point_world = self.tf_buffer.transform(point_camera, "odom")
            results.append([depth, point_world])

            # 在图像上绘制目标框
            cv2.rectangle(self.image_input, (box.xmin, box.ymin), (box.xmax, box.ymax), (0, 255, 0), 2)
            cv2.putText(self.image_input, box.Class, (box.xmin, box.ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        if len(results) > 0:  
            min_depth_index = 0
            for i in range(len(results)):
                if results[i][0] < results[min_depth_index][0]:
                    min_depth_index = i
            self.target_pub.publish(results[min_depth_index][1])

        # 显示Opencv格式的图像
        cv2.imshow("Image window", self.image_input)
        cv2.waitKey(3)

            # 发布处理后的图像
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.image_input, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr("Image publish error")

if __name__ == '__main__':
    rospy.init_node("yolo_object_detect")
    rospy.loginfo("Starting YOLO object detection")
    detector = YOLODetector()
    rospy.spin()


#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, cv2, cv_bridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import String
#rospy:提供許多簡易模組可以使用
#cv2:用於影像相關
#cv_bridge:opencv與ros間的轉換                                  

class Tracking:

    def __init__(self):
      self.bridge = cv_bridge.CvBridge()
      self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
	  #'/camera/color/image_raw'
      #self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
      self.move_pub = rospy.Publisher('GoToMove', String, queue_size=50)
	  #針對cmd_vel這個topic發訊息，格式為twist，一次最多一筆資料
     # self.twist = Twist()
	

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # opencv image use bgr8 -> ros image bgr8:藍綠紅顏色順序的彩色圖像
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # change HSV
        lower_green = np.array([156, 43, 46])
        upper_green = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green) # cv2.inRange(img,low,high)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
			#cv2.circle(影像, 圓心座標, 半徑, 顏色, 線條寬度)
            cv2.circle(image, center, 5, (0, 0, 255), -1)
            h, w, d = image.shape
            size = image.shape
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            err = cx - w / 2
            move_str = str(int(radius) ) + " " + str(err)
            self.move_pub.publish(move_str)

        if len(cnts) <= 0:
            print( "no orange" )
            move_str = "0"
            self.move_pub.publish(move_str)
			
        cv2.namedWindow("window2", cv2.WINDOW_NORMAL)
        cv2.imshow("window2", image)
        
if __name__ == '__main__':

    try:
        rospy.init_node('track', anonymous=True) 
		#初始化track這個node並且匿名，匿名好處是因為node名稱不能重複，匿名一次可以執行多個node
        rospy.loginfo("Starting cv_bridge_test node")
		#印出在command line上
        track = Tracking()
        rospy.spin()

    except KeyboardInterrupt:
        print ("Shutting down cv_bridge_test node.")
        cv2.destroyAllWindows()
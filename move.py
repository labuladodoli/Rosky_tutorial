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

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = -2.84
#burger型車
#BURGER_MAX_LIN_VEL:直線運動最大速度
#BURGER_MAX_ANG_VEL:旋轉運動最大角速度

WAFFLE_MAX_LIN_VEL = 0.26
#/camera/depth/image_rect_raw
WAFFLE_MAX_ANG_VEL = 1.82
#waffle型車

LIN_VEL_STEP_SIZE = 0.005
ANG_VEL_STEP_SIZE = 0.1
control_linear_vel  = 0.0
target_linear_vel   = 1.0
#LIN_VEL_STEP_SIZE:直線運動勻速步進範圍
#ANG_VEL_STEP_SIZE:旋轉運動角速步進範圍
class Moving:

    def __init__(self):
      self.bridge = cv_bridge.CvBridge()
      self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
      self.sub = rospy.Subscriber('GoToMove',String,self.callbackmove )
	  #針對cmd_vel這個topic發訊息，格式為twist，一次最多一筆資料
      self.twist = Twist()
	
    def checkLinearLimitVelocity(self, vel):

        vel = self.constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

        return vel

    def constrain(self, input, low, high):
        if input > high:
          input = high
        elif input < low:
          input = low
        else:
          input = input

        return input

		
		
    def callbackmove(self, data):
        cnts = 0
        str1 = str(data.data)
        if str1 == 0 :
	  cnts = -1
        else:	
            str2 = str1.split(" ")
            radius = int( str2[0] )
            err = str2[1]
            print(radius,err)
        if radius > 45  and radius < 160 :
            global target_linear_vel
            self.twist.linear.x = 1.0
            target_linear_vel = 1.0
            self.twist.angular.z = -float(err) / 200
            self.cmd_vel_pub.publish(self.twist)
        elif radius > 160  :
            global control_linear_vel

            target_linear_vel = self.checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
            if target_linear_vel <= 0 :
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
            else :
                print( target_linear_vel ) 
                print( "---" ) 
                self.twist.linear.x = target_linear_vel
                print( 'go back' )
                self.cmd_vel_pub.publish(self.twist)
        elif radius < 45 :
            print( "orange<45" )
            self.twist.linear.x = 0.0
            self.twist.angular.z =  -0.4
            self.cmd_vel_pub.publish(self.twist)

        if cnts < 0:
            print( "no orange" )
            self.twist.linear.x = 0.0
            self.twist.angular.z = -0.4
            self.cmd_vel_pub.publish(self.twist)
               

        elif  0xFF == ord('q'):
            self.twist.linear.x = 0.0; #twist.linear.y = 0.0; twist.linear.z = 0.0
            self.twist.angular.z = 0.0
if __name__ == '__main__':

    try:
        rospy.init_node('moving', anonymous=True) 
		#初始化track這個node並且匿名，匿名好處是因為node名稱不能重複，匿名一次可以執行多個node
        rospy.loginfo("Starting moving node")
		#印出在command line上
        move = Moving()
        rospy.spin()

    except KeyboardInterrupt:
        print ("Shutting down moving node.")
        cv2.destroyAllWindows()
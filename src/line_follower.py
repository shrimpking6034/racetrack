#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy, imutils
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()

    # def detect_lane(self, img):
    #     tmp = img.copy()
    #     h, w, d = img.shape
        

    def image_callback(self, msg):

        # get image from camera
        image = self.bridge.imgmsg_to_cv2(msg)

        # filter out everything that's not yellow
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        white = numpy.array([ 0, 0, 170])
        red = numpy.array([ 255, 255, 255])
        lower_gray = numpy.array([ 5, 5, 50])
        upper_gray = numpy.array([ 40, 40, 80])
        maskline = cv2.inRange(hsv,  white, red)
        mask = cv2.inRange(hsv,  lower_gray, upper_gray)
        masked = cv2.bitwise_and(image, image, mask=mask)

    # clear all but a 60 pixel band near the top of the image
        h, w, d = image.shape
        search_top = h/2
        search_bot = 3 * h /4
        # search_top = 3 * h /4
        # search_bot = search_top + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        maskline[0:search_top, 0:w] = 0
        maskline[search_bot:h, 0:w] = 0
        cv2.imshow("lane", maskline)
        cv2.imshow("band", mask)

        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00']) 
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
            # Move at 0.2 M/sec
            # add a turn if the robot is not on the line
            err = cx - w/2 
            self.twist.linear.x = 1.5
            self.twist.angular.z = -float(err) / 1000
            # print("Line detected. Following the line!")
            self.cmd_vel_pub.publish(self.twist)
        cv2.imshow("image", image)
        cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
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

    def canny(self, img):
        return cv2.Canny(img, 50, 200)
    
    def drawlines(self, img, lines, color = [255, 0, 0], thickness = 5):
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(img, (x1, y1), (x2, y2), color, thickness)
        return img

    def houghlines(self, img, rho, theta, threshold, minline, maxline):
        lines = cv2.HoughLinesP(img, rho, theta, threshold, minLineLength = minline, maxLineGap =  maxline)
        line_img = self.drawlines(img, lines)
        return line_img

    def roi(self, img, vertices):
        h, w, d = 0, 0, 0
        if len(img.shape) == 3:
            h, w, d = img.shape
        else:
            h, w, = img.shape
        img[0:h/2, 0:w] = 0
        img[h-60:h, 0:w] = 0
        return img
        # return cv2.bitwise_and(img, maskline)


    def image_callback(self, msg):
        # get image from camera
        image = self.bridge.imgmsg_to_cv2(msg)
        h, w, d = image.shape
        region_of_interest_vertices = [(0, h), (w / 2, h / 2),(w, h),]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        white = numpy.array([ 0, 0, 170])
        red = numpy.array([ 255, 255, 255])
        lower_gray = numpy.array([ 5, 5, 50])
        upper_gray = numpy.array([ 40, 40, 80])
        forline = cv2.inRange(hsv,  white, red)
        blur_gray = cv2.GaussianBlur(forline, (5,5), 0)
        edges = self.canny(blur_gray)
        
        imshape = image.shape
        vertices = numpy.array([[(100, imshape[0]), (450, 320), (550, 320), (imshape[1]-20, imshape[0])]], dtype = numpy.int32)
        maskline = self.roi(edges, vertices)
        lines = self.houghlines(maskline, 1, numpy.pi/270, 30, 50, 200)

        mask = cv2.inRange(hsv,  lower_gray, upper_gray)
        masked = cv2.bitwise_and(image, image, mask=mask)

    # clear all but a 60 pixel band near the top of the image
        search_top = h/2
        search_bot = 3 * h /4
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        cv2.imshow("lane", lines)
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
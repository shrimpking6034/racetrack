#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy, imutils
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

#global variables


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.watching = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()

    # for cv2.Canny
    def canny(self, img):
        return cv2.Canny(img, 50, 200)
    
    # draw lines on the img
    def drawlines(self, img, lines, color = [255, 0, 0], thickness = 5):
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(img, (x1, y1), (x2, y2), color, thickness)
        return img

    # detect lines using houghlinesP
    def houghlines(self, img, rho, theta, threshold, minline, maxline):
        lines = cv2.HoughLinesP(img, rho, theta, threshold, minLineLength = minline, maxLineGap =  maxline)
        line_img = self.drawlines(img, lines)
        return line_img

    # region of interest, limit where to edit
    def roi(self, img, vertices):
        h, w, d = 0, 0, 0
        if len(img.shape) == 3:
            h, w, d = img.shape
        else:
            h, w, = img.shape
        img[0:h/2, 0:w] = 0
        img[h-60:h, 0:w] = 0
        return img

    # callback function for scan anticipates if an object is in the way
    def scan_callback(self, msg):
        global center
        global left
        global right
        center = []
        left = []
        right = []
        for i in range(15):
            center.append(msg.ranges[345+i])
        for u in range(15):
            center.append(msg.ranges[0+i])
        right = msg.ranges[30]
        left = msg.ranges[330]

        for a in range(30):
            while a < .2:
                if right < left:
                    self.twist.angular.z = -.2
                else:
                    self.twist.angular.z = .2

    # call back method for image
    def image_callback(self, msg):
        # get image from camera
        image = self.bridge.imgmsg_to_cv2(msg)
        h, w, d = image.shape
        region_of_interest_vertices = [(0, h), (w / 2, h / 2),(w, h),]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # to detect road using white and red blocks
        white = numpy.array([ 0, 0, 170])
        red = numpy.array([255, 255, 255])

        # to detect road using gray areas in limited space
        lower_gray = numpy.array([ 5, 5, 50])
        upper_gray = numpy.array([ 40, 40, 80])
        
        # modifying image to detect lanes and cones
        forline = cv2.inRange(hsv,  white, red)
        blur_gray = cv2.GaussianBlur(forline, (5,5), 0)
        edges = self.canny(blur_gray)
        imshape = image.shape
        vertices = numpy.array([[(100, imshape[0]), (450, 320), (550, 320), (imshape[1]-20, imshape[0])]], dtype = numpy.int32)
        maskline = self.roi(edges, vertices)
        lines = self.houghlines(maskline, 1, numpy.pi/270, 30, 50, 200)

        #masked image of the road
        mask = cv2.inRange(hsv,  lower_gray, upper_gray)
        masked = cv2.bitwise_and(image, image, mask=mask)

    # clear all near the top of the image
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

            #draw Centroid
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
            # Move at 1.5 M/sec
            # add a turn if the robot is not on the line
            err = cx - w/2 
            self.twist.linear.x = 2 # faster
            self.twist.angular.z = -float(err) / 1000

            self.cmd_vel_pub.publish(self.twist)
        cv2.imshow("image", image)
        cv2.waitKey(3)



    


# set up nodes
rospy.init_node('follower')
follower = Follower()

rospy.spin()
 #!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from linefollower.py import Follower
from obstacle_avoider.py import are_we_clear()
from obstacle_avoider.py import which_direction()

# Creating an instance of Follower
rospy.init_node('racer')
racer = Follower()

def readyset():
    while (rospy.is_not_shutdown()):
        if(not are_we_clear()):
            

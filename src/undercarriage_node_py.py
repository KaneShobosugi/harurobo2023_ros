# !/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import numpy as np


class Wheel():
    def __init__(self) -> None:
        pass


class Undercarriage():
    def __init__(self) -> None:
        self.joySub_ = rospy.Subscriber
        self.canPub_ = rospy.Publisher

        rospy.spin()

    def joyCallback(_joy):
        if(_jo)



if __name__ == '__main__':
        Undercarriage()

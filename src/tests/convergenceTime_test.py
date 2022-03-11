#!/usr/bin/env python3

from copy import copy
from time import time

from torch import true_divide
import rospy
from std_msgs.msg import Float32, String, Bool   #Message is single float number
import numpy as np
import random
import matplotlib as mpl
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose, PoseStamped
from tracker_visp.msg import *
from tracker_visp.srv import *

global force_array, time_array, time_initial, initialized_time
force_array = np.array([], dtype=float)
time_array = np.array([], dtype=float)
initialized_time = False
written = False

def timeCallback(time):
    global time_array, time_initial, initialized_time, written

    if not time.data:
        time_initial = rospy.Time.now()
        print("Started!")
        written = False

    elif time.data:
        if not written:
            time_data = round((rospy.Time.now().to_sec() - time_initial.to_sec()), 2)
            file.write(str(time_data))
            print("Finished!")
            print("\n")
            file.write("\n")
            written = True


if __name__ == '__main__':
    policy = rospy.init_node("Convergence_time")

    file_name = "Convergence_time"
    file = open(file_name + ".txt","w+")

    rate = rospy.Rate(100)
    subForce = rospy.Subscriber('/convergence_time_test', Bool, timeCallback)

    while not rospy.is_shutdown():
        rate.sleep()

 
    
    file.close()

        
#!/usr/bin/env python

import functools
import rospy

from std_msgs.msg import Float32MultiArray

rospy.init_node("delay")
def delayed_callback(msg, event):
    pub.publish(msg)

    
def callback(msg):
#     val = rospy.get_param("delay")
#     val = str(val)
#     val=val.replace(',','.')
#     val = float(val) + 0.0
    val=0.120
    timer = rospy.Timer(rospy.Duration(val), #delay topic of 250 ms
                            functools.partial(delayed_callback, msg),
                            oneshot=True)
    
print "delay node started with 120ms"
sub = rospy.Subscriber("sensors_FSR_2_delay", Float32MultiArray, callback, queue_size=100)
pub = rospy.Publisher("sensors_FSR", Float32MultiArray, queue_size=100)
rospy.spin()


#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import json
import datetime
import sys


def stop_cmd():
    pub = rospy.Publisher('symbol_grounding', String, queue_size=10)
    rospy.loginfo("send a stop command to BT")
    
    d = {"timestamp": str(datetime.datetime.now()),
         "params": {"1": {"primitive_action": "stop",
                          "object": "na",
                          "source": "na",
                          "destination": "na"}},
                          "param_num": 1 }            
    
    pub.publish(json.dumps(d, encoding='ascii'))
    rospy.sleep(0.01)
    pub.publish(json.dumps(d, encoding='ascii'))
    rospy.sleep(0.01)
    pub.publish(json.dumps(d, encoding='ascii'))
    rospy.sleep(0.01)


if __name__ == '__main__':
    rospy.init_node('stop_cmd')
    rospy.sleep(1)

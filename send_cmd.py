#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from threading import Thread

class Center(object):
    msg = """
    Arm control
    -----------------------------------------
    Initialize, Grab, Putback, Poke
    -----------------------------------------
    """
    def __init__(self):
        print self.msg
        rospy.init_node('send_cmd')
        self._rate = rospy.Rate(10)
        self.pub = rospy.Publisher('Arm_cmd',String, queue_size=10)
        t1 = Thread(target=self.enterCmd)
        t1.daemon = True
        t1.start()

    def enterCmd(self):
        while not rospy.is_shutdown():
            try:
                cmd = str(raw_input(""))
                self.pub.publish(cmd)
                rospy.loginfo('Published %s'%cmd)
                self._rate.sleep()
            except:
                rospy.loginfo("Error!!")

cmdCenter = Center()
rospy.spin()

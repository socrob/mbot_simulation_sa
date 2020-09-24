#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class CMDVelRepublisher(object):
    def __init__(self):
        '''
        cmd_vel republisher, inputs cmd_vel Twist msg and republishes last received value
        for a certain amount of time at a certain frequency (can be adjusted by tuning parameters)
        '''
        rospy.Subscriber('cmd_vel_constant', Twist, self.callback)
        self.pub = rospy.Publisher('cmd_vel_prio_medium', Twist, queue_size=1)
        self.twist_msg = Twist()
        self.set_twist_to_zero()
        # ------parameters
        self.node_frequency = 60         # frequency in hz at which the node will run
        seconds_to_republish = 3         # the aproximate amount of time in seconds that you would like the msg to be republished
        # ------
        self.max_republished_msgs = self.node_frequency * seconds_to_republish    # republish cmd_vel a maximum amount of X msgs
        self.count = 0
        self.work = False
        rospy.loginfo('cmd_vel republisher node initialized...')

    def callback(self, msg):
        self.twist_msg = msg
        self.count = 0
        self.work = True
    
    def set_twist_to_zero(self):
        self.twist_msg.linear.x = 0.0
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.z = 0.0
        self.twist_msg.angular.x = 0.0
        self.twist_msg.angular.y = 0.0
        self.twist_msg.angular.z = 0.0
        self.pub.publish(self.twist_msg)
    
    def start_cmd_republisher(self):
        rate = rospy.Rate(self.node_frequency)
        while not rospy.is_shutdown():
            if self.work:
                # publish last cmd_vel value if needed
                self.pub.publish(self.twist_msg)
                self.count = self.count + 1
                if self.count >= self.max_republished_msgs:
                    self.set_twist_to_zero()
                    self.work = False
            rate.sleep()

def main():
    rospy.init_node('cmd_vel_republisher', anonymous=False)
    cmd_republisher = CMDVelRepublisher()
    cmd_republisher.start_cmd_republisher()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
临时车轮硬件节点 - 发布车轮 joint_states
用于补齐 robot_state_publisher 需要的车轮关节状态
"""

import rospy
from sensor_msgs.msg import JointState

class WheelTFBroadcaster:
    def __init__(self):
        rospy.init_node('wheel_tf_broadcaster')
        self.publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.rate = rospy.Rate(20)

        self.wheel_joint_names = [
            'front_right_wheel_joint',
            'front_left_wheel_joint',
            'behind_right_wheel_joint',
            'behind_left_wheel_joint',
        ]

        rospy.loginfo('Wheel joint state broadcaster started')
    
    def broadcast_wheels(self):
        """定期发布所有车轮的 joint state"""
        while not rospy.is_shutdown():
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = self.wheel_joint_names
            msg.position = [0.0, 0.0, 0.0, 0.0]
            self.publisher.publish(msg)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        broadcaster = WheelTFBroadcaster()
        broadcaster.broadcast_wheels()
    except rospy.ROSInterruptException:
        pass

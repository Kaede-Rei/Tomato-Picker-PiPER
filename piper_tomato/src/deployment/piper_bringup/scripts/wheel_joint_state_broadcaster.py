#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
临时车轮硬件节点 - 发布车轮 joint_states

用途：
1. 补齐整车 robot_description 中车轮关节的 joint_state
2. 让 robot_state_publisher 能持续发布 wheel_link -> base_link 的 TF
3. 后续真实底盘接入时，可以直接把 position/velocity 替换成硬件反馈
"""

import rospy
from sensor_msgs.msg import JointState


class WheelJointStateBroadcaster:
    def __init__(self):
        rospy.init_node("wheel_joint_state_broadcaster")

        self.publish_topic = rospy.get_param("~publish_topic", "/joint_states")
        self.rate_hz = float(rospy.get_param("~rate_hz", 100.0))

        self.stamp_offset = float(rospy.get_param("~stamp_offset", 0.0))

        self.wheel_joint_names = rospy.get_param("~wheel_joint_names", [
            "front_right_wheel_joint",
            "front_left_wheel_joint",
            "behind_right_wheel_joint",
            "behind_left_wheel_joint",
        ])

        self.publisher = rospy.Publisher(
            self.publish_topic,
            JointState,
            queue_size=1,
            tcp_nodelay=True,
        )

        self.position = [0.0] * len(self.wheel_joint_names)
        self.velocity = [0.0] * len(self.wheel_joint_names)
        self.effort = [0.0] * len(self.wheel_joint_names)

        rospy.loginfo(
            "Wheel joint state broadcaster started: topic=%s, rate=%.1f Hz, joints=%s",
            self.publish_topic,
            self.rate_hz,
            self.wheel_joint_names,
        )

    def spin(self):
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            msg = JointState()
            msg.header.stamp = rospy.Time.now() + rospy.Duration(self.stamp_offset)
            msg.name = self.wheel_joint_names
            msg.position = self.position
            msg.velocity = self.velocity
            msg.effort = self.effort

            self.publisher.publish(msg)
            rate.sleep()


if __name__ == "__main__":
    try:
        WheelJointStateBroadcaster().spin()
    except rospy.ROSInterruptException:
        pass
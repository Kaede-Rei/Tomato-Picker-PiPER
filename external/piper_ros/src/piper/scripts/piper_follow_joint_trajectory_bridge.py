#!/usr/bin/env python3

import threading

import actionlib
import rospy
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)
from sensor_msgs.msg import JointState


class PiperFollowJointTrajectoryBridge:
    def __init__(self):
        self._joint_names = rospy.get_param(
            "~joints", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        )
        self._command_topic = rospy.get_param("~command_topic", "/piper/joint_ctrl_command")
        self._default_velocity = float(rospy.get_param("~default_velocity", 50.0))
        self._rate_hz = float(rospy.get_param("~rate", 50.0))
        self._lock = threading.Lock()

        self._pub = rospy.Publisher(self._command_topic, JointState, queue_size=10)
        self._server = actionlib.SimpleActionServer(
            "~follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self._server.start()
        rospy.loginfo(
            "PiPER FollowJointTrajectory bridge ready: %s -> %s",
            rospy.resolve_name("~follow_joint_trajectory"),
            self._command_topic,
        )

    def _execute(self, goal):
        with self._lock:
            result = FollowJointTrajectoryResult()
            trajectory = goal.trajectory

            if not trajectory.points:
                result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
                result.error_string = "trajectory has no points"
                self._server.set_aborted(result, result.error_string)
                return

            index_by_name = {name: idx for idx, name in enumerate(trajectory.joint_names)}
            missing = [name for name in self._joint_names if name not in index_by_name]
            if missing:
                result.error_code = FollowJointTrajectoryResult.INVALID_JOINTS
                result.error_string = "missing joints: " + ", ".join(missing)
                self._server.set_aborted(result, result.error_string)
                return

            start_time = rospy.Time.now()
            rate = rospy.Rate(self._rate_hz)

            for point in trajectory.points:
                if rospy.is_shutdown():
                    result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
                    self._server.set_aborted(result, "ROS shutdown")
                    return

                if self._server.is_preempt_requested():
                    self._server.set_preempted()
                    return

                target_time = start_time + point.time_from_start
                while rospy.Time.now() < target_time:
                    if self._server.is_preempt_requested():
                        self._server.set_preempted()
                        return
                    rate.sleep()

                command = JointState()
                command.header.stamp = rospy.Time.now()
                command.name = list(self._joint_names)
                command.position = [
                    point.positions[index_by_name[name]] for name in self._joint_names
                ]
                command.velocity = [0.0] * len(self._joint_names) + [self._default_velocity]
                command.effort = []
                self._pub.publish(command)

                feedback = FollowJointTrajectoryFeedback()
                feedback.header.stamp = command.header.stamp
                feedback.joint_names = command.name
                feedback.desired.positions = command.position
                self._server.publish_feedback(feedback)

            result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
            self._server.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node("piper_follow_joint_trajectory_bridge")
    PiperFollowJointTrajectoryBridge()
    rospy.spin()

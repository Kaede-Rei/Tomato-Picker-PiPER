#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import math
import signal
import subprocess

import rospy
import actionlib

from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState

from piper_msgs.srv import QueryArm, QueryArmRequest
from piper_msgs.srv import ConfigArm, ConfigArmRequest
from piper_msgs.msg import MoveArmAction, MoveArmGoal
from piper_msgs.msg import SimpleMoveArmAction, SimpleMoveArmGoal


class Runner:
    def __init__(self):
        self.pass_items = []
        self.fail_items = []
        self.warn_items = []
        self.launch_proc = None

        self.query_srv_name = rospy.get_param("~arm_query_service", "/arm_query")
        self.config_srv_name = rospy.get_param("~arm_config_service", "/arm_config")
        self.move_arm_ns = rospy.get_param("~move_arm_action", "/move_arm")
        self.simple_move_arm_ns = rospy.get_param(
            "~simple_move_arm_action", "/simple_move_arm"
        )

        # 如果你的真实关节名不同，可以 rosparam 或环境变量改掉
        self.test_joint_name = rospy.get_param(
            "~test_joint_name", os.environ.get("TEST_JOINT_NAME", "link1")
        )

        # 是否执行 HOME / MOVE_TO_ZERO 这类“可能真的会动”的测试
        self.run_reset_tests = rospy.get_param("~run_reset_tests", True)
        self.run_end_tests = rospy.get_param("~run_end_tests", True)

        self.cur_pose = None
        self.cur_joints = []

    def info(self, msg):
        print(f"[INFO] {msg}")

    def ok(self, msg):
        print(f"[PASS] {msg}")
        self.pass_items.append(msg)

    def warn(self, msg):
        print(f"[WARN] {msg}")
        self.warn_items.append(msg)

    def fail(self, msg):
        print(f"[FAIL] {msg}")
        self.fail_items.append(msg)

    def summary(self):
        print("\n==================== 测试汇总 ====================")
        print(f"PASS 数量 : {len(self.pass_items)}")
        print(f"WARN 数量 : {len(self.warn_items)}")
        print(f"FAIL 数量 : {len(self.fail_items)}")

        if self.pass_items:
            print("\n[PASS 列表]")
            for x in self.pass_items:
                print(f"  - {x}")

        if self.warn_items:
            print("\n[WARN 列表]")
            for x in self.warn_items:
                print(f"  - {x}")

        if self.fail_items:
            print("\n[FAIL 列表]")
            for x in self.fail_items:
                print(f"  - {x}")

        print()
        return 0 if not self.fail_items else 1

    def start_launch(self):
        self.info("启动 piper_interface piper_start.launch ...")
        self.launch_proc = subprocess.Popen(
            ["roslaunch", "piper_interface", "piper_start.launch"]
        )
        self.info(f"roslaunch 已启动，PID={self.launch_proc.pid}")

    def stop_launch(self):
        if self.launch_proc and self.launch_proc.poll() is None:
            self.info(f"关闭 roslaunch (PID={self.launch_proc.pid}) ...")
            try:
                self.launch_proc.send_signal(signal.SIGINT)
                self.launch_proc.wait(timeout=10)
            except Exception:
                self.launch_proc.kill()

    def wait_for_system(self):
        try:
            rospy.wait_for_service(self.query_srv_name, timeout=90.0)
            rospy.wait_for_service(self.config_srv_name, timeout=90.0)
            self.ok("QueryArm / ConfigArm 服务已上线")
        except Exception as e:
            self.fail(f"服务未正常上线: {e}")

        self.move_client = actionlib.SimpleActionClient(self.move_arm_ns, MoveArmAction)
        self.simple_client = actionlib.SimpleActionClient(
            self.simple_move_arm_ns, SimpleMoveArmAction
        )

        if self.move_client.wait_for_server(rospy.Duration(60.0)):
            self.ok("move_arm action 已上线")
        else:
            self.fail("move_arm action 未上线")

        if self.simple_client.wait_for_server(rospy.Duration(60.0)):
            self.ok("simple_move_arm action 已上线")
        else:
            self.fail("simple_move_arm action 未上线")

        try:
            rospy.wait_for_message("/joint_states", JointState, timeout=15.0)
            self.ok("/joint_states 已收到消息")
        except Exception as e:
            self.fail(f"/joint_states 未正常收到消息: {e}")

        try:
            hz_text = subprocess.check_output(
                [
                    "bash",
                    "-lc",
                    "timeout 3 rostopic hz /joint_states 2>/dev/null || true",
                ],
                text=True,
            ).strip()
            if hz_text:
                self.info("/joint_states hz 输出：")
                print(hz_text)
        except Exception:
            pass

    def query_proxy(self):
        return rospy.ServiceProxy(self.query_srv_name, QueryArm)

    def config_proxy(self):
        return rospy.ServiceProxy(self.config_srv_name, ConfigArm)

    def check_query_current_joints(self):
        self.info("测试 QueryArm: GET_CURRENT_JOINTS")
        try:
            srv = self.query_proxy()
            req = QueryArmRequest()
            req.command_type = QueryArmRequest.GET_CURRENT_JOINTS
            req.values = []

            res = srv(req)
            print(res)

            if not res.success:
                self.fail(f"GET_CURRENT_JOINTS 返回 success=False: {res.message}")
                return

            if len(res.cur_joint) == 0:
                self.fail("GET_CURRENT_JOINTS 返回空关节数组")
                return

            self.cur_joints = list(res.cur_joint)
            self.ok(f"GET_CURRENT_JOINTS 成功，关节数={len(self.cur_joints)}")
        except Exception as e:
            self.fail(f"GET_CURRENT_JOINTS 调用异常: {e}")

    def check_query_current_pose(self):
        self.info("测试 QueryArm: GET_CURRENT_POSE")
        try:
            srv = self.query_proxy()
            req = QueryArmRequest()
            req.command_type = QueryArmRequest.GET_CURRENT_POSE
            req.values = []

            res = srv(req)
            print(res)

            if not res.success:
                self.fail(f"GET_CURRENT_POSE 返回 success=False: {res.message}")
                return

            q = res.cur_pose.orientation
            q_norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
            if q_norm < 1e-6:
                self.fail("GET_CURRENT_POSE 返回了无效四元数")
                return

            self.cur_pose = res.cur_pose
            self.ok("GET_CURRENT_POSE 成功")
        except Exception as e:
            self.fail(f"GET_CURRENT_POSE 调用异常: {e}")

    def check_config_orientation_constraint(self):
        self.info("测试 ConfigArm: SET_ORIENTATION_CONSTRAINT")
        try:
            srv = self.config_proxy()
            req = ConfigArmRequest()
            req.command_type = ConfigArmRequest.SET_ORIENTATION_CONSTRAINT
            req.quaternion = Quaternion(0.0, 0.0, 0.0, 1.0)
            req.point = Point(0.0, 0.0, 0.0)
            req.joint_names = []
            req.joints = []
            req.values = []

            res = srv(req)
            print(res)

            if res.success:
                self.ok("SET_ORIENTATION_CONSTRAINT 成功")
            else:
                self.fail(f"SET_ORIENTATION_CONSTRAINT 失败: {res.message}")
        except Exception as e:
            self.fail(f"SET_ORIENTATION_CONSTRAINT 调用异常: {e}")

    def check_config_position_constraint(self):
        self.info("测试 ConfigArm: SET_POSITION_CONSTRAINT")
        try:
            srv = self.config_proxy()
            req = ConfigArmRequest()
            req.command_type = ConfigArmRequest.SET_POSITION_CONSTRAINT
            req.quaternion = Quaternion(0.0, 0.0, 0.0, 1.0)
            req.point = Point(0.3, 0.0, 0.2)
            req.joint_names = []
            req.joints = []
            req.values = [0.05, 0.05, 0.05]

            res = srv(req)
            print(res)

            if res.success:
                self.ok("SET_POSITION_CONSTRAINT 成功")
            else:
                self.fail(f"SET_POSITION_CONSTRAINT 失败: {res.message}")
        except Exception as e:
            self.fail(f"SET_POSITION_CONSTRAINT 调用异常: {e}")

    def check_config_joint_constraint(self):
        self.info("测试 ConfigArm: SET_JOINT_CONSTRAINT")
        try:
            srv = self.config_proxy()
            req = ConfigArmRequest()
            req.command_type = ConfigArmRequest.SET_JOINT_CONSTRAINT
            req.quaternion = Quaternion(0.0, 0.0, 0.0, 1.0)
            req.point = Point(0.0, 0.0, 0.0)
            req.joint_names = [self.test_joint_name]
            req.joints = [0.0]
            # dispatcher 按 joint_count*2 / joint_count*2+1 读范围值
            req.values = [0.1, 0.1]

            res = srv(req)
            print(res)

            if res.success:
                self.ok(f"SET_JOINT_CONSTRAINT 成功（joint={self.test_joint_name}）")
            else:
                self.fail(f"SET_JOINT_CONSTRAINT 失败: {res.message}")
        except Exception as e:
            self.fail(f"SET_JOINT_CONSTRAINT 调用异常: {e}")

    def _identity_pose(self):
        p = Pose()
        p.position.x = 0.0
        p.position.y = 0.0
        p.position.z = 0.0
        p.orientation.x = 0.0
        p.orientation.y = 0.0
        p.orientation.z = 0.0
        p.orientation.w = 1.0
        return p

    def _require_current_state(self):
        if self.cur_pose is None:
            self.warn("当前 pose 不可用，使用单位姿态兜底")
            self.cur_pose = self._identity_pose()
        if not self.cur_joints:
            self.warn("当前 joints 不可用，使用全零兜底")
            self.cur_joints = [0.0] * 6

    def send_move_arm(self, name, fill_goal_fn, timeout=30.0):
        self.info(f"测试 ArmMoveAction: {name}")
        try:
            goal = MoveArmGoal()
            fill_goal_fn(goal)
            self.move_client.send_goal(goal)
            finished = self.move_client.wait_for_result(rospy.Duration(timeout))
            if not finished:
                self.move_client.cancel_goal()
                self.fail(f"{name} 超时")
                return

            state = self.move_client.get_state()
            result = self.move_client.get_result()
            print(result)

            if result and result.success:
                self.ok(f"ArmMoveAction::{name} 成功")
            else:
                self.fail(f"ArmMoveAction::{name} 失败, state={state}, result={result}")
        except Exception as e:
            self.fail(f"ArmMoveAction::{name} 异常: {e}")

    def send_simple_move_arm(self, name, fill_goal_fn, timeout=30.0):
        self.info(f"测试 SimpleMoveArmAction: {name}")
        try:
            goal = SimpleMoveArmGoal()
            fill_goal_fn(goal)
            self.simple_client.send_goal(goal)
            finished = self.simple_client.wait_for_result(rospy.Duration(timeout))
            if not finished:
                self.simple_client.cancel_goal()
                self.fail(f"{name} 超时")
                return

            state = self.simple_client.get_state()
            result = self.simple_client.get_result()
            print(result)

            if result and result.success:
                self.ok(f"SimpleMoveArmAction::{name} 成功")
            else:
                self.fail(
                    f"SimpleMoveArmAction::{name} 失败, state={state}, result={result}"
                )
        except Exception as e:
            self.fail(f"SimpleMoveArmAction::{name} 异常: {e}")

    def test_move_arm_all(self):
        self._require_current_state()

        def dummy_point(g):
            g.target_type = MoveArmGoal.TARGET_POINT
            g.point = Point(0.0, 0.0, 0.0)

        if self.run_reset_tests:
            self.send_move_arm(
                "HOME",
                lambda g: (
                    setattr(g, "command_type", MoveArmGoal.HOME),
                    dummy_point(g),
                    setattr(g, "joint_names", []),
                    setattr(g, "joints", []),
                    setattr(g, "values", []),
                    setattr(g, "waypoints", []),
                ),
            )

        self.send_move_arm(
            "MOVE_JOINTS",
            lambda g: (
                setattr(g, "command_type", MoveArmGoal.MOVE_JOINTS),
                dummy_point(g),
                setattr(g, "joint_names", []),
                setattr(g, "joints", self.cur_joints),
                setattr(g, "values", []),
                setattr(g, "waypoints", []),
            ),
        )

        self.send_move_arm(
            "MOVE_TARGET",
            lambda g: (
                setattr(g, "command_type", MoveArmGoal.MOVE_TARGET),
                setattr(g, "target_type", MoveArmGoal.TARGET_POSE),
                setattr(g, "pose", self.cur_pose),
                setattr(g, "joint_names", []),
                setattr(g, "joints", []),
                setattr(g, "values", []),
                setattr(g, "waypoints", []),
            ),
        )

        self.send_move_arm(
            "MOVE_TARGET_IN_EEF_FRAME",
            lambda g: (
                setattr(g, "command_type", MoveArmGoal.MOVE_TARGET_IN_EEF_FRAME),
                setattr(g, "target_type", MoveArmGoal.TARGET_POSE),
                setattr(g, "pose", self._identity_pose()),
                setattr(g, "joint_names", []),
                setattr(g, "joints", []),
                setattr(g, "values", []),
                setattr(g, "waypoints", []),
            ),
        )

        if self.run_end_tests:
            self.send_move_arm(
                "TELESCOPIC_END",
                lambda g: (
                    setattr(g, "command_type", MoveArmGoal.TELESCOPIC_END),
                    dummy_point(g),
                    setattr(g, "joint_names", []),
                    setattr(g, "joints", []),
                    setattr(g, "values", [0.0]),
                    setattr(g, "waypoints", []),
                ),
            )

            self.send_move_arm(
                "ROTATE_END",
                lambda g: (
                    setattr(g, "command_type", MoveArmGoal.ROTATE_END),
                    dummy_point(g),
                    setattr(g, "joint_names", []),
                    setattr(g, "joints", []),
                    setattr(g, "values", [0.0]),
                    setattr(g, "waypoints", []),
                ),
            )

        self.send_move_arm(
            "MOVE_LINE",
            lambda g: (
                setattr(g, "command_type", MoveArmGoal.MOVE_LINE),
                dummy_point(g),
                setattr(g, "joint_names", []),
                setattr(g, "joints", []),
                setattr(g, "values", []),
                setattr(g, "waypoints", [self.cur_pose, self.cur_pose]),
            ),
        )

        self.send_move_arm(
            "MOVE_BEZIER",
            lambda g: (
                setattr(g, "command_type", MoveArmGoal.MOVE_BEZIER),
                dummy_point(g),
                setattr(g, "joint_names", []),
                setattr(g, "joints", []),
                setattr(g, "values", []),
                setattr(g, "waypoints", [self.cur_pose, self.cur_pose, self.cur_pose]),
            ),
        )

        self.send_move_arm(
            "MOVE_DECARTES",
            lambda g: (
                setattr(g, "command_type", MoveArmGoal.MOVE_DECARTES),
                dummy_point(g),
                setattr(g, "joint_names", []),
                setattr(g, "joints", []),
                setattr(g, "values", []),
                setattr(g, "waypoints", [self.cur_pose, self.cur_pose]),
            ),
        )

        if self.run_reset_tests:
            self.send_move_arm(
                "MOVE_TO_ZERO",
                lambda g: (
                    setattr(g, "command_type", MoveArmGoal.MOVE_TO_ZERO),
                    dummy_point(g),
                    setattr(g, "joint_names", []),
                    setattr(g, "joints", []),
                    setattr(g, "values", []),
                    setattr(g, "waypoints", []),
                ),
            )

    def test_simple_move_arm_all(self):
        self._require_current_state()

        q = self.cur_pose.orientation
        # 这里只给 0，简单 action 会转成 RPY；不强求和当前姿态一致，只求走通接口
        zero_rpy = (0.0, 0.0, 0.0)

        def fill_dummy_arrays(g):
            g.x = [0.0]
            g.y = [0.0]
            g.z = [0.0]
            g.roll = [0.0]
            g.pitch = [0.0]
            g.yaw = [0.0]

        if self.run_reset_tests:
            self.send_simple_move_arm(
                "HOME",
                lambda g: (
                    setattr(g, "command_type", SimpleMoveArmGoal.HOME),
                    setattr(g, "target_type", SimpleMoveArmGoal.TARGET_POINT),
                    fill_dummy_arrays(g),
                    setattr(g, "joint_names", []),
                    setattr(g, "joints", []),
                    setattr(g, "values", []),
                ),
            )

        self.send_simple_move_arm(
            "MOVE_JOINTS",
            lambda g: (
                setattr(g, "command_type", SimpleMoveArmGoal.MOVE_JOINTS),
                setattr(g, "target_type", SimpleMoveArmGoal.TARGET_POINT),
                fill_dummy_arrays(g),
                setattr(g, "joint_names", []),
                setattr(g, "joints", self.cur_joints),
                setattr(g, "values", []),
            ),
        )

        self.send_simple_move_arm(
            "MOVE_TARGET",
            lambda g: (
                setattr(g, "command_type", SimpleMoveArmGoal.MOVE_TARGET),
                setattr(g, "target_type", SimpleMoveArmGoal.TARGET_POSE),
                setattr(g, "x", [self.cur_pose.position.x]),
                setattr(g, "y", [self.cur_pose.position.y]),
                setattr(g, "z", [self.cur_pose.position.z]),
                setattr(g, "roll", [zero_rpy[0]]),
                setattr(g, "pitch", [zero_rpy[1]]),
                setattr(g, "yaw", [zero_rpy[2]]),
                setattr(g, "joint_names", []),
                setattr(g, "joints", []),
                setattr(g, "values", []),
            ),
        )

        self.send_simple_move_arm(
            "MOVE_TARGET_IN_EEF_FRAME",
            lambda g: (
                setattr(g, "command_type", SimpleMoveArmGoal.MOVE_TARGET_IN_EEF_FRAME),
                setattr(g, "target_type", SimpleMoveArmGoal.TARGET_POSE),
                setattr(g, "x", [0.0]),
                setattr(g, "y", [0.0]),
                setattr(g, "z", [0.0]),
                setattr(g, "roll", [0.0]),
                setattr(g, "pitch", [0.0]),
                setattr(g, "yaw", [0.0]),
                setattr(g, "joint_names", []),
                setattr(g, "joints", []),
                setattr(g, "values", []),
            ),
        )

        if self.run_end_tests:
            self.send_simple_move_arm(
                "TELESCOPIC_END",
                lambda g: (
                    setattr(g, "command_type", SimpleMoveArmGoal.TELESCOPIC_END),
                    setattr(g, "target_type", SimpleMoveArmGoal.TARGET_POINT),
                    fill_dummy_arrays(g),
                    setattr(g, "joint_names", []),
                    setattr(g, "joints", []),
                    setattr(g, "values", [0.0]),
                ),
            )

            self.send_simple_move_arm(
                "ROTATE_END",
                lambda g: (
                    setattr(g, "command_type", SimpleMoveArmGoal.ROTATE_END),
                    setattr(g, "target_type", SimpleMoveArmGoal.TARGET_POINT),
                    fill_dummy_arrays(g),
                    setattr(g, "joint_names", []),
                    setattr(g, "joints", []),
                    setattr(g, "values", [0.0]),
                ),
            )

        self.send_simple_move_arm(
            "MOVE_LINE",
            lambda g: (
                setattr(g, "command_type", SimpleMoveArmGoal.MOVE_LINE),
                setattr(g, "target_type", SimpleMoveArmGoal.TARGET_POSE),
                setattr(g, "x", [self.cur_pose.position.x, self.cur_pose.position.x]),
                setattr(g, "y", [self.cur_pose.position.y, self.cur_pose.position.y]),
                setattr(g, "z", [self.cur_pose.position.z, self.cur_pose.position.z]),
                setattr(g, "roll", [0.0, 0.0]),
                setattr(g, "pitch", [0.0, 0.0]),
                setattr(g, "yaw", [0.0, 0.0]),
                setattr(g, "joint_names", []),
                setattr(g, "joints", []),
                setattr(g, "values", []),
            ),
        )

        self.send_simple_move_arm(
            "MOVE_BEZIER",
            lambda g: (
                setattr(g, "command_type", SimpleMoveArmGoal.MOVE_BEZIER),
                setattr(g, "target_type", SimpleMoveArmGoal.TARGET_POSE),
                setattr(
                    g,
                    "x",
                    [
                        self.cur_pose.position.x,
                        self.cur_pose.position.x,
                        self.cur_pose.position.x,
                    ],
                ),
                setattr(
                    g,
                    "y",
                    [
                        self.cur_pose.position.y,
                        self.cur_pose.position.y,
                        self.cur_pose.position.y,
                    ],
                ),
                setattr(
                    g,
                    "z",
                    [
                        self.cur_pose.position.z,
                        self.cur_pose.position.z,
                        self.cur_pose.position.z,
                    ],
                ),
                setattr(g, "roll", [0.0, 0.0, 0.0]),
                setattr(g, "pitch", [0.0, 0.0, 0.0]),
                setattr(g, "yaw", [0.0, 0.0, 0.0]),
                setattr(g, "joint_names", []),
                setattr(g, "joints", []),
                setattr(g, "values", []),
            ),
        )

        self.send_simple_move_arm(
            "MOVE_DECARTES",
            lambda g: (
                setattr(g, "command_type", SimpleMoveArmGoal.MOVE_DECARTES),
                setattr(g, "target_type", SimpleMoveArmGoal.TARGET_POSE),
                setattr(g, "x", [self.cur_pose.position.x, self.cur_pose.position.x]),
                setattr(g, "y", [self.cur_pose.position.y, self.cur_pose.position.y]),
                setattr(g, "z", [self.cur_pose.position.z, self.cur_pose.position.z]),
                setattr(g, "roll", [0.0, 0.0]),
                setattr(g, "pitch", [0.0, 0.0]),
                setattr(g, "yaw", [0.0, 0.0]),
                setattr(g, "joint_names", []),
                setattr(g, "joints", []),
                setattr(g, "values", []),
            ),
        )

        if self.run_reset_tests:
            self.send_simple_move_arm(
                "MOVE_TO_ZERO",
                lambda g: (
                    setattr(g, "command_type", SimpleMoveArmGoal.MOVE_TO_ZERO),
                    setattr(g, "target_type", SimpleMoveArmGoal.TARGET_POINT),
                    fill_dummy_arrays(g),
                    setattr(g, "joint_names", []),
                    setattr(g, "joints", []),
                    setattr(g, "values", []),
                ),
            )

    def run(self):
        self.start_launch()
        time.sleep(5.0)

        self.wait_for_system()

        self.check_query_current_joints()
        self.check_query_current_pose()

        self.check_config_orientation_constraint()
        self.check_config_position_constraint()
        self.check_config_joint_constraint()

        self.test_move_arm_all()
        self.test_simple_move_arm_all()

        return self.summary()


def main():
    rospy.init_node("piper_full_interface_tester", anonymous=True)
    runner = Runner()
    try:
        code = runner.run()
    finally:
        runner.stop_launch()
    sys.exit(code)


if __name__ == "__main__":
    main()

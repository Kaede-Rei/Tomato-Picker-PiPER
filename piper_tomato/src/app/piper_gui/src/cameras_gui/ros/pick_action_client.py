from piper_msgs2.msg import SimpleMoveArmAction, SimpleMoveArmGoal
from piper_msgs2.msg import PickTaskAction, PickTaskGoal
from tf.transformations import quaternion_from_euler
from typing import Optional, Tuple

import actionlib
import rospy


class PickActionClient:
    def __init__(
        self, pick_action_name: str, simple_move_action_name: str, wait_sec: float = 5.0
    ):
        self.pick_client = actionlib.SimpleActionClient(
            pick_action_name, PickTaskAction
        )
        self.pick_available = self.pick_client.wait_for_server(rospy.Duration(wait_sec))

        self.simple_move_client = actionlib.SimpleActionClient(
            simple_move_action_name, SimpleMoveArmAction
        )
        self.simple_move_available = self.simple_move_client.wait_for_server(
            rospy.Duration(wait_sec)
        )

        self.last_request_type = None
        self.feedback_cb = None
        self.done_cb = None

    def set_callbacks(self, feedback_cb=None, done_cb=None):
        self.feedback_cb = feedback_cb
        self.done_cb = done_cb

    def upsert_task(
        self,
        group_name: str,
        task_id: int,
        description: str,
        target_xyz: Tuple[float, float, float],
        target_frame_id: str,
        task_type: int,
        use_eef: bool,
        retry_times: int,
        go_safe_after_cancel: bool,
        use_place_pose: bool,
        place_config: dict,
        group_config: dict,
    ) -> None:
        if not self.pick_available:
            raise RuntimeError("/pick_action 不可用")

        goal = PickTaskGoal()
        goal.request_type = PickTaskGoal.UPSERT_TASK
        goal.group_name = group_name
        goal.id = int(task_id)
        goal.task_type = int(task_type)
        goal.description = description
        goal.target_type = PickTaskGoal.TARGET_POINT

        goal.target_point.x = float(target_xyz[0])
        goal.target_point.y = float(target_xyz[1])
        goal.target_point.z = float(target_xyz[2])
        goal.target_frame_id = target_frame_id

        goal.use_eef = bool(use_eef)
        goal.retry_times = max(0, min(255, int(retry_times)))
        goal.go_safe_after_cancel = bool(go_safe_after_cancel)

        goal.group_sort_type = int(
            group_config.get("group_sort_type", PickTaskGoal.GROUP_SORT_ID)
        )
        goal.group_dist_weight_orient = float(group_config.get("weight_orient", 0.30))
        goal.group_go_home_after_finish = bool(
            group_config.get("go_home_after_finish", True)
        )

        if use_place_pose:
            goal.use_place_pose = True
            goal.place_frame_id = place_config.get("frame_id", "base_link")

            if place_config.get("target_type", "point") == "pose":
                goal.place_target_type = PickTaskGoal.PLACE_TARGET_POSE
                goal.place_pose.position.x = float(place_config.get("x", 0.0))
                goal.place_pose.position.y = float(place_config.get("y", 0.0))
                goal.place_pose.position.z = float(place_config.get("z", 0.0))
                qx, qy, qz, qw = quaternion_from_euler(
                    float(place_config.get("roll", 0.0)),
                    float(place_config.get("pitch", 0.0)),
                    float(place_config.get("yaw", 0.0)),
                )
                goal.place_pose.orientation.x = qx
                goal.place_pose.orientation.y = qy
                goal.place_pose.orientation.z = qz
                goal.place_pose.orientation.w = qw
            else:
                goal.place_target_type = PickTaskGoal.PLACE_TARGET_POINT
                goal.place_point.x = float(place_config.get("x", 0.0))
                goal.place_point.y = float(place_config.get("y", 0.0))
                goal.place_point.z = float(place_config.get("z", 0.0))
        else:
            goal.use_place_pose = False
            goal.place_target_type = PickTaskGoal.PLACE_TARGET_NONE

        self.last_request_type = goal.request_type
        self.pick_client.send_goal(
            goal,
            feedback_cb=self.feedback_cb,
            done_cb=self.done_cb,
        )

    def execute_group(
        self,
        group_name: str,
        use_eef: bool,
        retry_times: int,
        go_safe_after_cancel: bool,
        group_config: dict,
    ) -> None:
        if not self.pick_available:
            raise RuntimeError("/pick_action 不可用")

        goal = PickTaskGoal()
        goal.request_type = PickTaskGoal.EXECUTE_TASK_GROUP
        goal.group_name = group_name
        goal.group_sort_type = int(
            group_config.get("group_sort_type", PickTaskGoal.GROUP_SORT_ID)
        )
        goal.group_dist_weight_orient = float(group_config.get("weight_orient", 0.30))
        goal.group_go_home_after_finish = bool(
            group_config.get("go_home_after_finish", True)
        )
        goal.use_eef = bool(use_eef)
        goal.retry_times = max(0, min(255, int(retry_times)))
        goal.go_safe_after_cancel = bool(go_safe_after_cancel)

        self.last_request_type = goal.request_type
        self.pick_client.send_goal(
            goal,
            feedback_cb=self.feedback_cb,
            done_cb=self.done_cb,
        )

    def cancel(self) -> None:
        if self.pick_available:
            self.pick_client.cancel_all_goals()

    def go_home(self, done_cb=None) -> None:
        if not self.simple_move_available:
            raise RuntimeError("/simple_move_arm 不可用")

        goal = SimpleMoveArmGoal()
        goal.command_type = SimpleMoveArmGoal.MOVE_TO_ZERO
        goal.target_type = SimpleMoveArmGoal.TARGET_POSE
        goal.x = [0.0]
        goal.y = [0.0]
        goal.z = [0.0]
        goal.roll = [0.0]
        goal.pitch = [0.0]
        goal.yaw = [0.0]

        self.simple_move_client.send_goal(goal, done_cb=done_cb)

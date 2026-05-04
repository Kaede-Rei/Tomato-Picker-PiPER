from typing import Tuple
from geometry_msgs.msg import PointStamped

import rospy
import tf2_ros
import tf2_geometry_msgs


class TfTools:
    def __init__(self, wait_sec: float = 0.2):
        self.wait_sec = wait_sec
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

    def transform_point(
        self, source_frame: str, target_frame: str, xyz: Tuple[float, float, float]
    ) -> Tuple[float, float, float]:
        if not source_frame or not target_frame:
            raise ValueError("Source frame and target frame must be specified.")

        point_stamped = PointStamped()
        point_stamped.header.frame_id = source_frame
        point_stamped.header.stamp = rospy.Time(0)
        point_stamped.point.x = xyz[0]
        point_stamped.point.y = xyz[1]
        point_stamped.point.z = xyz[2]

        out = self.buffer.transform(
            point_stamped, target_frame, rospy.Duration(self.wait_sec)
        )

        return out.point.x, out.point.y, out.point.z

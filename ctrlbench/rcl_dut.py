"""rclpy node under test: same behaviour as cpp/rcl_dut.cpp."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class Dut(Node):
    def __init__(self):
        super().__init__("ctrlbench_dut_py")
        self.pub = self.create_publisher(Header, "/ctrl_check", qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self.cb, qos_profile_sensor_data)

    def cb(self, m):
        ok = len(m.position) == 6 and all(math.isfinite(p) and abs(p) < 6.3 for p in m.position)
        out = Header()
        out.stamp = m.header.stamp
        out.frame_id = "ok" if ok else "bad"
        self.pub.publish(out)


def main():
    rclpy.init()
    node = Dut()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


class GripperPublisher(Node):
    def __init__(self):
        super().__init__("listener")

        self.max_value = 1.26
        self.min_value = -0.53
        self.alpha = 1.0 / (self.max_value - self.min_value)

        self.timer = self.create_timer(1.0 / 100.0, self.on_timer)
        self.joint_sub = self.create_subscription(
            JointState, "so_100_arm/joint_states", self._callback_joint_states, 10
        )
        self.gripper_pub = self.create_publisher(
            JointState, "so_100_arm/gripper_value", 10
        )
        self.gripper_position = None

    def on_timer(self):
        if self.gripper_position is not None:
            gripper_value = JointState()
            gripper_value.header.stamp = self.get_clock().now().to_msg()
            gripper_value.header.frame_id = "base_link"
            gripper_value.name = ["gripper_joint"]
            gripper_value.position = [
                self.alpha * self.gripper_position - self.alpha * self.min_value
            ]
            gripper_value.velocity = [0.0]
            gripper_value.effort = [0.0]
            self.gripper_pub.publish(gripper_value)
        pass

    def _callback_joint_states(self, msg: JointState):
        if len(msg.position) < 6:
            self.get_logger().warn("Not enough joint states received.")
            return

        # Assuming the gripper is controlled by the last joint
        self.gripper_position = msg.position[-1]


def main():
    try:
        rclpy.init()
        node = GripperPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

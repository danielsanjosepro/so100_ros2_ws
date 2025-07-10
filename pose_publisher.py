import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class PosePublisher(Node):
    def __init__(self):
        super().__init__("listener")

        # Declare and acquire `target_frame` parameter
        self.target_frame = (
            self.declare_parameter("target_frame", "Fixed_Gripper")
            .get_parameter_value()
            .string_value
        )

        self.max_value = 1.26
        self.min_value = 0.52
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0 / 100.0, self.on_timer)

        self.pose_pub = self.create_publisher(
            PoseStamped, "so_100_arm/current_pose", 10
        )

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = "Base"

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel, from_frame_rel, rclpy.time.Time()
            )

            self.get_logger().debug(
                f"Transform from {to_frame_rel} to {from_frame_rel}:\n"
                f"Translation: ({t.transform.translation.x}, "
                f"{t.transform.translation.y}, {t.transform.translation.z})\n"
                f"Rotation: ({t.transform.rotation.x}, "
                f"{t.transform.rotation.y}, {t.transform.rotation.z}, "
                f"{t.transform.rotation.w})"
            )

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = to_frame_rel
            pose_msg.pose.position.x = t.transform.translation.x
            pose_msg.pose.position.y = t.transform.translation.y
            pose_msg.pose.position.z = t.transform.translation.z
            pose_msg.pose.orientation.x = t.transform.rotation.x
            pose_msg.pose.orientation.y = t.transform.rotation.y
            pose_msg.pose.orientation.z = t.transform.rotation.z
            pose_msg.pose.orientation.w = t.transform.rotation.w

            self.pose_pub.publish(pose_msg)

        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {to_frame_rel} to {from_frame_rel}: {ex}"
            )
            return


def main():
    try:
        rclpy.init()
        node = PosePublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

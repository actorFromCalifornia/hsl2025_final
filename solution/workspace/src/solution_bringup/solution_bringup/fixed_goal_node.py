import math
import random

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf_transformations import quaternion_from_euler


class FixedGoalSetter(Node):
    """Sends a fixed navigation goal with optional retry offsets."""

    def __init__(self) -> None:
        super().__init__('fixed_goal_setter')

        # Declare parameters with default values
        self._declare_if_needed('use_sim_time', False)
        self._declare_if_needed('goal_x', 1.0)
        self._declare_if_needed('goal_y', 2.0)
        self._declare_if_needed('goal_yaw', 0.0)
        self._declare_if_needed('max_attempts', 3)
        self._declare_if_needed('retry_radius', 0.1)

        # Cache parameter values
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_yaw = self.get_parameter('goal_yaw').value
        self.max_attempts = self.get_parameter('max_attempts').value
        self.retry_radius = self.get_parameter('retry_radius').value

        self.get_logger().info(
            f'Задана цель: x={self.goal_x}, y={self.goal_y}, yaw={self.goal_yaw}'
        )

        # Action client for Nav2 navigation
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.attempt_count = 0
        self.current_goal_x = self.goal_x
        self.current_goal_y = self.goal_y
        self.current_goal_yaw = self.goal_yaw
        self.retry_timer = None

        # Delay the first goal to give Nav2 time to come up
        self.initial_timer = self.create_timer(10.0, self.initial_goal_callback)

    def _declare_if_needed(self, name: str, default_value) -> None:
        """Declare a parameter only if it was not provided via overrides."""
        if not self.has_parameter(name):
            self.declare_parameter(name, default_value)

    def initial_goal_callback(self) -> None:
        """Send the first goal after initialization."""
        self.initial_timer.cancel()
        self.send_goal(self.current_goal_x, self.current_goal_y, self.current_goal_yaw)

    def generate_random_offset(self) -> tuple[float, float, float]:
        """Generate a random offset inside the retry radius."""
        angle = random.uniform(0, 2 * math.pi)
        distance = random.uniform(0, self.retry_radius)

        offset_x = distance * math.cos(angle)
        offset_y = distance * math.sin(angle)
        offset_yaw = random.uniform(-0.17, 0.17)

        return offset_x, offset_y, offset_yaw

    def send_goal(self, x: float, y: float, yaw: float) -> None:
        """Send a navigation goal via the Nav2 action server."""
        if self.attempt_count >= self.max_attempts:
            self.get_logger().error(
                f'Достигнуто максимальное количество попыток ({self.max_attempts}). '
                'Прекращаю попытки.'
            )
            return

        self.attempt_count += 1

        quaternion = quaternion_from_euler(0.0, 0.0, yaw)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = quaternion[0]
        goal_msg.pose.pose.orientation.y = quaternion[1]
        goal_msg.pose.pose.orientation.z = quaternion[2]
        goal_msg.pose.pose.orientation.w = quaternion[3]

        self.get_logger().info(
            f'Попытка {self.attempt_count}: отправляю цель x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
        )

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future) -> None:
        """Handle the goal being accepted or rejected."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Цель отклонена навигатором')
            self.retry_with_offset()
            return

        self.get_logger().info('Цель принята, робот начал движение')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future) -> None:
        """Process the result provided by Nav2."""
        result_future = future.result()
        status = result_future.status

        if status == 4:
            self.get_logger().info('✅ Цель успешно достигнута!')
        else:
            self.get_logger().warn(f'❌ Не удалось достичь цели. Статус: {status}')

        self.retry_with_offset()

    def feedback_callback(self, feedback_msg) -> None:
        """Optional hook for logging navigation feedback."""
        # feedback = feedback_msg.feedback
        # self.get_logger().debug(f'Distance remaining: {feedback.distance_remaining:.2f} м')

    def delayed_retry_callback(self) -> None:
        """Trigger sending the goal again after a delay."""
        if self.retry_timer:
            self.retry_timer.cancel()
            self.retry_timer = None
        self.send_goal(self.current_goal_x, self.current_goal_y, self.current_goal_yaw)

    def retry_with_offset(self) -> None:
        """Retry navigation with a random offset if attempts remain."""
        if self.attempt_count < self.max_attempts:
            offset_x, offset_y, offset_yaw = self.generate_random_offset()

            self.current_goal_x = self.goal_x + offset_x
            self.current_goal_y = self.goal_y + offset_y
            self.current_goal_yaw = self.goal_yaw + offset_yaw

            self.get_logger().info(
                f'Случайное смещение: Δx={offset_x:.2f}, Δy={offset_y:.2f}, Δyaw={offset_yaw:.2f}'
            )

            if self.retry_timer:
                self.retry_timer.cancel()
            self.retry_timer = self.create_timer(1.0, self.delayed_retry_callback)
        else:
            self.get_logger().error('❌ Все попытки достичь цель исчерпаны')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FixedGoalSetter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

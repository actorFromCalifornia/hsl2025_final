import rclpy
import math
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import StaticTransformBroadcaster

TIME_STEP_MS = 32
MAX_VEL = 12.0
WHEEL_BASE = 0.25
WHEEL_RADIUS = 0.035

MAX_LINEAR = 0.55  # м/с
MAX_ANGULAR = 6.0  # рад/с
MAX_LINEAR_ACC = 3.3  # м/с²
MAX_ANGULAR_ACC = 30.0  # рад/с²

class WebotsDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot

        # Моторы
        self.LF = self._pick_motor("left_front_motor")
        self.RF = self._pick_motor("right_front_motor")
        self.LR = self._pick_motor("left_rear_motor")
        self.RR = self._pick_motor("right_rear_motor")

        self.current_v = 0
        self.current_w = 0

        self.lidar = self._setup_lidar()

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('webots_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

        self.tf_broadcaster = StaticTransformBroadcaster(self.__node)
        self.publish_static_transforms()

    # ================================================================
    # Устройства
    # ================================================================
    def _pick_motor(self, name: str):
        try:
            dev = self.robot.getDevice(name)
            dev.setPosition(float("inf"))
            dev.setVelocity(0.0)
            print(f"[webots_driver] picked motor '{name}'")
            return dev
        except Exception:
            return None

    def _setup_lidar(self):
        try:
            lidar = self.robot.getDevice("lidar")
            lidar.enable(TIME_STEP_MS)
            try:
                lidar.enablePointCloud(True)
            except Exception:
                pass
            print(f"[webots_driver] lidar enabled: {lidar.getHorizontalResolution()} beams, fov={lidar.getFov():.3f}")
            return lidar
        except Exception:
            return None
        
    # ================================================================
    # Обработка сообщений ROS2
    # ================================================================
    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        # Ограничение диапазона
        v = max(-MAX_LINEAR, min(MAX_LINEAR, forward_speed))
        w = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular_speed))

        # Ограничение ускорений
        dv = v - self.current_v
        dw = w - self.current_w
        max_dv = MAX_LINEAR_ACC * (TIME_STEP_MS / 1000.0)
        max_dw = MAX_ANGULAR_ACC * (TIME_STEP_MS / 1000.0)
        if abs(dv) > max_dv:
            v = self.current_v + max_dv * (1 if dv > 0 else -1)
        if abs(dw) > max_dw:
            w = self.current_w + max_dw * (1 if dw > 0 else -1)

        self.current_v, self.current_w = v, w

        # Пересчёт в скорости колёс
        v_l = v - (w * WHEEL_BASE / 2)
        v_r = v + (w * WHEEL_BASE / 2)
        k = MAX_VEL / max(1.0, abs(v_l), abs(v_r))
        vl, vr = v_l * k, v_r * k
        for m in [self.LF, self.LR]:
            if m: m.setVelocity(vl)
        for m in [self.RF, self.RR]:
            if m: m.setVelocity(vr)

    # ================================================================
    # Задание статических трансформаций
    # ================================================================
    def publish_static_transforms(self):
        transform = TransformStamped()
        transform.header.stamp = self.__node.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'lidar'
        
        # # Позиция из PROTO: translation 0.0 0.12 0
        # transform.transform.translation.x = 0.0
        # transform.transform.translation.y = 0.12  # 12 см вверх от центра
        # transform.transform.translation.z = 0.0
        
        # Вращение из PROTO: 
        # rotation 1 0 0 -1.5708 (-90° вокруг X)
        # rotation 0 1 0 3.141592653589793 (180° вокруг Y)
        
        # Комбинированное вращение: -90° X + 180° Y
        # Кватернион для такого вращения:
        # angle = math.pi  # 180 градусов
        # axis_x = 1.0
        # axis_y = 0.0
        # axis_z = 0.0
        
        # transform.transform.rotation.w = math.cos(angle/2)
        # transform.transform.rotation.x = axis_x * math.sin(angle/2)
        # transform.transform.rotation.y = axis_y * math.sin(angle/2) 
        # transform.transform.rotation.z = axis_z * math.sin(angle/2)
        
        self.tf_broadcaster.sendTransform(transform)
        self.__node.get_logger().info('Published static transform from base_link to lidar')

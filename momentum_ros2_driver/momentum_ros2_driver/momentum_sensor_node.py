import momentum_sdk as momentum
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu, NavSatFix, FluidPressure, Temperature, NavSatStatus
from math import radians, cos, sin
import rclpy


class MomentumSensor(Node):
    def __init__(self):
        super().__init__("momentum_sensor_node")

        # CAN interface parameter
        self.declare_parameter("CAN", "can0")

        # Getting CAN
        self.can = self.get_parameter("CAN").get_parameter_value().string_value
        self.get_logger().info(f"Using CAN interface: {self.can}")

        self.config = momentum.MomentumConfig()
        self.config.can_interface = self.can
        self.config.enable_callbacks = False  # No callbacks default

        # Creating instance
        self.momentum = momentum.Momentum(self.config)

        # Connection status
        self.connected = False

        self.imu_pub = self.create_publisher(Imu, "imu/data", 10)
        self.gps_pub = self.create_publisher(NavSatFix, "gps/fix", 10)
        self.vel_pub = self.create_publisher(TwistStamped, "gps/velocity", 10)
        self.pressure_pub = self.create_publisher(FluidPressure, "baro/pressure", 10)
        self.temp_pub = self.create_publisher(Temperature, "baro/temperature", 10)

        # Cached data
        self.last_imu = None
        self.last_gps = None
        self.last_vel = None
        self.last_baro = None

        self.connect()

        # timer to publish at 100 Hz
        self.timer = self.create_timer(0.01, self.publish_momentum_data)

    def connect(self):
        self.get_logger().info(f"Connnecting to Momentum board on {self.can}...")

        try:
            if self.momentum.connect():
                self.connected = True
                self.get_logger().info("Successfully Connected!")
                return True
            else:
                self.connected = False
                self.get_logger().info("Error connecting to Momentum Board!")
                return False
        except Exception as e:
            self.connected = False
            self.get_logger().info(f"Exception: {str(e)}")
            return False

    def publish_momentum_data(self):
        now = self.get_clock().now().to_msg()

        # update and cache lastest momentum data
        self.last_imu = self.momentum.get_imu_data()
        self.last_gps = self.momentum.get_gps_position()
        self.last_vel = self.momentum.get_gps_velocity()
        self.last_baro = self.momentum.get_barometric_data()

        # IMU Data
        if self.last_imu and self.last_imu.isValid:
            imu_msg = Imu()
            imu_msg.header.stamp = now
            imu_msg.header.frame_id = "imu_link"

            imu_msg.orientation.w = self.last_imu.quat_w
            imu_msg.orientation.x = self.last_imu.quat_x
            imu_msg.orientation.y = self.last_imu.quat_y
            imu_msg.orientation.z = self.last_imu.quat_z

            # Radians conversion is needed since ROS2 works with them and
            # gyro data is given as deg/s
            imu_msg.angular_velocity.x = radians(self.last_imu.gyro_x)
            imu_msg.angular_velocity.y = radians(self.last_imu.gyro_y)
            imu_msg.angular_velocity.z = radians(self.last_imu.gyro_z)

            # IMU linear_acceleration in ROS2 requires raw accel data, not
            # gravity-compensated, incoming from Momentum is compensated.
            imu_msg.linear_acceleration._x = self.last_imu.accel_x
            imu_msg.linear_acceleration._y = self.last_imu.accel_y
            imu_msg.linear_acceleration._z = self.last_imu.accel_z

            self.imu_pub.publish(imu_msg)

        # GPS Position Data
        if self.last_gps and self.last_gps.isValid:
            gps_msg = NavSatFix()
            gps_msg.header.stamp = now
            gps_msg.header.frame_id = "gps_link"

            gps_msg.latitude = self.last_gps.latitude_deg
            gps_msg.longitude = self.last_gps.longitude_deg
            gps_msg.altitude = self.last_gps.altitude_m
            gps_msg.status.status = NavSatStatus.STATUS_FIX
            gps_msg.status.service = NavSatStatus.SERVICE_GPS

            self.gps_pub.publish(gps_msg)

        # GPS Velocity Data
        if self.last_gps and self.last_gps.isValid:
            twist_msg = TwistStamped()
            twist_msg.header.stamp = now
            twist_msg.header.frame_id = "gps_link"

            speed = self.last_vel.speed_ms
            course_rad = radians(self.last_vel.course_deg)

            twist_msg.twist.linear.x = speed * cos(course_rad)
            twist_msg.twist.linear.y = speed * sin(course_rad)

            self.vel_pub.publish(twist_msg)

        # Barometric Data
        if self.last_baro and self.last_baro.isValid:
            pressure_msg = FluidPressure()
            pressure_msg.header.stamp = now
            pressure_msg.header.frame_id = "baro_link"

            pressure_msg.fluid_pressure = self.last_baro.pressure_pa

            self.pressure_pub.publish(pressure_msg)

            temp_msg = Temperature()
            temp_msg.header.stamp = now
            temp_msg.header.frame_id = "baro_link"

            temp_msg.temperature = self.last_baro.temp_c

            self.temp_pub.publish(temp_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MomentumSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

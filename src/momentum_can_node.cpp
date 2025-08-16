#include "scalpelspace_momentum_ros/momentum_can_node.hpp"
#include "scalpelspace_momentum_ros/momentum_driver/momentum_driver.h"
#include <chrono>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace scalpelspace_momentum_ros {

  MomentumCanNode::MomentumCanNode(const rclcpp::NodeOptions &options)
      : Node("momentum_can_node", options) {
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
      RCLCPP_FATAL(get_logger(), "Failed to open CAN socket");
      rclcpp::shutdown();
      return;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_FATAL(get_logger(), "ioctl failed");
      rclcpp::shutdown();
      return;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_, reinterpret_cast<struct sockaddr *>(&addr),
             sizeof(addr)) < 0) {
      RCLCPP_FATAL(get_logger(), "Bind failed");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(get_logger(), "Momentum is succesfully connected to CAN interface: %s", ifr.ifr_name);
    RCLCPP_INFO(get_logger(), "Run 'ros2 topic list' to check data'");

    momentum_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("momentum/imu_data", 10);
    momentum_gps_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("momentum/gps_fix", 10);
    momentum_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("momentum/gps_velocity", 10);
    momentum_pressure_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>("momentum/pressure", 10);
    momentum_temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>("momentum/temperature", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(10),
                               std::bind(&MomentumCanNode::poll_can, this));
  }

  MomentumCanNode::~MomentumCanNode() {
    if (can_socket_ >= 0) {
      close(can_socket_);
    }
  }

  void MomentumCanNode::poll_can() {
    struct can_frame frame;
    ssize_t nbytes = read(can_socket_, &frame, sizeof(frame));

    if (nbytes <= 0) {
      return;
    }

    static std::unordered_map<std::string, float> signals;

    // Decode the physical value.
    for (int mi = 0; mi < dbc_message_count; ++mi) {
      const can_message_t &dbc = dbc_messages[mi];
      if ((uint32_t)frame.can_id == dbc.message_id) {
        for (uint8_t si = 0; si < dbc.signal_count; ++si) {
          const can_signal_t &sig = dbc.signals[si];
          float phys = decode_signal(&sig, frame.data);
          signals[sig.name] = phys;
        }
        break;
      }
    }

    auto now = this->get_clock()->now();

    if (signals.count("quaternion_x") && signals.count("quaternion_y") && signals.count("quaternion_z") 
        && signals.count("quaternion_w") && signals.count("gyro_x") && signals.count("gyro_y") && signals.count("gyro_z")
        && signals.count("lin_accel_x") && signals.count("lin_accel_y") && signals.count("lin_accel_z")) {
      sensor_msgs::msg::Imu imu;
      imu.header.stamp = now;
      imu.header.frame_id = "imu_link";
      imu.orientation.x = signals["quaternion_x"];
      imu.orientation.y = signals["quaternion_y"];
      imu.orientation.z = signals["quaternion_z"];
      imu.orientation.w = signals["quaternion_w"];
      imu.angular_velocity.x = signals["gyro_x"];
      imu.angular_velocity.y = signals["gyro_y"];
      imu.angular_velocity.z = signals["gyro_z"];
      imu.linear_acceleration.x = signals["lin_accel_x"];
      imu.linear_acceleration.y = signals["lin_accel_y"];
      imu.linear_acceleration.z = signals["lin_accel_z"];

      imu.orientation_covariance[0] = -1;
      imu.angular_velocity_covariance[0] = -1;
      imu.linear_acceleration_covariance[0] = -1;
      momentum_imu_pub_->publish(imu);
    } 

    if (signals.count("latitude") && signals.count("longitude") && signals.count("altitude")){
      sensor_msgs::msg::NavSatFix gps;
      gps.header.stamp = now;
      gps.header.frame_id = "gps_link";
      gps.latitude = signals["latitude"];
      gps.longitude = signals["longitude"];
      gps.altitude = signals["altitude"];
      gps.position_covariance[0] = -1;
      momentum_gps_pub_->publish(gps); 
      
      if (signals.count("speed") && signals.count("course")){
        geometry_msgs::msg::TwistStamped vel;
        vel.header = gps.header;
        float spd = signals["speed"];
        float crs = signals["course"] * M_PI / 180.0;
        vel.twist.linear.x = spd * cos(crs);
        vel.twist.linear.y = spd * cos(crs);
        momentum_vel_pub_->publish(vel);
      }
    }

    if (signals.count("pressure")){
        sensor_msgs::msg::FluidPressure prs;
        prs.header.stamp = now;
        prs.header.frame_id = "barometric_link";
        prs.fluid_pressure = signals["pressure"];
        momentum_pressure_pub_->publish(prs);
        
        if (signals.count("temperature")){
          sensor_msgs::msg::Temperature temp;
          temp.header = prs.header;
          temp.temperature = signals["temperature"];
          momentum_temp_pub_->publish(temp);
        }
    } 
  }

} // namespace scalpelspace_momentum_ros

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<scalpelspace_momentum_ros::MomentumCanNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

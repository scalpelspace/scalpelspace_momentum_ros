#ifndef SCALPELSPACE_MOMENTUM_ROS_MOMENTUM_CAN_NODE_H
#define SCALPELSPACE_MOMENTUM_ROS_MOMENTUM_CAN_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <unordered_map>
#include <cmath>


namespace scalpelspace_momentum_ros {

  class MomentumCanNode : public rclcpp::Node {
  public:
    explicit MomentumCanNode(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~MomentumCanNode() override;

  private:
    void poll_can();
    void publish_momentum_data();

    int can_socket_{-1};
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr momentum_imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr momentum_gps_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr momentum_vel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr momentum_pressure_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr momentum_temp_pub_;

  };

} // namespace scalpelspace_momentum_ros

#endif // SCALPELSPACE_MOMENTUM_ROS_MOMENTUM_CAN_NODE_H

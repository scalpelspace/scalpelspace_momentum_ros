#ifndef SCALPELSPACE_MOMENTUM_ROS_MOMENTUM_CAN_NODE_H
#define SCALPELSPACE_MOMENTUM_ROS_MOMENTUM_CAN_NODE_H

#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>

namespace scalpelspace_momentum_ros {

  class MomentumCanNode : public rclcpp::Node {
  public:
    explicit MomentumCanNode(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~MomentumCanNode() override;

  private:
    void poll_can();

    int can_socket_{-1};
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
  };

#endif // SCALPELSPACE_MOMENTUM_ROS_MOMENTUM_CAN_NODE_H

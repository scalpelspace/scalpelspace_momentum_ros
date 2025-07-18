#include "scalpelspace_momentum_ros/momentum_can_node.hpp"

#include <chrono>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

namespace scalpelspace_momentum_ros {

  MomentumCanNode::MomentumCanNode(const rclcpp::NodeOptions &options)
      : Node("momentum_can_node", options) {
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open CAN socket");
      rclcpp::shutdown();
      return;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_FATAL(this->get_logger(), "ioctl failed");
      rclcpp::shutdown();
      return;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_, reinterpret_cast<struct sockaddr *>(&addr),
             sizeof(addr)) < 0) {
      RCLCPP_FATAL(this->get_logger(), "Bind failed");
      rclcpp::shutdown();
      return;
    }

    pub_ = this->create_publisher<can_msgs::msg::Frame>("received_frames", 10);
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(10),
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
    if (nbytes > 0) {
      can_msgs::msg::Frame msg;
      msg.id = frame.can_id;
      msg.dlc = frame.can_dlc;
      msg.is_extended = (frame.can_id & CAN_EFF_FLAG);
      msg.is_rtr = (frame.can_id & CAN_RTR_FLAG);
      msg.data = std::vector<uint8_t>(frame.data, frame.data + frame.can_dlc);
      pub_->publish(msg);
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

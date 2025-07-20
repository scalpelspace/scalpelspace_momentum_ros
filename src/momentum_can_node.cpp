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

    pub_ = create_publisher<can_msgs::msg::Frame>("received_frames", 10);
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

    // Publish ROS message of raw CAN message.
    can_msgs::msg::Frame msg;
    msg.id = frame.can_id;
    msg.dlc = frame.can_dlc;
    msg.is_extended = (frame.can_id & CAN_EFF_FLAG);
    msg.is_rtr = (frame.can_id & CAN_RTR_FLAG);
    for (size_t i = 0; i < frame.can_dlc && i < msg.data.size(); ++i) {
      msg.data[i] = frame.data[i];
    }
    pub_->publish(msg);

    // TODO: Delete dev, print.
    std::ostringstream oss;
    oss << "CAN ID=0x" << std::hex << std::uppercase << frame.can_id << std::dec
        << " DLC=" << static_cast<int>(frame.can_dlc) << " RAW=[";
    for (size_t i = 0; i < frame.can_dlc; ++i) {
      if (i) {
        oss << ' ';
      }
      oss << "0x" << std::setw(2) << std::setfill('0') << std::hex
          << std::uppercase << static_cast<int>(frame.data[i]);
    }
    oss << std::dec << "]";

    // Decode the physical value.
    for (int mi = 0; mi < dbc_message_count; ++mi) {
      const can_message_t &dbc = dbc_messages[mi];
      if ((uint32_t)frame.can_id == dbc.message_id) {
        for (uint8_t si = 0; si < dbc.signal_count; ++si) {
          const can_signal_t &sig = dbc.signals[si];
          float phys = decode_signal(&sig, frame.data);
          oss << " | " << sig.name << "=" << phys; // TODO: Delete dev, print.
        }
        break;
      }
    }

    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  }

} // namespace scalpelspace_momentum_ros

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<scalpelspace_momentum_ros::MomentumCanNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

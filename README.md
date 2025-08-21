# scalpelspace_momentum_ros

![ros2_humble_build](https://github.com/scalpelspace/scalpelspace_momentum_ros/actions/workflows/ros2_humble_build.yaml/badge.svg)

ROS2 package for CAN communication with the ScalpelSpace Momentum dev board.

---

<details markdown="1">
  <summary>Table of Contents</summary>

<!-- TOC -->
* [scalpelspace_momentum_ros](#scalpelspace_momentum_ros)
  * [1 Overview](#1-overview)
<!-- TOC -->

</details>

---

## 1 Overview
> [!IMPORTANT]
> CAN communication is expected to be set up via SocketCAN drivers.

Utilizes a local copy of [
`momentum_driver`](https://github.com/scalpelspace/momentum_driver) for low
level CAN operations (local copy only includes the C code files):

1. [src/momentum_driver](src/momentum_driver).
2. [include/scalpelspace_momentum_ros/momentum_driver](include/scalpelspace_momentum_ros/momentum_driver).

- Git submodule is not utilized for reduced setup complexity for users.
- Copies modified on `#include` filepaths to comply with standard ROS2 package
  structures.
## Cloning the repo and running the momentum board node
1. ```bash
   cd your_worskpace/src
   ```
3. ```bash
   git clone https://github.com/scalpelspace/scalpelspace_momentum_ros.git
   ```
4. ```bash
   cd ~/your_workspace
   ```
5. ```bash
   rosdep install --from-paths src -y --ignore-src
   ```
6. ```bash
   colcon build
   ```
7. ```bash
   source install/setup.bash
   ```
8. ```bash
   ros2 run scalpelspace_momentum_ros momentum_can_node
   ```
## Subscriber example for IMU data
> [!NOTE]  
> The momentum board publishes 5 topics.
> momentum/imu_data, momentum/gps_fix, momentum/gps_velocity, momentum/pressure, momentum/temperature

#### CPP
```cpp
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
using std::placeholders::_1;

class ImuSubs: public rclcpp::Node {
public:
  ImuSubs() : Node("momentum_imu_subscriber") {
    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "momentum/imu_data", 10,
      std::bind(&ImuSubs::callback, this, _1));
  }

private:
  void callback(const sensor_msgs::msg::Imu & msg) const {
    RCLCPP_INFO(this->get_logger(),
      "IMU | quat=[%.3f, %.3f, %.3f, %.3f] | gyro=[%.3f, %.3f, %.3f] | accel=[%.3f, %.3f, %.3f]",
      msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
      msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
      msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuSubs>());
  rclcpp::shutdown();
  return 0;
}
```
#### Python
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('momentum_imu_subscriber')
        self.subscription = self.create_subscription( Imu, 'momentum/imu_data',
            self.listener_callback, 10) self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(
            f'IMU | quat=[{msg.orientation.x:.3f}, {msg.orientation.y:.3f}, '
            f'{msg.orientation.z:.3f}, {msg.orientation.w:.3f}] | '
            f'gyro=[{msg.angular_velocity.x:.3f}, {msg.angular_velocity.y:.3f}, '
            f'{msg.angular_velocity.z:.3f}] | '
            f'accel=[{msg.linear_acceleration.x:.3f}, {msg.linear_acceleration.y:.3f}, '
            f'{msg.linear_acceleration.z:.3f}]')

def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = ImuSubscriber()
    rclpy.spin(imu_subscriber)
    imu_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
---

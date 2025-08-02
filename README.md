<div align="center">
  <img src="https://i.imgur.com/mrv4vQ6_d.png?maxwidth=520&shape=thumb&fidelity=high" alt="ScalpelSpace Logo" width="800">
</div>

------
ROS2 Packages for Momentum Board, it use Momentum SDK to publish data recieved from the board into ROS2 standard messages such as Imu, NavSatFix, FluidPressure, Temperature, NavSatStatus and TwistStamped.

> [!NOTE] 
> The documentation is in progress for this branch!

> [!IMPORTANT]  
> Following ROS2 standards, there was created a bringup and driver packages using python. 

> [!WARNING]
> If you want to use **Momentum SDK** outside ROS2 for C++ or Python projects please go to [Momentum SDK repo](https://github.com/scalpelspace/momentum_sdk.git). This will also generate all python bindings definitions.

#### Table of Contents

- [Install Dependecies](#installdependecies)
- [ROS2 Installation](#ROS2installation)
    - [How to install ROS2 Humble](#howtoinstallros2humble)
    - [Setting Workspace](#settingworskspace)
- [Subscribing to Momentum ROS2](#subscribinttomomentumros2)
-

> [!IMPORTANT]
> Run all this commands in your terminal

#### Install Dependecies

* CAN-utils installation:
```bash
sudo apt-get install -y can-utils iproute2 linux-modules-extra-$(uname -r)
```
> [!NOTE]
> Python dependecies are needed for the SDK even if you will only install it at ROS2 level and not globally

* Python dependecies:
```bash
pip3 install --user cantools pybind11 pybind11-stubgen
```
```bash
pip3 install --user --upgrade pip setuptools wheel
```

> [!NOTE]
> Only if you are using a Waveshare RS485 CAN HAT do the following steps

```bash
-sudo nano /boot/firmware/config.txt
```
```bash
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000
```
```bash
sudo reboot now
```

> [!NOTE]
> If you are using CAN-USB adapters or Washare RS485 CAN HAT this step is mandatory everytime you boot the system.

```bash
sudo ip link set can0 up type can bitrate 5000000
```

#### ROS2 Installation

> [!IMPORTANT]
> ROS2 Humble is the only distro tested with this packages. 

* **How to install ROS2 Humble**
please follow installation steps from the main documentation for [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

After installation run this command:
```bash
sudo apt install ros-humble-sensor-msgs ros-humble-geometry-msgs
```
* **Setting Workspace**
Now lets set a workspace if you dont have one yet
```bash
source /opt/ros/humble/setup.bash
```
> [!NOTE]
> Remember to add the name you want for **your workspace**
```bash 
mkdir -p ~/<your_workspace_name>/src 
```
```bash
cd ~/<your_workspace_name>/src
```
Now its time to add **Momentum ROS2 Driver**

```bash
git clone https://github.com/scalpelspace/scalpelspace_momentum_ros.git
```
>[!NOTE]
> It is important to navigate into the folder of scalpelspace_momentum_ros to activate some submodule with the following command:
```bash
cd scalpelspace_momentum_ros
```
```bash
git submodule update --init --recursive
```
```bash
cd ~/<your_workspace_name>
```
Make sure to run ROS2 dependecies, for that we need this commands:
```bash
sudo apt-get install python3-rosdep
```
```bash
sudo rosdep init
```
```bash
rosdep update
```
```bash
rosdep install --from-paths src -y --ignore-src
```

If everything is successfully install now we can build our workspace along **Momentum ROS2 Driver** 

>[!IMPORTANT]
> You must be in your root directory of your workspace to run the following command. **Do not build inside your src/ folder**

```bash
colcon build --symlink-install
```

If you get a 100% compilation, now is time to use **Momentum ROS2 Driver** by using the following commands:

```bash
source install/setup.bash
```
```bash
ros2 launch momentum_ros2_bringup momentum_bringup.launch.py
```
You should see the following **output** in your terminal:
```bash
[momentum_sensor_node-1] [INFO] [1754099669.161626526] [momentum_sensor_node]: Using CAN interface: can0
[momentum_sensor_node-1] [INFO] [1754099669.406316401] [momentum_sensor_node]: Connnecting to Momentum board on can0...
[momentum_sensor_node-1] [INFO] [1754099669.409608060] [momentum_sensor_node]: Successfully Connected!
```
Now if you run in a new terminal the following command, you should be able to see all the topics available to subscribe

```bash
ros2 topic list
```
**Output**
```bash
/baro/pressure
/baro/temperature
/gps/fix
/gps/velocity
/imu/data
```

if you run
```bash
ros2 topic echo /imu/data
```
You will be able to see the outcoming data from the IMU sensor for example
**Output**
```bash
header:
  stamp:
    sec: 1754100034
    nanosec: 294580234
  frame_id: imu_link
orientation:
  x: 0.002990454202517867
  y: -0.011963609606027603
  z: 0.6490365266799927
  w: 0.7606732249259949
orientation_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
angular_velocity:
  x: 0.0004261057744126781
  y: 0.0004261057744126781
  z: 0.0004261057744126781
angular_velocity_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
linear_acceleration:
  x: 0.20465275645256042
  y: -0.10662573575973511
  z: 9.815974235534668
linear_acceleration_covariance:
```
##### Subscribing to Momentum ROS2

This is an small example of how to susbcribe to Momentum ROS2 topics.
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

    def imu_callback(self, msg):
        self.get_logger().info('Received IMU accel: "%.2f m/s²"' % msg.linear_acceleration.x)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
> [!IMPORTANT]
> **Momentum ROS2 Driver** messages are the following types

## Momentum ROS2 messages type

| Momentum | ROS2 message type |
|----------|-------------------|
| /imu/data | sensor_msgs/msg/Imu |
| /gps/fix | sensor_msgs/msg/NavSatFix |
| /gps/velocity | geometry_msgs/msg/TwistStamped |
| /baro/pressure | sensor_msgs/msg/FluidPressure |
| /baro/temperature | sensor_msgs/msg/Temperature |

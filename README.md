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

CAN communication is expected to be setup via SocketCAN drivers.

Utilizes a local copy of [
`momentum_driver`](https://github.com/scalpelspace/momentum_driver) for low
level CAN operations (local copy only includes the C code files):

1. [src/momentum_driver](src/momentum_driver).
2. [include/scalpelspace_momentum_ros/momentum_driver](include/scalpelspace_momentum_ros/momentum_driver).

- Git submodule not utilized for reduced setup complexity for users.
- Copies modified on `#include` filepaths to comply with standard ROS2 package
  structures.

---

# Installation

RTEST is distributed as an open source project and can be installed from source or downloaded as a pre-built package.

## Binary Packages

Binaries are available for the Tier 1 operating systems listed in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers).

- [Ubuntu 24.04 (Noble Numbat)](https://github.com/Beam-and-Spyrosoft/rtest/releases/)

### Using apt (recommended)

```bash
$ # Add the repository
$ sudo apt update && sudo apt install -y software-properties-common
$ sudo add-apt-repository ppa:beam-and-spyrosoft/rtest
$ sudo apt update

$ # Install RTEST
$ sudo apt install ros-jazzy-rtest
```

## Install From Source

1. **Create a ROS 2 workspace (if you don't have one already)**:

```bash
$ mkdir -p ~/ros2_ws/src
```

2. **Clone the RTEST repository**:

```bash
$ cd ~/ros2_ws/src
$ git clone https://github.com/Beam-and-Spyrosoft/rtest.git
```

3. **Build the package**:

```bash
$ cd ~/ros2_ws
$ colcon build
```

4. **Verify installation**:

```bash
$ source ~/ros2_ws/install/setup.bash
$ colcon test --packages-select rtest_examples --event-handlers console_cohesion+
```

## Adding RTEST to Your Package

1. Add dependency in `package.xml`:
```xml
<test_depend>rtest</test_depend>
```

2. Configure CMake:
```cmake
find_package(rtest REQUIRED)

ament_add_gmock(my_node_test
  test/my_node_test.cpp
  src/my_node.cpp  # Include source directly
)
target_link_libraries(my_node_test rtest::rtest)
```

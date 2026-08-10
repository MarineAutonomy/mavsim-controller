# Controllers in other languages (C++, MATLAB, ...)

The bridge speaks plain ROS2, so any language with ROS2 bindings works. You only
need to:

1. Build the `interfaces` package (see [`../../interfaces/README.md`](../../interfaces/README.md))
   so you have `interfaces/msg/Actuator`.
2. Subscribe to `/<vessel>/vessel_state` (`std_msgs/Float64MultiArray`).
3. Publish `interfaces/Actuator` on `/<vessel>/actuator_cmd`.
4. Use the same `ROS_DOMAIN_ID` as the bridge.

## C++ (rclcpp) sketch

`CMakeLists.txt` dependencies:

```cmake
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)         # the package you built
ament_target_dependencies(my_controller rclcpp std_msgs interfaces)
```

Node skeleton:

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "interfaces/msg/actuator.hpp"

class Bridge : public rclcpp::Node {
public:
  Bridge() : Node("my_controller") {
    pub_ = create_publisher<interfaces::msg::Actuator>("/matsya_01/actuator_cmd", 10);
    sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/matsya_01/vessel_state", 10,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        // msg->data = [t,u,v,w,p,q,r,x,y,z,phi,theta,psi, <actuators...>]
        double x = msg->data[7], y = msg->data[8], yaw = msg->data[12];
        interfaces::msg::Actuator cmd;
        cmd.actuator_names  = {"cs_01", "th_01"};
        cmd.actuator_values = {compute_rudder(x, y, yaw), 1500.0};
        cmd.covariance      = {0.0, 0.0};
        pub_->publish(cmd);
      });
  }
private:
  rclcpp::Publisher<interfaces::msg::Actuator>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
};
```

## MATLAB (ROS Toolbox)

```matlab
setenv("ROS_DOMAIN_ID","0");            % must match the bridge
% Build/define the interfaces package once via ros2genmsg pointing at
% user_repo_new/interfaces, then:
node  = ros2node("/my_controller");
pub   = ros2publisher(node, "/matsya_01/actuator_cmd", "interfaces/Actuator");
sub   = ros2subscriber(node, "/matsya_01/vessel_state", "std_msgs/Float64MultiArray");

while true
    s = receive(sub, 10);
    x = s.data(8); y = s.data(9); yaw = s.data(13);   % MATLAB is 1-indexed
    msg = ros2message(pub);
    msg.actuator_names  = ["cs_01","th_01"];
    msg.actuator_values = [computeRudder(x,y,yaw), 1500.0];
    msg.covariance      = [0.0, 0.0];
    send(pub, msg);
end
```

## Reminders

- Field layout of `vessel_state.data`: `[t,u,v,w,p,q,r,x,y,z, attitude..., actuators...]`
  where attitude is 3 values (euler) or 4 (quaternion).
- Control surfaces `cs_<id>` in degrees, thrusters `th_<id>` in RPM.
- Always fill `actuator_names` (the bridge drops nameless commands).
- The bridge only exposes `actuator_cmd` for vessels YOU own; you cannot command
  vessels controlled from other machines.

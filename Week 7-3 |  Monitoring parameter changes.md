# Monitoring parameter changes

<br/>

>In this section, I will create a new package to contain some sample code, write some C++ code to use the ParameterEventHandler class, and test the resulting code.
<br/>

First of all, I will navigate to the ```ros2_ws/src```directory and create a new package called ```cpp_parameter_event_handler``` :
```
ros2 pkg create --build-type ament_cmake cpp_parameter_event_handler --dependencies rclcpp
```
After creating the package, I will update the ```package.xml``` file which is located inside of the newly created package:
```
<description>C++ parameter events client tutorial</description>
<maintainer email="ozodjonkunishev@email.com">jons</maintainer>
<license>Apache License 2.0</license>
```
Then, inside of the ```ros2_ws/src/cpp_parameter_event_handler/src``` directory, I will create a new file called ```parameter_event_handler.cpp``` and put the code below inside of the file:
```
gedit parameter_event_handler.cpp
```
```
#include <memory>

#include "rclcpp/rclcpp.hpp"

class SampleNodeWithParameters : public rclcpp::Node
{
public:
  SampleNodeWithParameters()
  : Node("node_with_parameters")
  {
    this->declare_parameter("an_int_param", 0);

    // Create a parameter subscriber that can be used to monitor parameter changes
    // (for this node's parameters as well as other nodes' parameters)
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    // Set a callback for this node's integer parameter, "an_int_param"
    auto cb = [this](const rclcpp::Parameter & p) {
        RCLCPP_INFO(
          this->get_logger(), "cb: Received an update to parameter \"%s\" of type %s: \"%ld\"",
          p.get_name().c_str(),
          p.get_type_name().c_str(),
          p.as_int());
      };
    cb_handle_ = param_subscriber_->add_parameter_callback("an_int_param", cb);
  }

private:
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SampleNodeWithParameters>());
  rclcpp::shutdown();

  return 0;
}
```

<br/>

Now, I have to add executables to the ```CMakeLists.txt``` file:
```
add_executable(parameter_event_handler src/parameter_event_handler.cpp)
ament_target_dependencies(parameter_event_handler rclcpp)

install(TARGETS
  parameter_event_handler
  DESTINATION lib/${PROJECT_NAME}
)
```
<br/>

After that, I will verify if I have missing dependencies or not:
```
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
```
Since I have no missing dependencies, I will go back to the root of my workspace and build my new package:
```
colcon build --packages-select cpp_parameter_event_handler
```
Then, I will open a new terminal, and source the setup files:
```
. install/setup.bash
```
Now, it is time to run the node:
```
ros2 run cpp_parameter_event_handler parameter_event_handler
```
To test if the node is active, I will open another terminal, source the setup files and run the command below:
```
ros2 param set node_with_parameters an_int_param 43
```
The output is:
![monitor param changes](https://user-images.githubusercontent.com/90167023/196704101-814c0e37-10b3-422c-b95a-19efe9c4eda3.png)
<br/>

The next step is to monitor changes to another node's parameters. To do this, I will navigate to the directory where I created the ```parameter_event_handler.cpp``` file and edit it by adding the following code to the constructor of the class:
```
// Now, add a callback to monitor any changes to the remote node's parameter. In this
// case, we supply the remote node name.
auto cb2 = [this](const rclcpp::Parameter & p) {
    RCLCPP_INFO(
      this->get_logger(), "cb2: Received an update to parameter \"%s\" of type: %s: \"%.02lf\"",
      p.get_name().c_str(),
      p.get_type_name().c_str(),
      p.as_double());
  };
auto remote_node_name = std::string("parameter_blackboard");
auto remote_param_name = std::string("a_double_param");
cb_handle2_ = param_subscriber_->add_parameter_callback(remote_param_name, cb2, remote_node_name);
```
Then, I will also add another member variable ```cb_handle2``` for additional callback handle:
```
private:
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle2_;  // Add this
};
```
Now, I will go back to the root of my workspace, build my updated package and source the setup files:
```
colcon build --packages-select cpp_parameter_event_handler
```
```
. install/setup.bash
```
To test monitoring of the remote parameters, I will run the newly-built 'parameter-event-handler` code:
```
ros2 run cpp_parameter_event_handler parameter_event_handler
```
Then, I will open another terminal and run the parameter_blackboard demo application:
```
ros2 run demo_nodes_cpp parameter_blackboard
```
Finally, from a third terminal, I will set a parameter on the parameter_blackboard node:
```
ros2 param set parameter_blackboard a_double_param 3.45
```
As an output in the parameter_event_handler window, I can see that the callback function was invoked upon the parameter update:
![monitor param changes3](https://user-images.githubusercontent.com/90167023/196706665-35862679-1b1b-4d82-a40b-1b3b182adb19.png)


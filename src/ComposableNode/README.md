# Composable Nodes:
Multiple nodes share the same process and memory space by being loaded into a container. This avoids the overhead of IPC by using shared memory, improving performance.

Follow this link:
https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-a-Composable-Node.html
---
# Class Definition
The only change to your class definition that you may have to do is ensure that the constructor for the class takes a NodeOptions argument.
```
VincentDriver(const rclcpp::NodeOptions & options) : Node("vincent_driver", options)
{
  // ...
}
```

# No More Main Method
Replace your main method with a pluginlib-style macro invocation.
```
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(palomino::VincentDriver)
```

# CMake Changes
First, add rclcpp_components as a dependency in your CMakeLists.txt with:
```
find_package(rclcpp_components REQUIRED)
```
Second, we’re going to replace our add_executable with a add_library with a new target name.
```
add_library(vincent_driver_component src/vincent_driver.cpp)
```
Third, replace other build commands that used the old target to act on the new target. i.e. ```ament_target_dependencies(vincent_driver ...)``` becomes ```ament_target_dependencies(vincent_driver_component ...)```

Fourth, add a new command to declare your component.
```
rclcpp_components_register_node(
    vincent_driver_component
    PLUGIN "palomino::VincentDriver"
    EXECUTABLE vincent_driver
)
```
Fifth and finally, change any installation commands in the CMake that operated on the old target to install the library version instead. For instance, do not install either target into ```lib/${PROJECT_NAME}```. Replace with the library installation.
```
ament_export_targets(export_vincent_driver_component)
install(TARGETS vincent_driver_component
        EXPORT export_vincent_driver_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
```
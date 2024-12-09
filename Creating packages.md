### C++ package
```zsh
ros2 pkg create --build-type ament_cmake <package_name>
```

```zsh
ros2 pkg create --build-type ament_cmake --node-name my_node my_package
```
### Python package
```zsh
ros2 pkg create --build-type ament_python <package_name>
```

```zsh
ros2 pkg create --build-type ament_python --node-name my_node my_package
```

## Building packages
```zsh
colcon build
```
___
`--symlink-install` is used to create shared directory for python scripts and not rebuild many times
_______
`--packages-select my_package` is used to build only selected packages as example -- **my package**

#### REMEMBER TO SOURCE WORK-SPACES AFTER BUILDING
```zsh
source install/setup.sh
```

## **Adding [[Nodes|nodes]] & types of [[Nodes#header|messages]] to packages** C++
### add the following lines to CMakeLists.txt
```cmake
add_executable($NODE_NAME$ src/$PACKAGE_NAME$/main.cpp src/$PACKAGE_NAME$/$INHERITATE$.cpp  ...)

ament_target_dependencies(task1_spiral rclcpp geometry_msgs)
#geometry_msgs - is a message
  

install(TARGETS
custome_node_1
custome_node_2

DESTINATION lib/${PROJECT_NAME})
```




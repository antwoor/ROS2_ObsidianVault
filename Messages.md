## There are some standard messages in ROS. Here are its definitions and structure
# [geometry_msgs](https://docs.ros2.org/galactic/api/geometry_msgs/index-msg.html)/msg/Twist Message

### C++ declaration
```cpp
#include "geometry_msgs/msg/twist.hpp"
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
auto velocityMsg = geometry_msgs::msg::Twist();
velocityMsg.linear.x = 0.1;
velocityMsg.linear.y = 0.1;
velocityMsg.linear.z = 0.1;
velocityMsg.angular.x = 0.1;
velocityMsg.angular.y = 0.1;
velocityMsg.angular.z = 0.1;

```
### Python declaration
```python
from geometry_msgs.msg import Twist
self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
self.cmd.linear.x = 0.0
self.cmd.linear.y = 0.0
self.cmd.linear.z = 0.0

self.cmd.angular.x = 0.0 
self.cmd.angular.y = 0.0
self.cmd.angular.z = 0.0

self.cmd_pub.publish(self.cmd)
```
# [sensor_msgs](https://docs.ros.org/en/api/sensor_msgs/html/index-msg.html)/Image Message
***P.S. almost always goes with CvBridge***
### C++ declaration
```cpp
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/image.hpp"

auto velocityMsg = geometry_msgs::msg::Twist();
velocityMsg.linear.x = 0.1;
velocityMsg.linear.y = 0.1;
velocityMsg.linear.z = 0.1;
velocityMsg.angular.x = 0.1;
velocityMsg.angular.y = 0.1;
velocityMsg.angular.z = 0.1;

rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
virtual void cameraCallback(const sensor_msgs::msg::Image::SharedPtr cameraMsg);
static cv_bridge::CvImagePtr cvPtr; //Create OpenCv Image pointer
cvPtr = cv_bridge::toCvCopy(cameraMsg, "bgr8"); //translate ROS cameraMSG to //OpenCv image
```
### Python declaration
```python
self.__bridge = CvBridge()
self.__camera_cb = self.create_subscription(Image,'/drone_vision/image_from_airsim', self.__camera_cb, 1)
self.__frame = self.__bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
```

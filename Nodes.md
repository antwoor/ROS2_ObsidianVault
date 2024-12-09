every executable in ROS is a node in a graph. You can see the graph via
```zsh
rqt_graph
```

# C++ instance of node
In Headers you should define types of [[Messages|messages]] that yo will be using
### header
```cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

using namespace std::chrono_literals;
class TrajectoryPublisher : public rclcpp::Node
{
    public:
    TrajectoryPublisher();
    virtual ~TrajectoryPublisher() = default; // Виртуальный деструктор
    private:
    rclcpp::TimerBase::SharedPtr _timer =NULL;
    protected:
    virtual void timer_callback();
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _pub = NULL;
    geometry_msgs::msg::Twist message;
};
```
### source
```cpp
#include "turtle_pub.h"
TrajectoryPublisher::TrajectoryPublisher() : Node("turtle_comander"){
    this->message.linear.x =0.5;
    this->declare_parameter<std::string>("cmd_vel_topic", "turtle1/cmd_vel");
    std::string velocity = this->get_parameter("cmd_vel_topic").as_string();
    _pub = this->create_publisher<geometry_msgs::msg::Twist>(velocity, 10);
    _timer = this->create_wall_timer(
        500ms,
        std::bind(&TrajectoryPublisher::timer_callback, this)
    );    
}
void TrajectoryPublisher::timer_callback(){
    static double sin_X = 0;
    sin_X > 2 * M_PI ? sin_X = 0: sin_X += 0.1;
    RCLCPP_INFO(this->get_logger(), "driving turtle %f", sin_X);
    this->message.angular.z = sin(sin_X);
    _pub->publish(this->message);
}
```
it would be a good thing if you will define [[Topics|topics]] as a **parameters** like this
```cpp
this->declare_parameter<std::string>("cmd_vel_topic", "turtle1/cmd_vel");
```
### main.cpp
```cpp
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPublisher>());
    rclcpp::shutdown();
}
```
# Python instance of node
```python
class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_subscriber_ = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.qr_subscriber_ = self.create_subscription(
            String,
            'qr_code',
            self.qr_callback,
            10)
	def main(self):
        rclpy.spin(self)
        self.destroy_node()
```
### main.py
```python
def main(args=None):
    rclpy.init(args=args)
    navigator = Navigation()
    navigator.main()
    rclpy.shutdown()
```

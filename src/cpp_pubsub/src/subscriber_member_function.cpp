#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    // Constructor: Initializes the node and creates a subscriber as a shared pointer
    MinimalSubscriber(): Node("minimal_subscriber") {
      subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    // Callback function: Handles incoming messages and prints their content
    void topic_callback(const std_msgs::msg::String & msg) const {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    // Member variable: Stores the subscriber as a shared pointer
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); // Initialize ROS 2
  rclcpp::spin(std::make_shared<MinimalSubscriber>()); // Keep the node running
  rclcpp::shutdown(); // Shutdown ROS 2
  return 0;
}


// A SharedPtr is a type of smart pointer that ensures shared ownership of an object.
// When all SharedPtrs pointing to an object are destroyed, the object itself is automatically deleted.
// In ROS2, SharedPtr is commonly used to manage node components like publishers, timers, and subscriptions.
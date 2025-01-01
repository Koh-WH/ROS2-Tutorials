#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>

void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)  {
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv) {
  // Initializes ROS 2 C++ client library
  rclcpp::init(argc, argv);

  // Creates a node named add_two_ints_server
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

  // Creates a service named add_two_ints for that node and automatically advertises it over the networks with the &add method
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

  // Prints a log message when it’s ready
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  // Spins the node, making the service available
  rclcpp::spin(node);
  rclcpp::shutdown();
}


// Server: Waits for requests and processes them.
// Client: Sends requests and waits for responses.
// Server-Client: When you need immediate results or acknowledgment (e.g., triggering a robotic arm to move).
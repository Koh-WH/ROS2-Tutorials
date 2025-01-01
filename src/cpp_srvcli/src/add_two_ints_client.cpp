#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Checks if the correct number of arguments (3, including the program name) are passed. If not, it prints a usage message and exits
  if (argc != 3) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
      return 1;
  }

  // Creates a ROS 2 Node named add_two_ints_client that will be used for communication with the service
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
  // Creates a client object of type AddTwoInts from the example_interfaces package. The client connects to a service named add_two_ints.
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

  // The integers a and b are assigned values from the command-line arguments (argv[1] and argv[2]), converting them from strings to integers using atoll()
  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  // Sends the request asynchronously using async_send_request().
  auto result = client->async_send_request(request);
  // Wait for the result. Uses spin_until_future_complete() to wait for the result of the service call.
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}


// Server: Waits for requests and processes them.
// Client: Sends requests and waits for responses.
// Server-Client: When you need immediate results or acknowledgment (e.g., triggering a robotic arm to move).


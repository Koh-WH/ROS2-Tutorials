# ROS2 Tutorials

## Table of Contents
---
- [Tutorials](#tutorials)
- [Communication Paradigms in ROS](#communication-paradigms-in-ros)
---

### Tutorials 
<a name="tutorials"></a>
- **cpp_pubsub**: Simple pub and sub in Cpp
- **py_pubsub**: Simple pub and sub in py
- **cpp_srvcli**: Simple server and client in Cpp
- **tutorial_interfaces**: Creating own custom msg and srv files
- **more_interfaces**: Implementing custom interfaces
- **cpp_parameters**: Using parameters in Cpp
- **ros2doctor**: Checks all aspects of ROS2 to identify issues @ https://github.com/ros2/ros2cli/tree/humble/ros2doctor
- **plugins**: Creating and using plugins. Polygon_base and polygon_plugins example

### Communication Paradigms in ROS
<a name="communication-paradigms-in-ros"></a>
| **Feature**       | **Server-Client**                  | **Publisher-Subscriber**            |
|--------------------|------------------------------------|--------------------------------------|
| **Communication**  | Synchronous (request-response)    | Asynchronous (publish-listen)       |
| **Direction**      | Bidirectional (client ⇔ server)   | Unidirectional (publisher → subscriber) |
| **Use Case**       | On-demand actions                | Continuous or event-based data      |
| **Scalability**    | One-to-One                        | One-to-Many or Many-to-One          |
| **Dependency**     | Client depends on server's response | Independent operation               |


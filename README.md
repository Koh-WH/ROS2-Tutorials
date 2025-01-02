# ROS2 Tutorials

## Table of Contents
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
---

### Communication Paradigms in ROS
<a name="communication-paradigms-in-ros"></a>
| **Feature**       | **Server-Client**                  | **Publisher-Subscriber**            |
|--------------------|------------------------------------|--------------------------------------|
| **Communication**  | Synchronous (request-response)    | Asynchronous (publish-listen)       |
| **Direction**      | Bidirectional (client ⇔ server)   | Unidirectional (publisher → subscriber) |
| **Use Case**       | On-demand actions                | Continuous or event-based data      |
| **Scalability**    | One-to-One                        | One-to-Many or Many-to-One          |
| **Dependency**     | Client depends on server's response | Independent operation               |

### Plugins, Interfaces, and Parameters

| **Feature**         | **Server-Client**                                      | **Publisher-Subscriber**                                    |
|---------------------|--------------------------------------------------------|-------------------------------------------------------------|
| **Plugins**         | - Plugins can be used to extend server functionality (e.g., server-side logic). <br> - Clients may dynamically load different server plugins depending on the request. | - Plugins can be used to modify or extend publisher or subscriber behavior (e.g., custom data serialization). <br> - Subscribers may use different plugins to process received messages. |
| **Interfaces**      | - Interfaces define the communication format for client-server requests and responses. <br> - Service interfaces (e.g., `.srv` files) are used to define the request and response structure. | - Interfaces define the message structure for publisher-subscriber communication. <br> - Topic interfaces (e.g., `.msg` files) are used to define the data type being exchanged. |
| **Parameters**      | - Parameters allow the server to be configured dynamically based on client requests. <br> - Clients may also pass parameters to a server as part of the service request. | - Parameters are used to configure publishers or subscribers, e.g., topic names, queue sizes, etc. <br> - Publishers and subscribers can also query parameters to adjust behavior at runtime. |
---
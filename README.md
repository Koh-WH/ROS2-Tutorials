# ROS2 Tutorials
Done on ros2-humble-20241205 release

## Table of Contents
- [Tutorials](#tutorials)
- [Communication Paradigms in ROS](#communication-paradigms-in-ros)
- [TF Formats](#Representations-of-tf2-in-ROS-and-Their-Interrelations)
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
- **action_tutorials_interfaces**: Structure of an action definition
- **action_tutorials_cpp**: Action server and action client to exchange goals, feedback, and results
- **Composable Node**: Multiple nodes share the same process and memory space by being loaded into a container. This avoids the overhead of IPC by using shared memory, improving performance
- **create_launch_files**: Creating of launch files, can see cpp_pubsub `ros2 launch cpp_pubsub pubsub_launch.xml`
- **launch_tutorial**: Using Substitutions arg for launch files
- **learning_tf2_cpp**: Static/FixedFrame/DynamicFrame/Normal Broadcasters and Listeners
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

### Services (Server-Client), Topics (Publisher-Subscriber), and Actions (Action Server-Client)

| Feature       | Services (Server-Client) | Topics (Publisher-Subscriber) | Actions (Action Server-Client) |
|---------------|--------------------------|-------------------------------|--------------------------------|
| **Communication** | Synchronous (request-response) | Asynchronous (publish-subscribe) | Asynchronous with feedback and result |
| **Direction** | Bidirectional (client ⇔ server) | Unidirectional (publisher → subscriber) | Bidirectional with feedback (client ⇔ server) |
| **Use Case** | On-demand operations requiring immediate response | Continuous or event-driven data streams | Long-running tasks requiring feedback and the ability to cancel |
| **Scalability** | One-to-One | One-to-Many or Many-to-One | One-to-One |
| **Dependency** | Client depends on server's availability | Decoupled; publishers and subscribers operate independently | Client depends on server's availability; feedback and result are optional |
| **State Management** | Stateless; each request is independent | Stateless; messages are independent | Stateful; maintains goal status and provides feedback |
| **Timeout Handling** | Clients can set timeouts for requests | Not applicable; messages are sent without acknowledgment | Clients can set timeouts and cancel goals if needed |
| **Example Use Cases** | Retrieving sensor data on demand, triggering specific actions | Broadcasting sensor data, status updates | Navigating to a location, performing complex manipulations |
---

### TF Representations
<a name="Representations-of-tf2-in-ROS-and-Their-Interrelations"></a>

| **Representation**       | **Components**                       | **Use Case**                                     | **Advantages**                              | **Disadvantages**                       | **Relationship**                                                                                                                                      |
|---------------------------|---------------------------------------|-------------------------------------------------|--------------------------------------------|-----------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------|
| **Translation + Rotation** | Translation: `[x, y, z]` <br> Rotation: `[roll, pitch, yaw]` or quaternion `[qx, qy, qz, qw]` | Basic transformation operations.               | Simple, intuitive.                      | Rotation may cause gimbal lock (with Euler angles). | Forms the base for all transformations. Translation and rotation are separated for clarity.                                                       |
| **Homogeneous Matrix**    | 4×4 matrix combining translation and rotation. Example: <br> \[R | T\] <br> \[0 | 1\] | Composite transformations, chaining.          | Unified representation, efficient math.  | Large memory usage, less intuitive.   | Converts translation and rotation into a single structure. Used in many internal calculations, especially in libraries like `tf2` and `Eigen`.        |
| **Quaternions**           | 4D vector `[qx, qy, qz, qw]`.        | Rotation in 3D space.                          | Avoids gimbal lock, compact.              | Less human-readable.                  | Represents rotation. Often derived from or converted to/from Euler angles or rotation matrices.                                                     |
| **Euler Angles**          | `[roll, pitch, yaw]` (angles in radians). | Human-readable rotations, debugging.           | Intuitive.                                | Prone to gimbal lock.                 | Converts to/from quaternions or rotation matrices. Euler angles are typically used for user input or display.                                        |
| **TransformStamped**      | - `header`: Timestamp, parent frame. <br> - `child_frame_id`: Target frame. <br> - `transform`: Translation `[x, y, z]` + Rotation `[qx, qy, qz, qw]`. | Communicating transformations in ROS.         | ROS integration, real-time communication. | ROS-specific.                        | Wrapper for broadcasting transformations between frames using `tf2_ros` or `tf2`.                                                                   |
| **tf2::Transform**        | - Translation (`tf2::Vector3`) <br> - Rotation (`tf2::Quaternion`). | Coding transformations in C++.                 | Unified, robust.                         | Requires C++ API.                     | Core class in the `tf2` library. Converts to/from `TransformStamped` and other representations like quaternions or matrices.                         |
| **Pose**                  | - `position`: `[x, y, z]` <br> - `orientation`: `[qx, qy, qz, qw]`. | Representing an object's position and orientation. | Simple, commonly used in ROS.            | Limited to position + orientation.    | Derived from or converted to/from `TransformStamped` for ROS messages. Typically used with `geometry_msgs/Pose`.                                     |
| **2D Transformations**    | - Translation: `[x, y]` <br> - Rotation: Single angle `theta`. | Robotics in 2D (e.g., planar navigation).      | Simple for 2D applications.              | Limited to planar applications.       | Simplified version of 3D transformations. Commonly used for mobile robots and maps.                                                                 |
| **Static Transform**      | Same as `TransformStamped` but fixed over time. | Static relationships between frames.           | Avoids overhead of publishing repeatedly. | Not dynamic.                          | Static transforms are used for frames with constant relationships (e.g., sensor mounting offsets). Converted to/from `TransformStamped` when needed. |

### Interrelations Between Representations

1. **Quaternions ↔ Euler Angles:**
   - Quaternions can be converted to Euler angles for human-readability and debugging.
   - Euler angles can be converted to quaternions for robust 3D transformations.

2. **Homogeneous Matrix ↔ Translation + Rotation:**
   - Translation and rotation can be combined into a homogeneous matrix for easier math operations.
   - The homogeneous matrix can be decomposed back into translation and rotation components.

3. **tf2::Transform ↔ TransformStamped:**
   - `tf2::Transform` is a C++ class for handling transformations in code.
   - `TransformStamped` is a ROS message type for communicating transformations between nodes.

4. **Pose ↔ TransformStamped:**
   - A pose represents a single transformation between two frames.
   - `TransformStamped` is used for broadcasting transformations between frames in a ROS environment.

5. **Static Transform ↔ TransformStamped:**
   - Static transforms are special cases of `TransformStamped` where the relationship between frames does not change over time.
---

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

### Communication Paradigms in ROS
<a name="communication-paradigms-in-ros"></a>
| **Feature**       | **Server-Client**                  | **Publisher-Subscriber**            |
|--------------------|------------------------------------|--------------------------------------|
| **Communication**  | Synchronous (request-response)    | Asynchronous (publish-listen)       |
| **Direction**      | Bidirectional (client ⇔ server)   | Unidirectional (publisher → subscriber) |
| **Use Case**       | On-demand actions                | Continuous or event-based data      |
| **Scalability**    | One-to-One                        | One-to-Many or Many-to-One          |
| **Dependency**     | Client depends on server's response | Independent operation               |
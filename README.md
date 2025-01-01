# ROS2 Tutorials


### Communication Paradigms in ROS
| **Feature**       | **Server-Client**                  | **Publisher-Subscriber**            |
|--------------------|------------------------------------|--------------------------------------|
| **Communication**  | Synchronous (request-response)    | Asynchronous (publish-listen)       |
| **Direction**      | Bidirectional (client ⇔ server)   | Unidirectional (publisher → subscriber) |
| **Use Case**       | On-demand actions                | Continuous or event-based data      |
| **Scalability**    | One-to-One                        | One-to-Many or Many-to-One          |
| **Dependency**     | Client depends on server's response | Independent operation               |
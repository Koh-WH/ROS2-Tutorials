Actions are defined in .action files of the form:
```
# Request
---
# Result
---
# Feedback
```
An action definition is made up of three message definitions separated by ---.
A request message is sent from an action client to an action server initiating a new goal.
A result message is sent from an action server to an action client when a goal is done.
Feedback messages are periodically sent from an action server to an action client with updates about a goal.
An instance of an action is typically referred to as a goal.


### To do
Build
Source
$ ros2 interface show action_tutorials_interfaces/action/Fibonacci

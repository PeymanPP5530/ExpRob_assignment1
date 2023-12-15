## First assignment of Experimental Robotics Laboratory
### Team members
| name | surname | ID |
| :------------ |:---------------:| :-----:|
|  Peyman | Peyvandipour | 5573284 |
| Arghavan   | Dalvand         |   5606362 |
| Nafiseh   | Monavari        |    5174160 |
### Project Description
Welcome to the ROSbot Marker Navigation project! This project focuses on implementing a sophisticated control system for a ROSbot, enabling it to navigate through an environment, detect, and interact with Aruco markers. The project spans both simulation and real-world scenarios, utilizing the Gazebo environment for simulation and a physical ROSbot for real-world experimentation.


#### Goal:
The primary objective of this assignment is to develop a robust control mechanism for a ROSbot, facilitating its movement to locate and reach four distinct Aruco markers with specific IDs: 11, 12, 13, and 15.

#### Implementation Phases:
-  Simulation Environment:
The initial phase involves working within the Gazebo simulation environment, where a ROSbot equipped with a fixed camera undergoes testing and refinement of its behavior. The focus is on finding and reaching the markers using a set of predefined instructions.

-  Real-world Implementation:
The project progresses to the real ROSbot deployed in a laboratory setting. The behavior, initially validated in simulation, is further tested and adjusted to ensure seamless operation in a physical environment.

#### Marker Interaction Logic:

The IDs of the markers hold specific instructions for the robot:

- Marker 11: Instructs the robot to rotate until it locates marker 12, then move towards and reach it.
- Marker 12: Directs the robot to rotate until it identifies marker 13, then moves towards and reaches it.
- Marker 13: Guides the robot to rotate until it detects marker 15, then moves towards and reaches it.
- Marker 15: Signals the completion of tasks. The robot stops once marker 15 is found.

------------

Please note that the condition for **reaching** a marker involves ensuring that at least one side of the marker is greater than a specified pixel threshold in the camera frame.

#### Project Details:

- ROS Version: This project is implemented in [ROS Noetic](https://wiki.ros.org/noetic "ROS Noetic").
- Simulation Environment: Gazebo is utilized for simulation purposes.
- Real Robot: A physical [ROSbot 2](https://husarion.com/manuals/rosbot/ "ROSbot 2") is employed for real-world testing.

#### Getting Started:




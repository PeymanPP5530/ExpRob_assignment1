
### Team Members
| name | surname | ID |
| :------------ |:---------------:| :-----:|
|  Peyman | Peyvandi Pour | 5573284 |
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

- Marker 11: Instruct the robot to rotate until it locates marker 12, then move towards and reach it.
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

The simulation requires the following steps for running:

- A ROS Noetic ([ROS Noetic installation instructions](http://wiki.ros.org/noetic/Installation:// "ROS Noetic installation instructions"))
- Run the ROS core by executing this command in terminal:
````shell
roscore
````
- Creat a ROS worksapace:
````shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
````
- Source the new setup.*sh file:
````shell
source ~/catkin_ws/devel/setup.bash
````
- Move to the `src` folder of the workspace:
````shell
cd ~/catkin_ws/src   
````
- Clone the package of the assignment:
````shell
git clone https://github.com/PeymanPP5530/ExpRob_assignment1
````
- Then:
````shell
cd ~/catkin_ws 
catkin_make
````
- Now, it is possible to run the whole project by running the launch file:
````shell
roslaunch assignment1_experimental exp1_fix_cam.launch 
````

#### video:


https://github.com/PeymanPP5530/ExpRob_assignment1/assets/120266362/42a2cf75-820c-40d7-8585-5ade02851887


### Possible Improvements:
#### Enhanced Path Planning for Optimal Navigation:
Consider integrating a robust path planning system into the robot's navigation algorithm. This addition would significantly improve the efficiency of the robot's movement by allowing it to identify and follow more optimal paths through various environments. By employing advanced path planning techniques, the robot can navigate more efficiently, potentially reducing travel time and optimizing its movements around obstacles or complex terrains.

#### Refinement of Aruco Marker Reading Algorithm:
During testing scenarios, issues arose when the robot attempted to read Aruco markers placed at a significant distance. These errors affected the robot's accuracy in interpreting the markers' information. To address this challenge, refining and fine-tuning the marker reading algorithm is essential. Enhancing the algorithm's capabilities will enable the robot to detect and interpret Aruco markers more accurately, even when they are situated farther away. Also, improving the detection algorithm's robustness and accuracy will enhance the robot's ability to effectively utilize markers for navigation and decision-making processes.

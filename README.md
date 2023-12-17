
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


### Explanation of the Code

The Python script is designed to control a robot's movement based on marker data received from the `/marker_data` topic and camera information from `/camera/color/camera_info` topic. Here's an explanation of the code structure and its functionalities:

#### Overview
- The script initializes a ROS node named `robot_controller`.
- It subscribes to topics such as `/marker_data` and `/camera/color/camera_info` to receive marker information and camera data, respectively.
- Publishers for `/cmd_vel` and `/output/image_raw/compressed` topics are set up to control the robot's movement and publish image data, respectively.

#### Class `robot_controller`
- This class contains methods to handle different robot actions based on marker data and camera information.
- `camera_center_callback`: Retrieves the center coordinates of the camera.
- `reached`: Stops the robot's movement upon reaching a marker, removes the marker from the list, and displays a confirmation message.
- `approche`: Initiates the robot's forward movement towards the targeted marker.
- `allign`: Adjusts the robot's orientation to align with the marker's position.
- `search`: Rotates the robot to locate the marker.
- `marker_data_callback`: Receives marker data and updates the robot's marker-related information.

#### Control Loop `control_loop`
- The main loop executes until all markers in `self.marker_list` are reached.
- It checks the current marker ID and executes the appropriate action (`reached`, `approche`, `allign`, or `search`) based on marker detection and alignment.
- The loop ensures the robot proceeds through the marker list by adjusting its movement and orientation until all markers are reached.

#### Execution Flow in `main`
- Waits for other ROS nodes to initialize properly before creating and spinning the `robot_controller` instance.
- A separate thread is created to manage ROS callbacks (`spin_thread`).
- Initiates the control loop after a short delay to ensure proper initialization.

This code effectively controls the robot's movements based on marker data received, aiming to reach specified markers while adjusting its orientation and movements accordingly. Further improvements can be made for better efficiency and robustness in marker detection and navigation.



#### Flowchart:
![flowchart](https://github.com/PeymanPP5530/ExpRob_assignment1/assets/120266362/7f889902-b8eb-46d0-8176-10e06c2a924a)


#### video:


https://github.com/PeymanPP5530/ExpRob_assignment1/assets/120266362/42a2cf75-820c-40d7-8585-5ade02851887


### Possible Improvements:
#### Enhanced Path Planning for Optimal Navigation:
- Consider integrating a robust path planning system into the robot's navigation algorithm. This addition would significantly improve the efficiency of the robot's movement by allowing it to identify and follow more optimal paths through various environments. By employing advanced path planning techniques, the robot can navigate more efficiently, potentially reducing travel time and optimizing its movements around obstacles or complex terrains.

#### Refinement of Aruco Marker Reading Algorithm:
- During testing scenarios, issues arose when the robot attempted to read Aruco markers placed at a significant distance. These errors affected the robot's accuracy in interpreting the markers' information. To address this challenge, refining and fine-tuning the marker reading algorithm is essential. Enhancing the algorithm's capabilities will enable the robot to detect and interpret Aruco markers more accurately, even when they are situated farther away. Also, improving the detection algorithm's robustness and accuracy will enhance the robot's ability to effectively utilize markers for navigation and decision-making processes.

These proposed enhancements aim to elevate the robot's navigation capabilities and accuracy in marker-based guidance. By implementing advanced path planning techniques and refining the marker detection algorithm, the robot can navigate more effectively, ensuring smoother movements and improved decision-making in diverse and challenging environments.

# SDC Assignment 3 - EKF/UKF Fusion
###### tags: `Self-Driving Cars`, `ROS`, `ROS2`

## Goal

1. Learn about **TF** in ROS
2. Learn how to **read rosbags**
3. Learn how to **visualize data** in Rviz
4. Learn how to **estimate robot state** by fusing */radar_odometry* and */gps* with **EKF/UKF**

## Environment

You can use either **ROS1** or **ROS2** to complete this assignment. The package we provide should be compatible for both ROS version. If you want to implement **your own library**, you should visualize the result in the same form and provide all your code along with instructions to how to run it properly.

## Data

- A rosbag from **NuScenes Dataset** ([Download](https://drive.google.com/drive/folders/1bHeqaeDPkoUZvftXLduIlS3M1ltpUt3G?usp=sharing))
  If you are using ROS2, unzip the file and you should see two files in one direcotry
  ![tree](https://i.imgur.com/a42Ownj.png)
  For ROS1 user, just download `sdc_hw3.bag`. If you are using the distribution of **Noetic**, make sure to download `sdc_hw3_noetic.bag`. Otherwise, you will not be able to lookup transform between different frames.
  
- Play the bag with the following command 
  ```bash
  # FOR ROS1
  rosbag play sdc_hw3.bag --clock
  # FOR ROS2
  ros2 bag play sdc_hw3 --clock
  ```
  The distro of **foxy** cannot handle `--clock` command, so you should reinstall the newest version of **galactic**.
  The `--clock` argument is for overwriting ROSTime with the time when the bag is recorded. If you play a bag with this command, remember to set a rosparam of `use_sim_sime` by the following command.
  ```bash
  # FOR ROS1
  rosparam set use_sim_time true
  # FOR ROS2
  ros2 param set <node_name> true # replace <node_name> with your node_name
  ```
  
- Topics
  ![topics](https://i.imgur.com/WLPQ7M9.png)

  **/radar_odometry** is the **local measurement** which have accumulated error.
        
  **/gps** is the **global measurement** which only provides x,y position at 1Hz.


## Library

You should use this package [robot_localization](https://github.com/cra-ros-pkg/robot_localization) for fusing /radar_odometry and /gps

Please read following sections to accomplish this assignment.
- [State Estimation Nodes](http://docs.ros.org/melodic/api/robot_localization/html/state_estimation_nodes.html)
- [Preparing Your Data for Use with robot_localization](http://docs.ros.org/melodic/api/robot_localization/html/preparing_sensor_data.html)
- [Configuring robot_localization](http://docs.ros.org/melodic/api/robot_localization/html/configuring_robot_localization.html#)
- [Working with the robot_localization Package](https://roscon.ros.org/2015/presentations/robot_localization.pdf)


### Hints

1. `robot_localization` is originally written for ROS1. You can switch to ros2 branch by `git checkout ros2` to make it compatible with ROS2 workspace.
2. You will need to modify the parameter yaml file as follow:
```yaml
odom_frame: /origin
base_link_frame: /car
world_frame: /origin
frequency: 10
sensor_timeout: 0.75
two_d_mode: true
transform_time_offset: 0
publish_tf: false
```
3. When compiling, you may encounter errors of package not found. `geographiclib` is necessary for `robot_localization` to handle coordinate system conversion. Follow [this](https://hackmd.io/@Biomotion/ryyhBVXEY) link to compile and install `geographiclib`.
4. Sometimes when you are compiling new packages, you may see compile error of package not found like `geometry_msgs`. The simplist way to try is installing it via apt package. (`sudo apt install ros-noetic-geometry-msgs` with using ROS Noetic as an example.)
5. In this task, we only want to fuse the **xy position** of the two measurements. Try to understand the documents provided and modify the yaml file accordingly.

## Visualization

We recommand using RViz for visualize the result.

- Launch rviz with the command:
  ```bash
  # FOR ROS1
  rviz
  # FOR ROS2
  rviz2
  ```
- Add topics you want to show and visuzlize the fusion result.(In our example, we've visualized three odometry topics along with their covariance and point cloud topic from lidar.)
- **Note**: You need to set correct `frame_id` in order to render all the objects in the 3D viewer. Change the value of **Global Options > Fixed Frame** to where you want your data to be rendered on.

## Assignment

### EKF Fusion
[Example Video](https://youtu.be/aiHl3NwYeHA)

In the following picture, **RED** arrows are ground truth odometry, **GREEN** arrows are EKF result, **BLUE** arrows are radar odometry, **PURPLE** ecllips represents covariance.
- Screenshot of the result using **only GPS** in EKF.

    ![Pic](https://i.imgur.com/AwJxrJP.png)
- Screenshot of the result using **only radar odemetry** in EKF.

    ![Pic](https://i.imgur.com/YcbIw3l.png)
- Screenshot of the result using **both GPS and radar odemetry** in EKF.

    ![Pic](https://i.imgur.com/T2XOg1m.png)
    
**Please include your personal information to prove that you have finished this on your own. (Ex. your username in terminals)**

### Discussion

1. How do robot_localization package know the **covariance matrix** of GPS and radar odometry?
2. What is the covariance matrix of GPS and what does it mean?
3. In the yaml file, do you set differential parameter of odometry and GPS to **true**? or **false**? Why?

### Bonus

- Complete sensor fusion by **UKF**.
- Compare EKF result with UKF result. Describe your findings and explain why.


## Submission

Please Hand-in The Following files:

1. Result and Discussion (filename: ekf_result.pdf)
    - Screenshot of EKF Results Along with Your Personal Information(60%)
    - Discussion Answers (40%)
    - Bonus (10%)

2. Source code
    - your yaml config file

Pack all files in a zip file: `<student_id>_hw3.zip`
Deadline: 10/20 22:00



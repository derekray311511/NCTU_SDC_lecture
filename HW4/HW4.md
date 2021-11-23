# SDC Assignment 4 - ICP LiDAR Localization
###### tags: `Self-Driving Cars`, `ROS`, `ROS2`

## Goal

1. Learn how to **match LiDAR scans** of a self-driving car given a point cloud map.
2. Learn how to utilize **ICP** in PointCloudLibrary(PCL).
3. Learn how to  transform sensor messages to the correct frame by **tf library**.
4. Learn how to **publish** and **subscribe** messages on ROS
5. It's a first try to the **midterm localization competition**


## Environment

You can use either **ROS1** or **ROS2** to complete this assignment. If you want to implement **your own library**, you should visualize the result in the same form and provide all your code along with instructions to how to run it properly.

## Data

- A rosbag from **ITRI** ([Download](https://drive.google.com/drive/folders/18vENtgMtraqEZDuovZMZtXkJwmAGVh2Q?usp=sharing))
    If you are using ROS2, unzip the file and you should see two files in one direcotry
    For ROS1 users, just download `sdc_hw4.bag`. If you are using the distribution of **Noetic**, make sure to download `sdc_hw4_noetic.bag`. Otherwise, you will not be able to lookup transform between different frames.
  
- Play the bag with the following command 
    ```bash
    # FOR ROS1
    rosbag play sdc_hw4.bag --clock
    # FOR ROS2
    ros2 bag play sdc_hw4 --clock
    ```
    The `--clock` argument is for overwriting ROSTime with the time recorded in bag. If you play a bag with this command, remember to set a rosparam of `use_sim_sime` by the following command.
    ```bash
    # FOR ROS1
    rosparam set use_sim_time true
    # FOR ROS2
    ros2 param set <node_name> true # replace <node_name> with your node_name
  ```
    If your computer is not fast enough, your node may drop some message if the queue is full. You can change the rate of playing bags by `-r`  arguement.
    ```bash
    # FOR ROS1
    rosbag play sdc_hw4.bag --clock -r 0.1
    # FOR ROS2
    ros2 bag play sdc_hw4 --clock -r 0.1
    ```
- Topics
    ![topics](https://i.imgur.com/R7R7eZ9.png)
    You should read `/lidar_points` and match it onto the map  provided in the next section.

- Map ([Download](https://drive.google.com/file/d/1TfhjO_Sjr72NizrZ4VUubRFM9vlO0aba/view?usp=sharing))
    This is a point cloud map of ITRI campus. The map is in pcd format which is defined by PCL. You can visualize it in RViz.
    ![Map](https://i.imgur.com/KFQ0rar.png)


## Library

PointCloudLibrary(PCL) is a library for processing 2D/3D point clouds. It also provides several popular scan matching algorithms including ICP. By performing scan matching, we can estimate the relative pose between two point clouds. In order to localize where the SDC is, we simply perform scan matching between current LiDAR scan and map.


Please read the following documents to learn how to utilize PCL in ROS and process point clouds:
- [Point Cloud Library | The Point Cloud Library (PCL) is a standalone, large scale, open project for 2D/3D image and point cloud processing.](https://pointclouds.org/)
- [Introduction — Point Cloud Library 0.0 documentation](https://pcl.readthedocs.io/projects/tutorials/en/latest/#)
- [Reading Point Cloud data from PCD files](https://pcl.readthedocs.io/projects/tutorials/en/latest/reading_pcd.html)
- [Downsampling a PointCloud using a VoxelGrid filter](https://pcl.readthedocs.io/projects/tutorials/en/latest/voxel_grid.html)
- [pcl_conversions: pcl_conversions.h File Reference](http://docs.ros.org/en/indigo/api/pcl_conversions/html/pcl__conversions_8h.html)
- [Point Cloud Library (PCL): pcl::IterativeClosestPoint< PointSource, PointTarget, Scalar > Class Template Reference](https://pointclouds.org/documentation/classpcl_1_1_iterative_closest_point.html)
- [How to use iterative closest point — Point Cloud Library 0.0 documentation](https://pcl.readthedocs.io/projects/tutorials/en/latest/iterative_closest_point.html)

### Hints
1. The frame_id of the lidar scans is `velodyne`.
3. [tf2](http://wiki.ros.org/tf2) is a tool for lookup transform between different frames in ROS.
4. Be aware of the frame_id of every messages you're using, including new messages you want to publish.
5. You should set the frame_id in header of map PointCloud2 as `world` in order to show in RViz
6. You should use GPS measurements as the initial guess of the **first** ICP matching. After the first ICP matching, you can simply use the result as the initial guess of next ICP matching. To specify initial guess:
    ```cpp
    icp.align(*cloud, initial_guess);
    ```
6. GPS didn’t provide rotation information of self-driving car, so we provide the yaw angle of the first point. You should use this value in your initial guess:
    ```
    yaw =  -2.2370340344819 (rad)
    ```

## Assignment

### Publish and Visualize Point Cloud Map
Publish to a topic named `map` at 1Hz and visualize it in RViz. It should look like the image below.
![Map](https://i.imgur.com/KFQ0rar.png)

### ICP Localization
You will need to create a new node to perform the following tasks:
- subscribe `/gps` for initialization
- subscribe `/lidar_points` for scan matching.
- downsample the point cloud of the scans using voxel grid filter to reduce the computation time
- transform lidar scans to `base_link` frame by `pcl_ros::transformPointCloud`
- match the points onto point cloud map using ICP
- transform lidar scans to `world` frame and publish
- publish your localization result as `nav_msgs/Odometry.msg`
    ```
    /header/frame_id = "world"
    /child_frame_id = "base_link"
    ```

Please visualize your result in the same picture as shown below.

![Localization Result](https://i.imgur.com/et3Qu8Z.jpg)
**Red Arrows**: ground truth odometry
**Blue Arrows**: your result
**Cyan Dots**: GPS measurements
**White Points**: point cloud from LiDAR
**Colorful Points**: point cloud map

## Submission

Please Hand-in a PDF file including **screenshots** of your result, including:
1. Map point cloud
2. Localization result with all information mentioned above

Naming Rule: `<student_id>_hw4.pdf`
Deadline: 11/3 22:00



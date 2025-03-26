# Pepper-SLAM

A project for running and evaluating SLAM algorithms with the Pepper robot.

## Description

This project contains the necessary files to run and evaluate SLAM algorithms with the Pepper robot.

Using ROS Noetic, algorithms such as Gmapping with depth camera data, Gmapping with predicted depth data using Depth Anything, and ORB-SLAM3 in a monocular setting have been implemented.

To create an isolated environment without dependency issues, Docker was used. The project was developed in VSCode, using the VSCode Dev Containers extension, which makes development inside a container easier.

## Getting Started

### Dependencies

The recommended way of running this project is using Docker, VSCode and VSCode Dev Containers extension. This way an isolated and consistent environment is created and you do not need to worry about dependency issues or impacting your local machine. The environment uses ROS Noetic and Ubuntu 20.04.

### Installing

Open the VSCode Command Palette and select:

```
Dev Containers: Reopen in Container
```

If you wish to use ORB-SLAM3, it can be realized using [orb_slam3_ros](https://github.com/thien94/orb_slam3_ros). To set up the package, wait for catkin workspace to build, then refer to the source instructions. Note that you may need to make some adjustments to build the Pangolin package. You can clone the orb_slam3_ros repository into ```catkin_ws/src```.

Since ORB-SLAM3 requires camera parameters, and they can differ based on Pepper's version, you should obtain them manually. The necessary camera matrix and distortion parameters for ORB-SLAM3 in the monocular setting can be obtained using ROS camera_calibration package. The front camera parameters can be obtained analogous to [Pepper Calibration](http://wiki.ros.org/pepper/Tutorials/Calibration). To see how camera matrix and distortion parameters relate to the ones required for the settings file, you can refer to [OpenCV Calibration](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html).

Once you have obtained them, you can copy the Pepper config to the catkin workspace using the command:

```
cp src/orb_slam3_ros/config/Monocular/Pepper.yaml catkin_ws/src/orb_slam3_ros/config/Monocular/Pepper.yaml
``` 
Substitute Camera1 parameters with your own.

Similarly, you can copy the launch file with the command:
```
cp src/orb_slam3_ros/launch/pepper_mono.launch catkin_ws/src/orb_slam3_ros/launch/pepper_mono.launch
``` 
Make sure the camera's raw image topic matches with your robot's.

## Usage

### Pepper setup

Communication with the Pepper robot can be realized using [naoqi_driver](https://github.com/ros-naoqi/naoqi_driver).

Before launching naoqi_driver, you should connect to the robot and launch a few commands. To prevent Pepper's Autonomous Life mode with interfering with the algorithms, it is recommended to turn it off:

```
ssh username@<pepperip>
qicli call ALAutonomousLife.setState disabled
qicli call ALMotion.wakeUp
```

Additionally, you can make Pepper face forward direction with the following commands:

```
qicli call ALMotion.setAngles "HeadPitch" 0.0 0.1
qicli call ALMotion.setAngles "HeadYaw" 0.0 0.1
```

### Data collection

If you wish to compare and analyze SLAM algorithms later, collecting data can be a great way to iterate through algorithms without physically moving the robot again.

To control the robot, you can launch:
```
rosrun rqt_robot_steering rqt_robot_steering
```
This will open a tool that lets you control the linear and angular velocity of the robot. Make sure that the velocity topic matches your robot's.

Once you are ready, you can record all of the published topics by opening a new terminal and running:

```
rosbag record -O path_to_recording.bag -a
```

### Running SLAM

SLAM can be realized either with the pre-recorded data or in real-time. The process is very similar.

If necessary, you can launch roscore for other ROS processes to work:
```
roscore
```

If you wish to evaluate the estimated trajectory using the evaluation script, you can record it by opening a new terminal and launching:
```
python src/mapping/mapping/scripts/traj_rec.py
```

Note that for ORB-SLAM3 you may change the subscribed topic from ```/trajectory``` to ```/orb_slam3_ros/trajectory```.

This will create a traj.csv file, once the script has finished.

Depending on the SLAM algorithm, the process varies.

* To run Gmapping with depth camera data, simply open a new terminal and run:
    ```
    roslaunch pepper_mapping pepper_mapping.launch
    ```

* If you wish to run Gmapping with predicted depth data using Depth Anything, you should first download and put the trained models under ```checkpoints``` directory. The expected metric depth estimation model is the indoor one trained on the NYUv2 dataset. For details, refer to [Metric Depth Estimation](https://github.com/LiheYoung/Depth-Anything/tree/main/metric_depth).

    Once it is done, you can launch the algorithm in a separate terminal using:

    ```
    roslaunch pepper_mapping pepper_mapping_nn.launch
    ```

* In the case of ORB-SLAM3, if you wish to use the evaluation script with the results, you should call another conversion script, since the evaluation script expects an occupancy grid, while ORB-SLAM3 operates with point clouds. The conversion script in another terminal can be run with the following command:

    ```
    python src/mapping/mapping/scripts/pcd2grid.py
    ```

    To run ORB-SLAM3 itself, open a new terminal and use the command:

    ```
    roslaunch orb_slam3_ros pepper_mono.launch
    ```

If you are using pre-recorded data, play the recording to simulate movement by opening a new terminal and launching:
```
rosbag play path_to_recording.bag --clock
```
The --clock argument ensures that the bag time is published, which can be used by hector_trajectory_server.

Otherwise you can start controlling the robot with your chosen method.

Save the constructed map by opening a new terminal and using map_server:
```
rosrun map_server map_saver -f path_to_map
```

### Evaluation

Once SLAM has finished, you can run the evaluation script with the command:

```
python scripts/mapping/evaluation/calc_res.py
```

This script starts an interactive tool that lets you measure the accuracy of the reconstructed map and the predicted trajectory. An initial automatic transformation is applied to align the estimated and ground truth data based on the first point. However, the map itself can change orientation during SLAM, therefore it is possible to adjust it manually using the tool. Using the appropriate keys, you can translate and rotate the map further. Once it has been aligned with the ground truth contour, you can click on the map contour points to obtain the predicted contour. This will create a results file with the predicted trajectory error and map contour IoU.

To learn more about it, refer to the script directly.

## License

This project is licensed under the GNU General Public License, Version 3.0 - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

I would like to thank everyone whose work I used in this project.
The projects that were particularly helpful in implementation:

* [ros-naoqi](https://github.com/ros-naoqi)
* [Pepper Mapping](https://github.com/mreimbold/pepper_mapping)
* [Depth Anything](https://github.com/LiheYoung/Depth-Anything)
* [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
* [ORB-SLAM3-ROS](https://github.com/thien94/orb_slam3_ros)
# EECE 5554 Final Project - Team 13

## Usage

1. Install [gtsam](https://github.com/borglab/gtsam/releases) (Georgia Tech Smoothing and Mapping library, 4.0.0-alpha2)
    ```
    wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip
    cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/
    cd ~/Downloads/gtsam-4.0.0-alpha2/
    mkdir build && cd build
    cmake ..
    sudo make install
    ```

2. Pull the project file and compile
    ```
    git clone https://gitlab.com/5554sec2group1/final-project.git --recurse-submodules
    cd final-project/lego-loam/catkin_ws/src/
    catkin_make -j1
    ```

3. Launch

 - Launch the lego-loam algorithm:
    ```
    cd ..
    source devel/setup.sh
    roslaunch lego_loam run.launch
    ```
  - Play your rosbag file

    In a different terminal, run the following:
    ```
    rosbag play /path/to/bag --clock --topic /ns2/velodyne_points
    ```
    Remember change the ```/path/to/bag``` to your bag file path.

    In this project, we are using the ```morning_stereo_rgb_ir_lidar_gps.bag``` file.


## Parameters
Run the above command will let you do the lego-loam algorithm using the right lidar with loop closure on and imu data off.

utility.h file is in the following directory:
```
catkin_ws/src/LeGO-LOAM/LeGO-LOAM/include/utility.h
```

1. For using the left lidar:
  - Modify the ```utility.h``` line 53 to ```/ns1/velodyne_points```;
  - Recompile and launch lego-loam algorithm again;
  - Play the rosbag file. When running the ```rosbag play``` command, change ```ns2``` to ```ns1```.
2. For switching the loop closure on/off:
  - Modify the ```utility.h``` line 104 to ```false```;
  - Recompile and launch lego-loam algorithm again;
  - Play the rosbag file.
3. For using the IMU data(not recommended):
  - Change your command of playing the rosbag with following:
    ```
    rosbag play /path/to/bag --clock --topic /ns2/velodyne_points /imu/imu
    ```
    (add /imu/imu in the end)
4. For using the joint point cloud
  - Uncomment out line 18 in the ```run.launch``` file
  - Change the ```pointCloudTopic``` to ```/velodyne_points``` in ```utility.h``` header file
  - Change the ```useCloudRing``` flag to ```false``` in in ```utility.h``` header file

## Commands about Velodyne

For looking into the data gathered by velodyne lidar, you will need the following commands:

1. Install ROS Dependency for Velodyne VLP16 LiDAR:
   ```
    sudo apt-get install ros-kinetic-velodyne
    ```
2. Install the dependency:
    ```
    cd catkin_ws/src/velodyne
    rosdep install --from-paths . --ignore-src --rosdistro kinetic -y
    ```
3. Compile the catkin workspace:
    ```
    cd catkin_ws
    catkin_make
    source devel/setup.bash
    ```
4. Run roscore and rviz:
    ```
    roscore
    rosrun rviz rviz -f velodyne
    ```
    Add "PointCloud2" and Change the Fixed frame to ```velodyne```
5. Play your rosbag file.

## Instructions of implementing SqueezeSeg

1. The instructions are tested on Ubuntu 16.04 with python 2.7 and tensorflow 1.4

2. Clone the SqueezeSeg repository. The root directory is named as $SQSG_ROOT.

3. Use pip to install required Python packages:
    ```
    pip install -r requirements.txt
    ```
4. To run the demo script:
    ```
    cd $SQSG_ROOT/
    python ./src/demo.py
    ```
5. If the installation is correct, the detector should write the detection results as well as 2D label maps to $SQSG_ROOT/data/samples_out.
   You can test any .npy format data by put the file under "SQSG_ROOT/data/samples/" folder.
    
6. NOTE: There are two ROS Package at "SQSG_ROOT/ROS_packages/" which are used to visualize the segmentation results in Rviz.
   However, they are still need to be debuged. The main issue is to publish the sqeeuze_seg/points topic to correct node, but currently the rqt graph is always blank.

7. To use any .bag file which have Velodyne points visualize the SqueezeSeg result in Rviz: 
    (Bug exists at the rostopic node, still working on to find the issue)

    (1) move from squeezeseg_cpp_preprocessing folder to your ROS workspace;

    (2) open script/online.py and modify checkpoint path to your system;

    (3) open script/segment_node.py and modify sys.path.append path as well;

    (4) run catkin_make;

    (5) run roslaunch squeezeseg_cpp_preprocessing ss_cpp_preprocessing.launch;

    (6) open RViz and check squeeze_seg/points topic whether publishing correctly or not;

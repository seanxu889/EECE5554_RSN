# EECE5554_Sensing_Navigation
EECE5554 Robotics Sensing and Navigation Northeastern’s Autonomous Car (NUANCE) LiDAR Project

Data: car_IR_RGB_lidar data

In this dataset, Northeastern’s autonomous car(NUANCE) is driven manually along the streets of Newbury street in Boston. The dataset has stereo RGB cameras looking forward, IR camera looking forward, 2 Velodyne VLP-16 lidar mounted on top of the car, IMU, GPS. The main focus in this dataset was to collect camera data with at least one  loop closures. The sensors like lidars, gps, imu in combination can serve as ground truth for visual slam algorithms.

The camera calibration for the RGB cameras are available in rosbag, IR camera calibration is provided in the info.txt file. The specs of the cameras, Lidar are also provided in the info.txt file.

![image](https://github.com/seanxu889/EECE5554_Sensing_Navigation/blob/master/Data/car_IR_RGB_lidar_data_screenshot.png)

Results of Segmentation and Feature Extraction part

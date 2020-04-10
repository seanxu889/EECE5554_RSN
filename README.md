# EECE5554_Sensing_Navigation
EECE5554 Robotics Sensing and Navigation Northeastern’s Autonomous Car (NUANCE) LiDAR Project

## Data: car_IR_RGB_lidar data

In this dataset, Northeastern’s autonomous car(NUANCE) is driven manually along the streets of Newbury street in Boston. The dataset has stereo RGB cameras looking forward, IR camera looking forward, 2 Velodyne VLP-16 lidar mounted on top of the car, IMU, GPS. The main focus in this dataset was to collect camera data with at least one  loop closures. The sensors like lidars, gps, imu in combination can serve as ground truth for visual slam algorithms.

The camera calibration for the RGB cameras are available in rosbag, IR camera calibration is provided in the info.txt file. The specs of the cameras, Lidar are also provided in the info.txt file.

![image](https://github.com/seanxu889/EECE5554_Sensing_Navigation/blob/master/Data/car_IR_RGB_lidar_data_screenshot.png)

## Results of Segmentation and Feature Extraction part

Raw point cloud at time t: Pt = {p1, p2, …, pi, …, pn}:

![image](https://github.com/seanxu889/EECE5554_RSN/blob/master/results/2a_raw2.gif)

Extract ground points using column-wise evaluation (ground plane estimation):

![image](https://github.com/seanxu889/EECE5554_RSN/blob/master/results/2b_ground2.gif)

Apply image-based segmentation to the range image to group points into many clusters:

![image](https://github.com/seanxu889/EECE5554_RSN/blob/master/results/2b_seg2.gif)

Sort the points to edge and planar feature points:

![image](https://github.com/seanxu889/EECE5554_RSN/blob/master/results/2d.gif)

Further extract edge features and planar features:

![image](https://github.com/seanxu889/EECE5554_RSN/blob/master/results/2c.gif)

## Results of Mapping part

<a href="http://www.youtube.com/watch?feature=player_embedded&v=ZVJjqH2R0SQ
" target="_blank"><img src="http://img.youtube.com/vi/ZVJjqH2R0SQ/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>

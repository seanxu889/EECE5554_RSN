# EECE5554_Sensing_Navigation
EECE5554 Robotics Sensing and Navigation project using Northeastern’s Autonomous Car (NUANCE) data: 
Real-World Lidar Odometry and Mapping Using LeGO-LOAM and Point Cloud Road-Object Segmentation using SqueezeSeg

<p align="center">
<img src="https://github.com/seanxu889/EECE5554_RSN/blob/master/Data/NUance.jpg" height="220" width="1200">
</p>

## Data: car_IR_RGB_lidar data

In this dataset, Northeastern’s autonomous car(NUANCE) is driven manually along the streets of Newbury street in Boston. The dataset has stereo RGB cameras looking forward, IR camera looking forward, 2 Velodyne VLP-16 lidar mounted on top of the car, IMU, GPS. The main focus in this dataset was to collect camera data with at least one  loop closures. The sensors like lidars, gps, imu in combination can serve as ground truth for visual slam algorithms.

The camera calibration for the RGB cameras are available in rosbag, IR camera calibration is provided in the info.txt file. The specs of the cameras, Lidar are also provided in the info.txt file.

<p align="center">
<img src="https://github.com/seanxu889/EECE5554_Sensing_Navigation/blob/master/Data/car_IR_RGB_lidar_data_screenshot.png" height="300" width="600">
</p>

## LeGO-LOAM Point Cloud Segmentation and Feature Extraction

Raw point cloud at time t: Pt = {p1, p2, …, pi, …, pn}:
<p align="center">
<img src="https://github.com/seanxu889/EECE5554_RSN/blob/master/results/2a_raw2.gif">      
</p>
      
Project to a range image with size of1800x16. Each pixel represent a point pi and also associate the range value ri.
Extract ground points using column-wise evaluation (ground plane estimation):
<p align="center">
<img src="https://github.com/seanxu889/EECE5554_RSN/blob/master/results/2b_ground2.gif"> 
</p>

Apply image-based segmentation to the range image to group points into many clusters. Omit the clusters that have fewer than 30 points:
<p align="center">
<img src="https://github.com/seanxu889/EECE5554_RSN/blob/master/results/2b_seg2.gif"> 
</p>

Here, we have ground and large object points. Only save these points in range image. After segmentation, the processing efficiency and feature extraction accuracy are improved, and fast and reliable feature extraction can be performed.

Extract features from ground and segmented points. Evaluate the roughness / curvature of point pi use C. From all sub-images, we get 40 edge features and 80 planar features:
<p align="center">
<img src="https://github.com/seanxu889/EECE5554_RSN/blob/master/results/2d.gif"> 
</p>

Further extract edge features and planar features:
<p align="center">
<img src="https://github.com/seanxu889/EECE5554_RSN/blob/master/results/2c.gif"> 
</p>

Finally, we get edge and planar feature points. The number of features also greatly reduced. Based on these features, we can run into the Lidar odometry and mapping modules.

## LeGO-LOAM Final Mapping
click the GIF to view YouTube video showing mapping process and final 3D mapping: ns1_no_loop_closure_no_imu

<div align="center">
      <a href="http://www.youtube.com/watch?feature=player_embedded&v=ZVJjqH2R0SQ
" target="_blank">
      <img 
       src="https://github.com/seanxu889/EECE5554_RSN/blob/master/results/overview1.gif" 
       alt="IMAGE ALT TEXT HERE" width="560" height="360" border="10" />
       </a>
     </div>

## SqueezeSeg Point Cloud Road-Object Segmentation

demo results:
<p align="center">
<img src="https://github.com/seanxu889/EECE5554_RSN/blob/master/results/plot_2011_09_26_0093_0000000412.png"> 
</p>

<p align="center">
<img src="https://github.com/seanxu889/EECE5554_RSN/blob/master/results/plot_2011_09_26_0001_0000000030.png"> 
</p>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Header.h"
#include "tf2_ros/transform_listener.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include "utility.h"

ros::Publisher joined_lidar_pub;

std::atomic<uint32_t> joined_lidar_seq_num;

geometry_msgs::TransformStamped ns1_left_lidar_transform;
geometry_msgs::TransformStamped ns2_right_lidar_transform;

tf::StampedTransform lidar1tf;
tf::StampedTransform lidar2tf;

tf2_ros::Buffer tfBuffer;

uint32_t get_seq_num() {
    return joined_lidar_seq_num++;
}

void vlp16_sensor_1_callback(const sensor_msgs::PointCloud2::ConstPtr& lidar1_in)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr pclCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformedPclCloud;
  pclCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  transformedPclCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

  pcl::fromROSMsg(*lidar1_in, *pclCloud);
  pcl_ros::transformPointCloud(*pclCloud, *transformedPclCloud, lidar1tf);

  sensor_msgs::PointCloud2 msg = sensor_msgs::PointCloud2();

  pcl::toROSMsg(*transformedPclCloud, msg);

  msg.header = lidar1_in->header;
  msg.header.seq = get_seq_num();
  msg.height = lidar1_in->height;
  msg.width = lidar1_in->width;
  msg.fields = lidar1_in->fields;
  msg.is_bigendian = lidar1_in->is_bigendian;
  msg.is_dense = lidar1_in->is_dense;
  msg.point_step = lidar1_in->point_step;
  msg.row_step = lidar1_in->row_step;
  // msg.data = lidar1_in->data;
  joined_lidar_pub.publish(msg);
}

void vlp16_sensor_2_callback(const sensor_msgs::PointCloud2::ConstPtr& lidar2_in)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr pclCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformedPclCloud;
  pclCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  transformedPclCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  
  pcl::fromROSMsg(*lidar2_in, *pclCloud);
  pcl_ros::transformPointCloud(*pclCloud, *transformedPclCloud, lidar2tf);

  sensor_msgs::PointCloud2 msg = sensor_msgs::PointCloud2();

  pcl::toROSMsg(*transformedPclCloud, msg);

  msg.header = lidar2_in->header;
  msg.header.seq = get_seq_num();
  msg.height = lidar2_in->height;
  msg.width = lidar2_in->width;
  msg.fields = lidar2_in->fields;
  msg.is_bigendian = lidar2_in->is_bigendian;
  msg.is_dense = lidar2_in->is_dense;
  msg.point_step = lidar2_in->point_step;
  msg.row_step = lidar2_in->row_step;
  // msg.data = lidar2_in->data;
  joined_lidar_pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "velodyne_joined");

  ros::NodeHandle n;

  get_seq_num();

  joined_lidar_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 1000);

  // tf2_ros::Buffer tfBuffer;
  // tf2_ros::TransformListener tfListener(tfBuffer);

  tf::TransformListener tfListener;

  while(n.ok()) {
    try {
      tfListener.lookupTransform("cam_array_base_link", "vlp16_port", ros::Time(0), lidar1tf);
      break;
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
    }
  }

  while(n.ok()) {
    try {
      tfListener.lookupTransform("cam_array_base_link", "vlp16_starboard", ros::Time(0), lidar2tf);
      break;
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
    }
  }

  ROS_INFO("lidar1tf child frame id: %s", lidar1tf.child_frame_id_.c_str());
  ROS_INFO("lidar1tf frame id: %s", lidar1tf.frame_id_.c_str());
  ROS_INFO("lidar2tf child frame id: %s", lidar2tf.child_frame_id_.c_str());
  ROS_INFO("lidar2tf frame id: %s", lidar2tf.frame_id_.c_str());

  // try {
  //   ns1_left_lidar_transform = tfBuffer.lookupTransform("cam_array_base_link", "vlp16_port", ros::Time(0));
  // } catch (tf2::TransformException &ex) {
  //   ROS_WARN("%s", ex.what());
  // }

  // try {
  //   ns2_right_lidar_transform = tfBuffer.lookupTransform("cam_array_base_link", "vlp16_starboard", ros::Time(0));
  // } catch (tf2::TransformException &ex) {
  //   ROS_WARN("%s", ex.what());
  // }

  ros::Subscriber lidar_1_sub = n.subscribe("/ns1/velodyne_points", 1000, vlp16_sensor_1_callback);

  ros::Subscriber lidar_2_sub = n.subscribe("/ns2/velodyne_points", 1000, vlp16_sensor_2_callback);

  ros::spin();

  return 0;
}
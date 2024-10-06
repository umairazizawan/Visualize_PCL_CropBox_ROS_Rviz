# Visualize_PCL_CropBox_ROS_Rviz
Visualize_PCL_CropBox_ROS_Rviz is presents an approach For Debugging Transformation (TF) issues when working with PCL CropBox in ROS. 

PCL Cropbox only keeps the points which lye inside its bounds. But in case the coordinates and transformation of the Cropbox are not correct, it is difficult to find the location where the cropbox is actually being created and therefore correct the issue. 
This is especially the case when working with tf trees obtained from a client, customer or a third party, as they may not be completely accurate and it is difficult and time consiming to correct them. In this case we ideally want a way to visualize the location of the cropbox and use this as the initial point for debugging. 

One Approach to solve this issue is presented below.

## Debugging Approach

1. Fill a section of the 3D Space in Rviz with a cubic point cloud using The script cubic_point_cloud_publisher.py.

2. Provide this point cloud, to the node using PCL Cropbox.

3. If the section of Rviz contains your cropbox it should become visible in Rviz. If not adjust the `cubic_point_cloud_publisher.py.` to cover a different section of the 3D Space.



## Supported Platforms

These scripts were tested on Linux (specifically UBUNTU 20) with ROS NEOTIC
But they should be valid for other version of ROS1 which support PointCloud2 msg and other Linux Distro.
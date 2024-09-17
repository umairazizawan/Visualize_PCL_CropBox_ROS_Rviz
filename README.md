# Visualize_PCL_CropBox_ROS_Rviz
An approach to Debugging Transform issues when working with PCL CropBox in ROS. 

PCL Cropbox is meant to only retain the section of the point cloud which is inside its bounds. 
In case the coordinates and transformation of the Cropbox are not specified correctly, finding the actual location of the cropbox can present a challenge, especially when working with custom tf trees. 

One Debugging Approach for this issue is provided here.

## Debugging Approach

1. Fill a section of the 3D Space with a cubic point cloud using The script cubic_point_cloud_publisher.py.

2. Provide this point cloud, to the node containing PCL Cropbox function and find the actual position of CropBox in Rviz.



# Visualize_PCL_CropBox_ROS_Rviz
An approach to Debugging Transform issues when working on PCL CropBox with ROS. 

The Cropbox will only display the section of the point cloud which is inside its bounds. 
In case the coordinates of the Cropbox are not specified correctly finding the actual location of the cropbox can present a challenge, especially when working with custom tf trees. 

One Debugging Approahc is provided here.

## Debugging Approach

Fill a section of the 3D Space with a cubic point cloud.
Provide this point cloud to PCL Cropbox and find the actual position of CropBox in Rviz.

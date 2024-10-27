#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header


##############################  PARAMETERS  #######################################################
CUBE_SIZE               = 10
CUBE_SPACING            = 0.5

NODE_NAME               = 'point_cloud_publisher'
ROS_TOPIC_NAME          = '/point_cloud'

PUBLISH_RATE_HZ         = 1
ROS_TOPIC_QUEUE         = 10

MSG_FRAME_ID            = "map"

###################################################################################################
def generate_cubic_point_coordinates(cube_size, spacing):
    '''
    This function generates an array of points covering the coordinates of a cube.
    The cube will cover an equal length in positive and negative coordinates along all axes.
    To change the coordinates of the cube please vary the upper and lower bound on np.arrage used
    for the calculation x,y and z coordinates as required.
    Parameters
    ----------
    cube_size   : float
        Length of one side of a cube.
    spacing     : float
        Distance between points

    NOTE: Please be careful when adjusting these Parameters, as rendering a large point cloud points
    can slow down your machine.

    '''
    # Create a grid of equally spaced points within the space
    x = np.arange(-cube_size / 2, cube_size / 2, spacing)
    y = np.arange(-cube_size / 2, cube_size / 2, spacing)
    z = np.arange(-cube_size / 2, cube_size / 2, spacing)

    # Generate a 3D grid of points to get all possible combination/coordinates inside the space
    xv, yv, zv = np.meshgrid(x, y, z, indexing='xy')

    # Flatten the grid to a list of (x, y, z) points
    points = np.column_stack((xv.flatten(), yv.flatten(), zv.flatten()))

    return points.astype(np.float32)

###################################################################################################
def publish_cubic_point_cloud(pub):
    '''
    Publish a Cubic point cloud.
    Please adjust frame_id and timestamp and apply tf transformation if required.
    '''
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = MSG_FRAME_ID 

    # Generate points with a specified length and spacing
    points = generate_cubic_point_coordinates(CUBE_SIZE, CUBE_SPACING)

    point_cloud_msg = pc2.create_cloud_xyz32(header, points.tolist())

    pub.publish(point_cloud_msg)

###################################################################################################
def point_cloud_publisher():

    rospy.init_node(NODE_NAME, anonymous=True)

    pub = rospy.Publisher(ROS_TOPIC_NAME, PointCloud2, queue_size=ROS_TOPIC_QUEUE)

    rate = rospy.Rate(PUBLISH_RATE_HZ)

    while not rospy.is_shutdown():

        publish_cubic_point_cloud(pub)
        rate.sleep()

###################################################################################################
if __name__ == '__main__':
    try:
        point_cloud_publisher()
    except rospy.ROSInterruptException:
        pass

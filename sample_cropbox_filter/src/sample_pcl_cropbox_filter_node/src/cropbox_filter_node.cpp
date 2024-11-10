#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

class CropBoxFilterNode {
public:
    CropBoxFilterNode() {

        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        //Set bounding box limits
        min_x = -1.0;
        min_y = -1.0;
        min_z = -1.0;
        max_x =  1.0;
        max_y =  1.0;
        max_z =  1.0;


        point_cloud_sub_ = nh.subscribe("/point_cloud", 1, &CropBoxFilterNode::pointCloudCallback, this);
        filtered_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_point_cloud", 1);
    }

private:
    ros::Subscriber point_cloud_sub_;
    ros::Publisher filtered_cloud_pub_;

    // Parameters for CropBox limits
    double min_x, min_y, min_z, max_x, max_y, max_z;

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {

        // Convert the ROS PointCloud2 message to a PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

        // Apply CropBox filter
        pcl::CropBox<pcl::PointXYZ> crop_box;
        crop_box.setInputCloud(pcl_cloud);
        crop_box.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0));
        crop_box.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0));

        // Filter the point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        crop_box.filter(*cropped_cloud);

        // Convert the filtered PCL point cloud back to a ROS message
        sensor_msgs::PointCloud2 filtered_point_cloud;
        pcl::toROSMsg(*cropped_cloud, filtered_point_cloud);

        // Retain header of cloud_msg
        filtered_point_cloud.header = cloud_msg->header;

        // Publish the filtered cloud
        filtered_cloud_pub_.publish(filtered_point_cloud);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cropbox_filter_node");

    // Create the CropBoxFilterNode object
    CropBoxFilterNode node;

    ros::spin();

    return 0;
}

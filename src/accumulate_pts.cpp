
/* 
 * Aim: To approximate and vizualize tf coordinates of different sensors
 * (including lidar, IMU and, GPS antennas)
 *
 * Author: Dr. Saurab VERMA (saurab_verma@i2r.a-star.edu.sg)
 */

// Link libraries
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char **argv)
{

	// Initialize
	ros::init(argc, argv, "accumulate_pts");
	ros::NodeHandle pnh("~");
	std::string ros_node_name = ros::names::append(pnh.getNamespace(), "accumulate_pts");

	// User input
	std::string bag_name, topic_name, pcd_name;
	 pnh.param<std::string>("bag_name", bag_name, "input.bag");
	 pnh.param<std::string>("topic_name", topic_name, "/livox/lidar");
	 pnh.param<std::string>("pcd_name", pcd_name, "output.pcd");

	 // Open rosbag
	 rosbag::Bag bag;
	 bag.open(bag_name, rosbag::bagmode::Read);
	 rosbag::View view(bag, rosbag::TopicQuery(topic_name));

	 // Read all rosbag data
	 pcl::PointCloud<pcl::PointXYZI> acc_cloud;
	 for (const rosbag::MessageInstance &message : view)
	 {
		 // Convert ROS msgs to pcl
		 sensor_msgs::PointCloud2::ConstPtr pc2_msg = message.instantiate<sensor_msgs::PointCloud2>();
		 pcl::PointCloud<pcl::PointXYZI> cloud;
		 pcl::fromROSMsg(*pc2_msg, cloud);

		 // Accumulate all pointcloud data over multiple messages
		 acc_cloud += cloud;
	}

	// Store data
	pcl::io::savePCDFile(pcd_name, acc_cloud);
	ROS_INFO_STREAM("Accumulated pointcloud stored to " << pcd_name);
}

/* 
 * Aim: To approximate and vizualize tf coordinates of different sensors
 * (including lidar, IMU and, GPS antennas)
 *
 * Author: Dr. Saurab VERMA (saurab_verma@i2r.a-star.edu.sg)
 */

// Link libraries
#include <chrono>
#include <thread>
#include <fstream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char **argv)
{
	// Initialize
	ros::init(argc, argv, "extract_sensors");
	ros::NodeHandle pnh("~");
	std::string ros_node_name = ros::names::append(pnh.getNamespace(), "extract_sensors");

	// User input
	std::string pcd_name, in_csv_name, out_csv_name;
	pnh.param<std::string>("pcd_name", pcd_name, "av.pcd");
	pnh.param<std::string>("in_csv_name", in_csv_name, "abs_tf.csv");
	pnh.param<std::string>("out_csv_name", out_csv_name, "rel_tf.csv");

	// Read pcd
	pcl::PointCloud<pcl::PointXYZI>::Ptr av_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::io::loadPCDFile(pcd_name, *av_cloud);

	// Read csv
	std::string line;
	std::ifstream CsvFile;
	CsvFile.open(in_csv_name);
	if (!CsvFile)
	{
		ROS_FATAL_STREAM("Unable to open csv file" << in_csv_name);
		return 1;
	}
	unsigned int item_count = 0;
	while (getline(CsvFile, line))
	{
		// TODO: Show axes to sensors

		// TODO: Remove all points except those inside boxes
	}

	// Close csv
	CsvFile.close();

	// Create boxes around sensors and visualize
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(5.0, "lidar");
	viewer->initCameraParameters();
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> intensity_distribution(av_cloud, "intensity");
	viewer->addPointCloud<pcl::PointXYZI>(av_cloud, intensity_distribution, "pointcloud");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	// TODO: Compute and store relative tf
}
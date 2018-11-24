#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>


using namespace std;
using namespace cv;

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "cam_transform_publisher");
	ros::NodeHandle nh("~");
	ros::NodeHandle nh2;

	sensor_msgs::Image rgb_image;
	image_transport::ImageTransport it_;



}
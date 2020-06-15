#ifndef CALIBRATIONTOOL_H
#define CALIBRATIONTOOL_H

#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


//pcl
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/filters/voxel_grid.h>//滤波
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/crop_box.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/icp.h> //ICP(iterative closest point)配准

#include <pcl/features/fpfh_omp.h> //包含fpfh加速计算的omp(多核并行计算)
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //随机采样一致性去除
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

//STD C++ INCLUDES
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <sstream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>
#include <memory>
#include <dirent.h>

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// // rgbd_srv/rgbd.h
// #include "rgbd_srv/rgbd.h"

// tf
#include <tf/transform_listener.h>

//Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

//sophus
#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

// ceres
#include <ceres/ceres.h>


#endif // COMMON_INCLUDE_H


using namespace std;
using namespace cv;

namespace calibtool
{
enum {DETECTION = 0, CAPTURING = 1, CALIBRATION =2};

struct Extrinsic_Parameter
{
	Mat transform;
    Mat Rv;
	Mat t;
};

void TFToTransformMatrix(const tf::StampedTransform& tfpose, Eigen::Matrix4d& transform);

void loadTFPoses(const string& robot_pose_filename,
                 cv::Mat& tfPoses);

bool initCameraIntrinsic(const std::string& CAMERA_DATA_FILE, Mat& cameraMatrix, Mat& distCoeffs);

Mat rotVecFromTransform(Mat transform);

Mat transVecFromTransform(Mat transform);

Mat calcBoardCornerPositions(Size boardSize, float squareSize);


void deprojectPixelToPoint2(Eigen::Vector3f& point, int pixel_coloum, int pixel_row, float depth, cv::Mat& cameraMatrix, cv::Mat& distCoeffs);

void saveStampedImgPointcloudTFPose ( const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud, 
                const cv::Mat& image,
                const tf::StampedTransform& tfpose,
                const int& counter,
                const string& robot_pose_filename,
                const string& dataOutputDir);
                
void saveStampedImgPointcloud( const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud,
                               const cv::Mat& image,
                               const int& counter,
                               const string& dataOutputDir );
                               
void deprojectPixelToPoint(Eigen::Vector3f& point, int pixel_coloum, int pixel_row, float depth, cv::Mat& cameraMatrix, cv::Mat& distCoeffs);

void generateSphereFragment(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model, Eigen::Vector3f& center, float r, bool fragment = true);

vector<string> getFilesInDirectory ( const string& cate_dir, const string suffix);

void extractPlaneFeatureFromCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud_source,
                                  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud_destin);
                                                              
}
#endif // CALIBRATION_TOOL_H

#ifndef CALIBRATIONTOOL_H
#define CALIBRATIONTOOL_H

#include "CommonInclude.h"
#include "settings.h"


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

bool calculate_extrinsic(Mat image,Mat *transform,Mat object_point,Mat cameraMatrix,Mat distCoeffs,Settings s);

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
                                                              
}
#endif // CALIBRATION_TOOL_H

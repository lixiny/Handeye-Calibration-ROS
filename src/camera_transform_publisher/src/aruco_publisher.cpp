#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
//STD C++ INCLUDES
#include <iostream>
#include <cstring>

// ros
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// tf
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>

//Eigen
#include <Eigen/Core>

#include "aruco_plane.h"

using namespace std;
using namespace cv;

string camera_link;
string ar_marker;

bool initCameraIntrinsic(const std::string &CAMERA_DATA_FILE, Mat &cameraMatrix, Mat &distCoeffs)
{
    FileStorage fs(CAMERA_DATA_FILE, FileStorage::READ);
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coeffs"] >> distCoeffs;

    cout << "Camera instrinsic matrix" << endl;
    cout << cameraMatrix << endl;
    cout << "camera distortion coefficients" << endl;
    cout << distCoeffs << endl;

    fs.release();

    if (cameraMatrix.empty())
        return false;
    if (distCoeffs.empty())
        return false;

    return true;
}

class ArucoExtractor
{
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub_rgb;

    sensor_msgs::Image rgb_msg;
    campub::ArucoPlane *paruco;
    cv::Mat rgb;

    tf::TransformBroadcaster br;
    tf::Transform tranform;
    Eigen::Matrix4f Tc2m, Tm2c;

    string cameraTopic;

public:
    ArucoExtractor(
        ros::NodeHandle &nh_,
        campub::ArucoPlane *ap_,
        cv::Mat cameraMatrix_,
        cv::Mat distCoeffs_,
        string CameraTopic_) : it(nh_),
                               paruco(ap_),
                               cameraTopic(CameraTopic_)
    {
        image_sub_rgb = it.subscribe(cameraTopic, 1, &ArucoExtractor::imageCb_rgb, this);
    }

    void imageCb_rgb(const sensor_msgs::ImageConstPtr &msg)
    {
        rgb_msg = *msg;
        try
        {
            rgb = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8)->image;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge execption: %s", e.what());
            return;
        }

        bool validAruco = paruco->calculateExtrinsicFromImage(rgb);
        if (validAruco)
        {
            Tc2m = paruco->getTransform().cast<float>();
            Tm2c = Tc2m.inverse();
            paruco->drawingAxis(rgb);
            paruco->drawingCube(rgb);
            cv::putText(rgb,
                        "valid aruco plane found",
                        cv::Point(10, 40),              // Coordinates
                        cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
                        1.0,                            // Scale. 2.0 = 2x bigger
                        cv::Scalar(0, 255, 0),          // BGR Color
                        0                               // Line Thickness (Optional)
            );
        }
        else
        {
            cv::putText(rgb,
                        "no aruco plane found",
                        cv::Point(10, 40),              // Coordinates
                        cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
                        1.0,                            // Scale. 2.0 = 2x bigger
                        cv::Scalar(0, 0, 255),          // BGR Color
                        0                               // Line Thickness (Optional)
            );
            cv::imshow("aruco_publisher", rgb);
            cv::waitKey(1);

            tranform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
            tranform.setRotation(tf::Quaternion(0, 0, 0, 1));

            br.sendTransform(tf::StampedTransform(
                tranform, ros::Time::now(),
                camera_link,
                ar_marker));
            return;
        }

        tf::Vector3 origin;
        origin.setValue(Tc2m(0, 3),
                        Tc2m(1, 3),
                        Tc2m(2, 3));
        tf::Matrix3x3 tf3d;
        tf3d.setValue(Tc2m(0, 0), Tc2m(0, 1), Tc2m(0, 2),
                      Tc2m(1, 0), Tc2m(1, 1), Tc2m(1, 2),
                      Tc2m(2, 0), Tc2m(2, 1), Tc2m(2, 2));

        tf::Quaternion tfqt;
        tf3d.getRotation(tfqt);

        tranform.setOrigin(origin);
        tranform.setRotation(tfqt);

        br.sendTransform(tf::StampedTransform(tranform,
                                              ros::Time::now(),
                                              camera_link,
                                              ar_marker));

        cv::imshow("aruco_publisher", rgb);
        cv::waitKey(1);
    }
};

int main(int argc, char **argv)
{
    cout << "\n\n";
    ros::init(argc, argv, "aruco_publisher");
    ros::NodeHandle nh("~");

    string cameraIntrinsicInput;
    string cameraTopic;
    string rgbdServiceName;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    float tagSideLen;
    float planeSideLen;

    nh.param("cameraIntrinsicInput", cameraIntrinsicInput, std::string("./camera_intrinsic_color.xml"));
    nh.param("cameraTopic", cameraTopic, std::string("/camera/image"));
    nh.param("cameraLinkName", camera_link, std::string("/camera_link"));
    nh.param("arMarkerName", ar_marker, std::string("/ar_marker"));
    nh.param("tagSideLen", tagSideLen, 0.035f);
    nh.param("planeSideLen", planeSideLen, 0.25f);
    nh.param("rgbdServiceName", rgbdServiceName, std::string("realsense2_server"));

    cout << "Reading Camera Intrinsic File from : " << cameraIntrinsicInput << endl;

    if (!initCameraIntrinsic(cameraIntrinsicInput, cameraMatrix, distCoeffs))
    {
        cerr << "FATAL:  Cannot locate [camera_intrinsics] XML file Or [camera_intrinsics] XML file contains invalid data. Program will exit." << endl;
        return -1;
    }

    ros::NodeHandle nh2;

    campub::ArucoPlane aruco(tagSideLen, planeSideLen);
    aruco.setCameraIntrinsic(cameraMatrix, distCoeffs);

    ArucoExtractor ae(nh2, &aruco, cameraMatrix, distCoeffs, cameraTopic);
    ros::spin();

    return 0;
}
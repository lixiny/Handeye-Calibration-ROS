#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
//STD C++ INCLUDES
#include <iostream>
#include <cstring>

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

// tf
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Transform.h>

//Eigen
#include <Eigen/Core>

using namespace std;
using namespace cv;

string camera_link;
string ar_marker;

class ChessBoardExtractor
{

    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub_rgb;

    sensor_msgs::Image rgb_msg;

    tf::TransformBroadcaster br;
    tf::Transform transform;

    cv::Mat cornerPositions;
    cv::Size boardSize;
    float squareSize;

    cv::Mat rgb;
    cv::Mat gray;

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    string cameraTopic;

public:
    ChessBoardExtractor(
        ros::NodeHandle &nh_,
        cv::Size boardSize_,
        float squareSize_,
        cv::Mat cameraMatrix_,
        cv::Mat distCoeffs_,
        string CameraTopic_) : it(nh_),
                               boardSize(boardSize_),
                               squareSize(squareSize_),
                               cameraMatrix(cameraMatrix_),
                               distCoeffs(distCoeffs_),
                               cameraTopic(CameraTopic_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_rgb = it.subscribe(cameraTopic, 1, &ChessBoardExtractor::imageCb_rgb, this);
        cornerPositions = setBoardCornerPositions(boardSize, squareSize);
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

        vector<cv::Point2f> pointBuf;
        Eigen::Matrix4d transfCam2Marker;

        auto width = rgb.cols, height = rgb.rows;
        bool chessBoardFound = false;

        // cv::cvtColor(rgb, gray, COLOR_RGB2GRAY);
        chessBoardFound = cv::findChessboardCorners(rgb,
                                                    boardSize,
                                                    pointBuf,
                                                    CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

        if (!chessBoardFound)
        {
            cv::putText(rgb,
                        "no chessboard found",
                        cv::Point(10, 40),              // Coordinates
                        cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
                        1.0,                            // Scale. 2.0 = 2x bigger
                        cv::Scalar(0, 0, 255),          // BGR Color
                        0                               // Line Thickness (Optional)
            );
            cv::imshow("chessboard_publisher", rgb);
            cv::waitKey(1);

            transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
            transform.setRotation(tf::Quaternion(0, 0, 0, 1));

            br.sendTransform(tf::StampedTransform(transform,
                                                  ros::Time::now(),
                                                  camera_link,
                                                  ar_marker));
            return;
        }

        // // this line is slow
        // cv::cornerSubPix(
        //     gray,
        //     pointBuf,
        //     cv::Size(11,11),
        //     cv::Size(-1,-1),
        //     cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1)
        // );
        cv::drawChessboardCorners(rgb, boardSize, Mat(pointBuf), chessBoardFound);

        cv::putText(rgb,
                    "valid chessboard found",
                    cv::Point(10, 40),              // Coordinates
                    cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
                    1.0,                            // Scale. 2.0 = 2x bigger
                    cv::Scalar(0, 255, 0),          // BGR Color
                    0                               // Line Thickness (Optional)
        );
        cv::imshow("chessboard_publisher", rgb);
        cv::waitKey(1);

        Mat imagePoints(pointBuf.size(), 2, CV_32F);
        for (int i = 0; i < int(pointBuf.size()); i++)
        {
            imagePoints.at<float>(i, 0) = pointBuf[i].x;
            imagePoints.at<float>(i, 1) = pointBuf[i].y;
        }

        CvMat *cv_rotation_vector = cvCreateMat(3, 1, CV_32F);
        CvMat *cv_translation_vector = cvCreateMat(3, 1, CV_32F);
        CvMat cv_object_point = cornerPositions;
        CvMat cv_image_point = imagePoints;
        CvMat cv_camera_matrix = cameraMatrix;
        CvMat cv_dist_coeffs = distCoeffs;
        cvFindExtrinsicCameraParams2(&cv_object_point,
                                     &cv_image_point,
                                     &cv_camera_matrix,
                                     &cv_dist_coeffs,
                                     cv_rotation_vector,
                                     cv_translation_vector);

        float *rotation1 = (float *)cvPtr2D(cv_rotation_vector, 0, 0);
        float r1, r2, r3;
        r1 = *rotation1;
        rotation1++;
        r2 = *rotation1;
        rotation1++;
        r3 = *rotation1;
        float *translation1 = (float *)cvPtr2D(cv_translation_vector, 0, 0);
        float t1, t2, t3;
        t1 = *translation1;
        translation1++;
        t2 = *translation1;
        translation1++;
        t3 = *translation1;
        Mat rodri(3, 1, CV_64F);
        rodri.at<double>(0, 0) = r1;
        rodri.at<double>(1, 0) = r2;
        rodri.at<double>(2, 0) = r3;
        Mat dcm(3, 3, CV_64F);
        cv::Rodrigues(rodri, dcm);
        transfCam2Marker(0, 3) = t1;
        transfCam2Marker(1, 3) = t2;
        transfCam2Marker(2, 3) = t3;
        transfCam2Marker(3, 3) = 1.0;

        transfCam2Marker(3, 0) = 0;
        transfCam2Marker(3, 1) = 0;
        transfCam2Marker(3, 2) = 0;

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                transfCam2Marker(i, j) = dcm.at<double>(i, j);

        // Eigen::Matrix4d &Tc2m = transfCam2Marker;
        Eigen::Matrix4d Tc2m = transfCam2Marker;
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

        transform.setOrigin(origin);
        transform.setRotation(tfqt);

        br.sendTransform(tf::StampedTransform(transform,
                                              ros::Time::now(),
                                              camera_link,
                                              ar_marker));
    }

    cv::Mat setBoardCornerPositions(Size boardSize, float squareSize)
    {
        int width = boardSize.width;
        int height = boardSize.height;
        Mat corners(width * height, 3, CV_32F);
        for (int i = 0; i < boardSize.height; i++)
        {
            for (int j = 0; j < boardSize.width; j++)
            {
                corners.at<float>(i * boardSize.width + j, 0) = float(-j * squareSize);
                corners.at<float>(i * boardSize.width + j, 1) = float(i * squareSize);
                corners.at<float>(i * boardSize.width + j, 2) = 0.0f;
            }
        }
        return corners;
    }
};

class CameraInfo
{
public:
    cv::Mat intrinsic = cv::Mat::eye(3, 3, CV_32F);
    cv::Mat distortion = cv::Mat::zeros(1, 5, CV_32F);
    int readyflag = 0;

    CameraInfo() {}
    void cameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr &msg)
    {
        cout << "\e[1;34m"
             << "I received a camera_info."
             << "\e[0m" << endl;
        readyflag = 1;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                intrinsic.at<float>(i, j) = msg->K[3 * i + j];
        for (int j = 0; j < 5; j++)
            distortion.at<float>(0, j) = msg->D[j];
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "chessboard_publisher");
    ros::NodeHandle nh("~");

    string cameraTopic;
    string cameraInfoTopic;
    cv::Size boardSize;
    float squareSize;
    int chessboardWidth;
    int chessboardHeight;

    nh.param("chessboardWidth", chessboardWidth, 11);
    nh.param("chessboardHeight", chessboardHeight, 8);
    nh.param("squareSize", squareSize, 0.01f);
    nh.param("cameraTopic", cameraTopic, std::string("/realsense/rgb"));
    nh.param("cameraInfoTopic", cameraInfoTopic, std::string("/realsense/camera_info"));
    nh.param("cameraLinkName", camera_link, std::string("/camera_link"));
    nh.param("arMarkerName", ar_marker, std::string("/ar_marker"));

    ros::Subscriber camera_info_sub;
    ros::NodeHandle nci;
    CameraInfo ci;
    camera_info_sub = nci.subscribe(cameraInfoTopic, 1, &CameraInfo::cameraInfoCb, &ci);
    while (!ci.readyflag)
        ros::spinOnce();
    nci.shutdown();

    boardSize.width = chessboardWidth;
    boardSize.height = chessboardHeight;

    ros::NodeHandle nh2;
    ChessBoardExtractor ce(nh2, boardSize, squareSize, ci.intrinsic, ci.distortion, cameraTopic);
    cout << "\e[1;33m"
         << "Chessboard publisher is ready."
         << "\e[0m" << endl;

    ros::spin();
}

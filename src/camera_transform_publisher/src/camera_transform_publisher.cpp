#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
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
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>

//Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

using namespace std;
using namespace cv;


string camera_link;
string ar_marker;

bool statusChange = true;



class ChessBoardExtractor
{

    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub_rgb;

    sensor_msgs::Image rgb_image;

    tf::TransformBroadcaster br;
    tf::Transform transform_b;

    cv::Mat cornerPositions;
    cv::Size boardSize;
    float squareSize;

    cv::Mat rgb;
    cv::Mat rgbSmallSize;
    cv::Mat gray;

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    string cameraTopic;

public:

    ChessBoardExtractor(ros::NodeHandle& nh_, cv::Size boardSize_, float squareSize_, cv::Mat cameraMatrix_, cv::Mat distCoeffs_, string CameraTopic_)
        : it(nh_), boardSize(boardSize_), squareSize(squareSize_), cameraMatrix(cameraMatrix_), distCoeffs(distCoeffs_), cameraTopic(CameraTopic_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_rgb = it.subscribe(cameraTopic, 1, &ChessBoardExtractor::imageCb_rgb, this);
        cornerPositions = setBoardCornerPositions(boardSize, squareSize);
        // transform_b.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        // transform_b.setRotation( tf::Quaternion(0, 0, 0, 1) );
        // br.sendTransform(tf::StampedTransform(transform_b, ros::Time::now(),
        //                                           "camera_link", "ar_marker_0"));
    }

    ~ChessBoardExtractor()
    {
    }

    void imageCb_rgb(const sensor_msgs::ImageConstPtr& msg)
    {
        rgb_image = *msg;
        try{
            rgb = cv_bridge::toCvCopy ( rgb_image, sensor_msgs::image_encodings::BGR8 )->image;

        }   catch ( cv_bridge::Exception& e ) {
            ROS_ERROR ( "cv_bridge execption: %s", e.what() );
            return;
        }

        vector<cv::Point2f> pointBuftemp;
        vector<cv::Point2f> pointBuf;
        Eigen::Matrix4d transfCam2Marker;

        auto width = rgb.cols, height = rgb.rows;
        if(width > 640 || height > 480) {
	        float shrinkScale = width/640.0f;
	        height /= shrinkScale;
	        width = 640;
	        cv::resize(rgb, rgbSmallSize, cv::Size(height,width), INTER_LINEAR);
	        bool chessBoardFound1 = cv::findChessboardCorners(rgbSmallSize, boardSize, pointBuftemp, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
	        if ( ! chessBoardFound1 ) {
                cv::putText(rgb, 
                    "no chessboard found",
                    cv::Point(10,10), // Coordinates
                    cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
                    1.0, // Scale. 2.0 = 2x bigger
                    cv::Scalar(0,0,255), // BGR Color
                    1 // Line Thickness (Optional)
                ); 
	            cv::imshow("image", rgb);
	            cv::waitKey(1);
                
	            transform_b.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
	            transform_b.setRotation( tf::Quaternion(0, 0, 0, 1) );

	            br.sendTransform(tf::StampedTransform(transform_b, ros::Time::now(),
	                                                  camera_link,
	                                                  ar_marker
	                                                  ));
	            vector<cv::Point2f>().swap(pointBuftemp);
	            return;
	        }
        }
        
        cv::cvtColor(rgb, gray, COLOR_RGB2GRAY);
        bool chessBoardFound2 = cv::findChessboardCorners(rgb, boardSize,  pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
        cv::cornerSubPix( gray, pointBuf, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1) );
        cv::drawChessboardCorners(rgb, boardSize, Mat(pointBuf), chessBoardFound2);

        cv::putText(rgb, 
            "valid chessboard found",
            cv::Point(10,10), // Coordinates
            cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
            1.0, // Scale. 2.0 = 2x bigger
            cv::Scalar(0,255,0), // BGR Color
            1 // Line Thickness (Optional)
        ); 
        cv::imshow("image", rgb);
        cv::waitKey(1);

        Mat imagePoints(pointBuf.size(), 2, CV_32F);
        for(int i = 0; i < int(pointBuf.size()); i++) {
            imagePoints.at<float>(i, 0) = pointBuf[i].x;
            imagePoints.at<float>(i, 1) = pointBuf[i].y;
        }

        CvMat* cv_rotation_vector = cvCreateMat(3,1, CV_32F);
        CvMat* cv_translation_vector = cvCreateMat(3,1,CV_32F);
        CvMat cv_object_point = cornerPositions;
        CvMat cv_image_point = imagePoints;
        CvMat cv_camera_matrix = cameraMatrix;
        CvMat cv_dist_coeffs = distCoeffs;
        cvFindExtrinsicCameraParams2(  &cv_object_point,
                                       &cv_image_point,
                                       &cv_camera_matrix,
                                       &cv_dist_coeffs,
                                       cv_rotation_vector,
                                       cv_translation_vector);

        float *rotation1=(float*)cvPtr2D(cv_rotation_vector, 0, 0);
        float r1,r2,r3;
        r1=*rotation1;
        rotation1++;
        r2=*rotation1;
        rotation1++;
        r3=*rotation1;
        float *translation1=(float*)cvPtr2D(cv_translation_vector, 0, 0);
        float t1,t2,t3;
        t1=*translation1;
        translation1++;
        t2=*translation1;
        translation1++;
        t3=*translation1;
        Mat rodri(3,1,CV_64F);
        rodri.at<double>(0,0)=r1;
        rodri.at<double>(1,0)=r2;
        rodri.at<double>(2,0)=r3;
        Mat dcm(3,3,CV_64F);
        cv::Rodrigues(rodri,dcm);
        transfCam2Marker(0,3) = t1;
        transfCam2Marker(1,3) = t2;
        transfCam2Marker(2,3) = t3;
        transfCam2Marker(3,3) = 1.0;

        transfCam2Marker(3,0) = 0;
        transfCam2Marker(3,1) = 0;
        transfCam2Marker(3,2) = 0;

        for (int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                transfCam2Marker(i,j) = dcm.at<double>(i,j);
            }
        }

        // Eigen::Matrix4d &Tc2m = transfCam2Marker;
        Eigen::Matrix4d Tc2m = transfCam2Marker;
        tf::Vector3 origin;
        origin.setValue(static_cast<double>(Tc2m(0,3)),
                        static_cast<double>(Tc2m(1,3)),
                        static_cast<double>(Tc2m(2,3)));
        tf::Matrix3x3 tf3d;
        tf3d.setValue(static_cast<double>(Tc2m(0,0)), static_cast<double>(Tc2m(0,1)), static_cast<double>(Tc2m(0,2)),
                      static_cast<double>(Tc2m(1,0)), static_cast<double>(Tc2m(1,1)), static_cast<double>(Tc2m(1,2)),
                      static_cast<double>(Tc2m(2,0)), static_cast<double>(Tc2m(2,1)), static_cast<double>(Tc2m(2,2)));

        tf::Quaternion tfqt;
        tf3d.getRotation(tfqt);

        transform_b.setOrigin(origin);
        transform_b.setRotation(tfqt);

        br.sendTransform(tf::StampedTransform(transform_b,
                                              ros::Time::now(),
                                              camera_link,
                                              ar_marker
                                              ));
    }

    cv::Mat setBoardCornerPositions(Size boardSize, float squareSize){
        int width = boardSize.width;
        int height = boardSize.height;
        Mat corners(width * height, 3, CV_32F);
        for(int i = 0 ; i < boardSize.height; i++) {
            for(int j = 0; j < boardSize.width; j++) {
                corners.at<float>(i * boardSize.width + j, 0) = float( -  j * squareSize );
                corners.at<float>(i * boardSize.width + j, 1) = float( i * squareSize );
                corners.at<float>(i * boardSize.width + j, 2) = 0.0f;
            }
        }
        return corners;
    }


};

bool initCameraIntrinsic(const std::string& CAMERA_DATA_FILE, Mat& cameraMatrix, Mat& distCoeffs)
{
    FileStorage fs(CAMERA_DATA_FILE, FileStorage::READ);
    fs["camera_matrix"]>>cameraMatrix;
    fs["distortion_coeffs"]>>distCoeffs;

    cout<<"Camera instrinsic matrix"<<endl;
    cout<<cameraMatrix<<endl;
    cout<<"camera distortion coefficients"<<endl;
    cout<<distCoeffs<<endl;

    fs.release();

    if(cameraMatrix.empty()) return false;
    if(distCoeffs.empty()) return false;

    return true;
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "cam_transform_publisher_l");
    ros::NodeHandle nh("~");

    string cameraIntrinsicInput;
    string cameraTopic;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Size boardSize;
    float squareSize;
    int chessboardWidth;
    int chessboardHeight;


    nh.param("cameraIntrinsicInput", cameraIntrinsicInput, std::string("./camera_intrinsic_color.xml") );
    nh.param("chessboardWidth", chessboardWidth, 11);
    nh.param("chessboardHeight", chessboardHeight, 8);
    nh.param("squareSize",  squareSize, 0.01f);
    nh.param("cameraTopic", cameraTopic, std::string("/camera/image"));
    nh.param("cameraLinkName", camera_link, std::string("/camera_link"));
    nh.param("arMarkerName", ar_marker, std::string("/ar_marker"));

    if( ! initCameraIntrinsic(cameraIntrinsicInput, cameraMatrix, distCoeffs)){
        cerr << "FATAL:  Cannot locate [camera_intrinsics] XML file Or"
                " [camera_intrinsics] XML file contains invalid data. Program will exit." << endl;
        return -1;
    }

    cout << "\n******  Using CHESSBOARD with --size " << chessboardWidth << " x " << chessboardHeight << " --square " << squareSize << "\n\n";
    boardSize.width = chessboardWidth;
    boardSize.height = chessboardHeight;


    ros::NodeHandle nh2;
    ChessBoardExtractor ce( nh2, boardSize, squareSize, cameraMatrix, distCoeffs, cameraTopic);

    ros::spin();

}


#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

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

#define KEYCODE_SPACE   0x20
#define KEYCODE_ESC     0x1B

string  robotPoseOutput;
string  EETFname;
string  baseTFname;
bool    recordingTF;


// ################################### function prototype ############################################

int kbhit(void);
int getch();

// ################################### main function ###########################################
int main(int argc, char** argv)
{
    cout << "\n\n";

    ros::init ( argc, argv, "grab_tf_node" );
    ros::NodeHandle nh("~");

    nh.param("robotPoseOutput" , robotPoseOutput, std::string("./robot_pose.xml"));
    nh.param("EETF", EETFname, std::string("/ee_link"));
    nh.param("baseTF", baseTFname, std::string("/base_link"));

    cout << "robotPoseOutput " << robotPoseOutput << endl;

    int fileSavedCounter = 0;

    tf::TransformListener tfListener;
    tf::StampedTransform  tfTransform;

    ros::Rate loop_rate ( 30 );

    FileStorage fswrite(robotPoseOutput, FileStorage::WRITE);

    vector<cv::Mat> allTFPose;

    while( ros::ok() )
    {
        try
        {
            /** get the latest tfTransform from robot_base_link(source) to robot_end_link(target) */
            tfListener.lookupTransform (baseTFname,   EETFname,  ros::Time ( 0 ), tfTransform );

        } catch ( tf::TransformException ex ) {
            ROS_ERROR("%s", ex.what());
            cout << "Warning: Current may not have tf data! " << endl;
            loop_rate.sleep();
            continue;
        }



       if( kbhit())   //space down
       {
           char c = getchar();
           if(c == KEYCODE_ESC)
           {
                cout<<"\n Finishing grab tf data ..."<<endl;
                Mat robot_poses(fileSavedCounter, 7, CV_64F); 

                for (int i = 0; i < fileSavedCounter; i++) {
                    for (int j = 0; j < 7; j++) {
                        robot_poses.at<double>(i,j) = allTFPose[i].at<double>(0,j);
                    }
                }

                fswrite << "robotposes" << robot_poses;
                fswrite.release();
                ros::shutdown();
                break;
           } else {
                
                cout<<"............... saving tf data.............." << endl;
                Mat Mat_tfpose_1_7(1,7,CV_64F);  // row:1 coloum:7


                tf::Quaternion quat ( 0,0,0,0 );
                tf::Vector3 transl ( 0,0,0 );
                // tf::Vector3 so3 (0,0,0);

                /** tfTransform contains information of rotarion(as tf::Quaternion) and trasnlation(as tf::Vector3) */
                quat = tfTransform.getRotation();
                transl = tfTransform.getOrigin();
                // float rotation_angle = q.getAngle();   //[0,2Pi]
                // tf::Vector3 rotation_axis = q.getAxis();
                // so3 = rotation_axis * rotation_angle;
                Mat_tfpose_1_7.at<double>(0,0) = transl.x();
                Mat_tfpose_1_7.at<double>(0,1) = transl.y();
                Mat_tfpose_1_7.at<double>(0,2) = transl.z();
                Mat_tfpose_1_7.at<double>(0,3) = quat.x();
                Mat_tfpose_1_7.at<double>(0,4) = quat.y();
                Mat_tfpose_1_7.at<double>(0,5) = quat.z();
                Mat_tfpose_1_7.at<double>(0,6) = quat.w();

                cout << "tf: "<< fileSavedCounter <<" /" <<  baseTFname << " to /" << EETFname << " is: [x, y, z, qx, qy, qz, qw] : \n"  << Mat_tfpose_1_7 << endl;
                allTFPose.push_back(Mat_tfpose_1_7);
                fileSavedCounter++;
           }

       }

       loop_rate.sleep();
        // ros::spinOnce();
        
    }

    return 0;

}


int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if(ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

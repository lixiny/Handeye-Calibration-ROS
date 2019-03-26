



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

#include "sophus/so3.h"
#include "sophus/se3.h"

#include <ceres/ceres.h>
#include "ceres_extensions.h"

using namespace std;
using namespace cv;

void loadTFPoses(const string& robot_pose_filename,
                 cv::Mat& tfPoses)
{
    FileStorage fsread(robot_pose_filename, FileStorage::READ);
    fsread["robotposes"] >> tfPoses;
    fsread.release();
}


struct ReprojectionError3D {

    ReprojectionError3D(const Eigen::Vector3d _PInCamera_a, // spherecenter Point viewd by camera position i
                        const Eigen::Vector3d _PInCamera_b,
                        const Eigen::Isometry3d _Tfbg_a,
                        const Eigen::Isometry3d _Tfbg_b):
        PInCamera_a(_PInCamera_a), PInCamera_b(_PInCamera_b), Tfbg_a(_Tfbg_a), Tfbg_b(_Tfbg_b)
    {

    }

    template <typename T>
    bool operator()(const T* const handeye_rotation, const T* const handeye_translation, T* residuals) const {

        T qw = T(handeye_rotation[0]); //handeye w
        T qx = T(handeye_rotation[1]);
        T qy = T(handeye_rotation[2]);
        T qz = T(handeye_rotation[3]);

        T tx = T(handeye_translation[0]);
        T ty = T(handeye_translation[1]);
        T tz = T(handeye_translation[2]);

        double tcx, tcy, tcz;  //sphere center in camera
        double qiw, qix, qiy, qiz;  // quanterions of Rbg;
        double tix, tiy, tiz;   // translation of tbg;


        // ## For data# a:
        tcx = PInCamera_a.x();
        tcy = PInCamera_a.y();
        tcz = PInCamera_a.z();
        auto quantbg_a = Eigen::Quaterniond(Tfbg_a.rotation());
        qiw = quantbg_a.w();
        qix = quantbg_a.x();
        qiy = quantbg_a.y();
        qiz = quantbg_a.z();
        tix = Tfbg_a.translation().x();
        tiy = Tfbg_a.translation().y();
        tiz = Tfbg_a.translation().z();

        T Pinbase_aw = T ( qix*(qix*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) + qiw*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) - qiz*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) + qiy*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz))) - qiw*(qix*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) - qiw*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) + qiy*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) + qiz*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz))) + qiy*(qiy*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) + qiz*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) + qiw*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) - qix*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz))) + qiz*(qiz*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) - qiy*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) + qix*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) + qiw*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz)))
                           );
        T Pinbase_ax = T (tix + qiw*(qix*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) + qiw*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) - qiz*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) + qiy*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz))) + qix*(qix*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) - qiw*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) + qiy*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) + qiz*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz))) + qiy*(qiz*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) - qiy*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) + qix*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) + qiw*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz))) - qiz*(qiy*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) + qiz*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) + qiw*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) - qix*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz)))
                          );
        T Pinbase_ay = T (tiy + qiw*(qiy*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) + qiz*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) + qiw*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) - qix*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz))) - qix*(qiz*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) - qiy*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) + qix*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) + qiw*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz))) + qiy*(qix*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) - qiw*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) + qiy*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) + qiz*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz))) + qiz*(qix*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) + qiw*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) - qiz*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) + qiy*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz)))
                          );
        T Pinbase_az = T (tiz + qiw*(qiz*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) - qiy*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) + qix*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) + qiw*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz))) + qix*(qiy*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) + qiz*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) + qiw*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) - qix*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz))) - qiy*(qix*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) + qiw*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) - qiz*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) + qiy*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz))) + qiz*(qix*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) - qiw*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) + qiy*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) + qiz*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz)))
                          );

       // ## For data# b:
        tcx = PInCamera_b.x();
        tcy = PInCamera_b.y();
        tcz = PInCamera_b.z();
        auto quantbg_b = Eigen::Quaterniond(Tfbg_b.rotation());
        qiw = quantbg_b.w();
        qix = quantbg_b.x();
        qiy = quantbg_b.y();
        qiz = quantbg_b.z();
        tix = Tfbg_b.translation().x();
        tiy = Tfbg_b.translation().y();
        tiz = Tfbg_b.translation().z();

        T Pinbase_bw = T ( qix*(qix*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) + qiw*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) - qiz*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) + qiy*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz))) - qiw*(qix*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) - qiw*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) + qiy*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) + qiz*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz))) + qiy*(qiy*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) + qiz*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) + qiw*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) - qix*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz))) + qiz*(qiz*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) - qiy*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) + qix*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) + qiw*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz)))
                           );
        T Pinbase_bx = T (tix + qiw*(qix*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) + qiw*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) - qiz*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) + qiy*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz))) + qix*(qix*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) - qiw*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) + qiy*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) + qiz*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz))) + qiy*(qiz*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) - qiy*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) + qix*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) + qiw*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz))) - qiz*(qiy*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) + qiz*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) + qiw*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) - qix*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz)))
                          );
        T Pinbase_by = T (tiy + qiw*(qiy*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) + qiz*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) + qiw*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) - qix*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz))) - qix*(qiz*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) - qiy*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) + qix*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) + qiw*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz))) + qiy*(qix*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) - qiw*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) + qiy*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) + qiz*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz))) + qiz*(qix*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) + qiw*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) - qiz*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) + qiy*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz)))
                          );
        T Pinbase_bz = T (tiz + qiw*(qiz*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) - qiy*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) + qix*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) + qiw*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz))) + qix*(qiy*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) + qiz*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) + qiw*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) - qix*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz))) - qiy*(qix*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) + qiw*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) - qiz*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) + qiy*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz))) + qiz*(qix*(tx + qw*(qw*tcx + qy*tcz - qz*tcy) + qy*(qw*tcz + qx*tcy - qy*tcx) + qx*(qx*tcx + qy*tcy + qz*tcz) - qz*(qw*tcy - qx*tcz + qz*tcx)) - qiw*(qx*(qw*tcx + qy*tcz - qz*tcy) - qw*(qx*tcx + qy*tcy + qz*tcz) + qy*(qw*tcy - qx*tcz + qz*tcx) + qz*(qw*tcz + qx*tcy - qy*tcx)) + qiy*(ty + qw*(qw*tcy - qx*tcz + qz*tcx) - qx*(qw*tcz + qx*tcy - qy*tcx) + qy*(qx*tcx + qy*tcy + qz*tcz) + qz*(qw*tcx + qy*tcz - qz*tcy)) + qiz*(tz + qw*(qw*tcz + qx*tcy - qy*tcx) + qx*(qw*tcy - qx*tcz + qz*tcx) - qy*(qw*tcx + qy*tcz - qz*tcy) + qz*(qx*tcx + qy*tcy + qz*tcz)))
                          );

        residuals[0] = Pinbase_aw - Pinbase_bw;
        residuals[1] = Pinbase_ax - Pinbase_bx;
        residuals[2] = Pinbase_ay - Pinbase_by;
        residuals[3] = Pinbase_az - Pinbase_bz;

        return true;
    }


    const Eigen::Vector3d PInCamera_a, PInCamera_b;
    const Eigen::Isometry3d Tfbg_a, Tfbg_b;




};



int main(int argc, char** argv){
    cout << "hello world" << endl;
    string robot_pose_file = "./test/testdata/robot_pose.xml";
    cv::Mat tfBaseGrippers;
    loadTFPoses(robot_pose_file, tfBaseGrippers);

    vector<Eigen::Quaterniond> vQuantbg;
    vector<Eigen::Vector3d> vTranslbg;
    vector<Eigen::Isometry3d> vTbg;
    vector<Eigen::Vector3d> vSphereCenterInCamera;


    // -0.0426610901036  0.0435682041765  0.0628709556475  -0.0213248179092  0.00884362796331  -0.893675406915  0.44811974902
    Eigen::Quaterniond quantgc;
    quantgc.x() = -0.0213248179092;
    quantgc.y() = 0.00884362796331;
    quantgc.z() = -0.893675406915;
    quantgc.w() = 0.44811974902;
    Eigen::Vector3d translgc(-0.0426610901036,  0.0435682041765,  0.0628709556475);

    Eigen::Isometry3d Tgc = Eigen::Isometry3d::Identity();
    Tgc.rotate(quantgc);
    Tgc.pretranslate(translgc);
    cout << "Transform gripper to camera (groundtruth): \n" << Tgc.matrix() << endl;

    auto rotgc = Tgc.rotation();
    translgc = Tgc.translation();
    quantgc = Eigen::Quaterniond(rotgc);
    cout << "in tf fromat:x y z qx qy qz qw is : ";
    cout << translgc.transpose() << " " << quantgc.coeffs().transpose() << endl;


    Eigen::Quaterniond quantgc_unknown;
    quantgc_unknown.x() = -0.0213248179092;
    quantgc_unknown.y() = 0.00884362796331;
    quantgc_unknown.z() = -0.893675406915;
    quantgc_unknown.w() = 0.44811974902;
    Eigen::Vector3d translgc_unknow(-0.05,  0.12,  0.1);

    auto q_init = quantgc_unknown;
    auto t_init = translgc_unknow;

    Eigen::Isometry3d Tgc_unknown = Eigen::Isometry3d::Identity();
    Tgc_unknown.rotate(quantgc_unknown);
    Tgc_unknown.pretranslate(translgc_unknow);
    cout << "Transform gripper to camera (before optim): \n" << Tgc_unknown.matrix() << endl;

    int nPose = tfBaseGrippers.rows;

    for(int i = 0; i < nPose; i++ ) {
        cv::Mat& tf = tfBaseGrippers;
        Eigen::Vector3d translbgi(tf.at<double>(i,0), tf.at<double>(i,1), tf.at<double>(i,2));
        Eigen::Quaterniond quantbgi;

        quantbgi.x() = tf.at<double>(i,3);
        quantbgi.y() = tf.at<double>(i,4);
        quantbgi.z() = tf.at<double>(i,5);
        quantbgi.w() = tf.at<double>(i,6);

        vTranslbg.push_back(translbgi);
        vQuantbg.push_back(quantbgi);

        Eigen::Isometry3d Tbgi = Eigen::Isometry3d::Identity();
        Tbgi.rotate(quantbgi);
        Tbgi.pretranslate(translbgi);
        vTbg.push_back(Tbgi);
    }

    Eigen::Vector3d sphereCenterInB(1.5, 1.5, 1.5);


    for(int i = 0; i < nPose; i++) {

        Eigen::Isometry3d Tbci = vTbg[i] * Tgc;
        Eigen::Isometry3d Tcbi = Tbci.inverse();

        Eigen::Vector3d sphereCenterInCi = Tcbi * sphereCenterInB;
        //        cout << "sphere center in camera: " << sphereCenterInCi.transpose()<< endl;
        vSphereCenterInCamera.push_back(sphereCenterInCi);

    }

    for(int i = 0; i < nPose; i++) {
        Eigen::Isometry3d Tbci = vTbg[i] * Tgc;
        Eigen::Vector3d pb = Tbci * vSphereCenterInCamera[i];
        Eigen::Vector3d delta = pb - sphereCenterInB;
        cout << "Error between [transformed point from camera to base] - [point in base]: (should be 0):  " << delta.transpose() << endl;
    }


    ceres::Problem problem;

    double qx_handeye[4] = {q_init.w(), q_init.x(), q_init.y(), q_init.z()};
    double tx_handeye[3] = {t_init.x(), t_init.y(), t_init.z()};


    for(int i = 0; i < nPose; i++) {
        for(int j = i+1; j < nPose; j++) {

            problem.AddResidualBlock(
                        new::ceres::AutoDiffCostFunction<ReprojectionError3D, 4, 4, 3>(
                            new ReprojectionError3D(vSphereCenterInCamera[0], vSphereCenterInCamera[1], vTbg[0], vTbg[1])),
                    NULL,
                    qx_handeye,
                    tx_handeye);

        }
    }


    ceres::Solver::Options options;
    options.use_nonmonotonic_steps = true;
    options.preconditioner_type = ceres::SCHUR_JACOBI;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_type = ceres::TRUST_REGION;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.num_threads = 4;
    options.max_num_iterations = 1000;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;

    cout<<summary.BriefReport() <<endl;

    cout << "real value      in tf fromat:x y z qx qy qz qw is : ";
    cout << translgc.transpose() << " " << quantgc.coeffs().transpose() << endl;

    cout << "init value      in tf fromat:x y z qx qy qz qw is : ";
    cout << t_init.transpose() << " " << q_init.coeffs().transpose() << endl;

    cout << "estm value      in tf fromat:x y z qx qy qz qw is : ";
    cout << translgc_unknow.transpose() << " " << quantgc_unknown.coeffs().transpose() << endl;



    return 0;

}

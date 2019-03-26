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


class ReprojectionError3D {
public:
    ReprojectionError3D(const Eigen::Vector3d _PInCamerai, // spherecenter Point viewd by camera position i
                        const Eigen::Vector3d _PInCameraj,
                        const Eigen::Isometry3d _Tfbgi,
                        const Eigen::Isometry3d _Tfbgj):
        PInCamerai(_PInCamerai), PInCameraj(_PInCameraj), Tfbgi(_Tfbgi), Tfbgj(_Tfbgj)
    {}

    template <typename T>
    bool operator()(const T* const handeye_rotation, const T* const handeye_translation, T* residuals) const {


        Eigen::Quaternion<T> q_handeye = Eigen::Map<const Eigen::Quaternion<T>>(handeye_rotation);
        Eigen::Matrix<T,3,1> t_handeye = Eigen::Map<const Eigen::Matrix<T,3,1>>(handeye_translation);


        // Make sure the Eigen::Vector world point is using the ceres::Jet type as it's Scalar type !!!
        Eigen::Matrix<T,3,1> pointInCamerai;
        pointInCamerai << T(PInCamerai.x()), T(PInCamerai.y()), T(PInCamerai.z());
        /** Transform pointInCamra to pointInGripper for position i */

        // Rotate the point using Eigen rotations
        // cout << "pointInCamerai :" << T(pointInCamerai[0]) << " " << T(pointInCamerai[1]) << " " << T(pointInCamerai[2]) << endl;
        Eigen::Matrix<T,3,1> pointInGripperi = q_handeye * pointInCamerai; // unfinished !!!
        // Map T* to Eigen Vector3 with correct Scalar type
        pointInGripperi += t_handeye;


        /** Transform pointInGripper to pointInBase for position i */
        auto rotbgi = Tfbgi.rotation();
        auto translbgi = Tfbgi.translation();
        Eigen::Quaterniond qbgi(rotbgi);
        Eigen::Quaternion<T> q_bgi;
        q_bgi.w() = T(qbgi.w());    q_bgi.x() = T(qbgi.x());   q_bgi.y() = T(qbgi.y());   q_bgi.z() = T(qbgi.z());
        Eigen::Matrix<T,3,1> t_bgi;
        t_bgi << T(translbgi.x()) , T(translbgi.y()), T(translbgi.z());

        Eigen::Matrix<T,3,1> pointInBasei = q_bgi * pointInGripperi;  // unfinished !!
        pointInBasei += t_bgi;


        /** ##########################################################################################################################*/
        Eigen::Matrix<T,3,1> pointInCameraj;
        pointInCameraj << T(PInCameraj.x()), T(PInCameraj.y()), T(PInCameraj.z());

        /** Transform pointInCamra to pointInGripper for position j */
        Eigen::Matrix<T,3,1> pointInGripperj = q_handeye * pointInCameraj; // unfinished !!!
        pointInGripperj += t_handeye;

        /** Transform pointInGripper to pointInBase for position j */
        auto rotbgj = Tfbgj.rotation();
        auto translbgj = Tfbgj.translation();
        Eigen::Quaterniond qbgj(rotbgj);
        Eigen::Quaternion<T> q_bgj;
        q_bgj.w() = T(qbgj.w());    q_bgj.x() = T(qbgj.x());   q_bgj.y() = T(qbgj.y());   q_bgj.z() = T(qbgj.z());
        Eigen::Matrix<T,3,1> t_bgj;
        t_bgj << T(translbgj.x()) , T(translbgj.y()), T(translbgj.z());

        Eigen::Matrix<T,3,1> pointInBasej = q_bgj * pointInGripperj;  // unfinished !!
        pointInBasej += t_bgj;

        residuals[0] = T(T(pointInBasei[0]) - T(pointInBasej[0]));  //   deltaX = Xi - Xj
        residuals[1] = T(T(pointInBasei[1]) - T(pointInBasej[1]));
        residuals[2] = T(T(pointInBasei[2]) - T(pointInBasej[2]));

        return true;
    }


    const Eigen::Vector3d PInCamerai, PInCameraj;
    const Eigen::Isometry3d Tfbgi, Tfbgj;

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

    Eigen::AngleAxisd delta_rot(M_PI/6, Eigen::Vector3d(1.2,7.9,-10));
    Eigen::Vector3d delta_t(3.35, 0.35, 0.50);  // 沿x,y,z轴平移 0.15,0.15,0.1 米
    Eigen::Isometry3d Tdelta = Eigen::Isometry3d::Identity();
    Tdelta.rotate(delta_rot);
    Tdelta.pretranslate(delta_t);

    auto Tgc_unknown = Tgc * Tdelta;
    auto quantgc_unknown = Eigen::Quaterniond(Tgc_unknown.rotation());
    auto translgc_unknow = Tgc_unknown.translation();

    // deep copy the init value for comparision.
    auto q_init = Eigen::Quaterniond(quantgc_unknown);
    auto t_init = Eigen::Vector3d(translgc_unknow);


    int nPose = tfBaseGrippers.rows;

    // 读取
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

    // 定义球心
    Eigen::Vector3d sphereCenterInB(1.5, 1.5, 1.5);


    double w_sigma = 0.01;
    cv::RNG rng;

    for(int i = 0; i < nPose; i++) {

        Eigen::Isometry3d Tbci = vTbg[i] * Tgc;
        Eigen::Isometry3d Tcbi = Tbci.inverse();

        Eigen::Vector3d sphereCenterInCi = Tcbi * sphereCenterInB;

        // add noise on sphereCenter to simulate real test.
        Eigen::Vector3d daltaCenter(rng.gaussian(w_sigma), rng.gaussian(w_sigma), rng.gaussian(w_sigma));
        cout << "adding purturb to the viewd sphere center: delta:" << daltaCenter.transpose() << endl;
        sphereCenterInCi += daltaCenter;
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


    // Use my LocalParameterization
    ceres::LocalParameterization *quaternion_parameterization = new ceres_ext::EigenQuaternionParameterization();

    int counter = 0;
    for(int i = 0; i < nPose; i++) {
        for(int j = i+1; j < nPose; j++) {
            problem.AddResidualBlock(
                        new::ceres::AutoDiffCostFunction<ReprojectionError3D, 3, 4, 3>(
                            new ReprojectionError3D(vSphereCenterInCamera[i], vSphereCenterInCamera[j], vTbg[i], vTbg[j])),
                        NULL,
                        quantgc_unknown.coeffs().data(),
                        translgc_unknow.data());
            counter++;

        }
    }

    cout << "Total (C_" << nPose << "_2): " << counter << " number of error edges added! " << endl;



    problem.SetParameterization(quantgc_unknown.coeffs().data(), quaternion_parameterization );


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

    cout<<summary.BriefReport() <<"\n\n\n";


    cout << "##################  HANDEYE is transformation from ee_link to camera_link \n" <<
            "##################  or tf: (/ee_link,  /camera_link) \n" <<
            "##################  or father: /ee_link,  child: /camera_link \n" <<
            "##################  or P3d(in ee_link) = T_handeye * P3d(in camera_link) \n\n";

    cout << "real value      in tf fromat:(x y z qx qy qz qw) is : ";
    cout << translgc.transpose() << " " << quantgc.coeffs().transpose() << endl;

    cout << "init value      in tf fromat:(x y z qx qy qz qw) is : ";
    cout << t_init.transpose() << " " << q_init.coeffs().transpose() << endl;

    cout << "estm value      in tf fromat:(x y z qx qy qz qw) is : ";
    cout << translgc_unknow.transpose() << " " << quantgc_unknown.coeffs().transpose() << endl;

    ros::init(argc, argv, "handeye_tf_publisher");
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;
    tf::Transform transform_ee_cam;

    tf::Vector3 origin;
    origin.setValue(translgc_unknow.x(), translgc_unknow.y(), translgc_unknow.z());
    tf::Quaternion tfqt;
    tfqt.setW(quantgc_unknown.w());
    tfqt.setX(quantgc_unknown.x());
    tfqt.setY(quantgc_unknown.y());
    tfqt.setZ(quantgc_unknown.z());

    transform_ee_cam.setOrigin(origin);
    transform_ee_cam.setRotation(tfqt);

    ros::Rate loopRate(100);
    while(ros::ok()) {
        br.sendTransform(tf::StampedTransform(transform_ee_cam,
                                              ros::Time::now(),
                                              "ee_link",
                                              "camera_link_result"
                                            ));
        loopRate.sleep();
    }


    return 0;

}

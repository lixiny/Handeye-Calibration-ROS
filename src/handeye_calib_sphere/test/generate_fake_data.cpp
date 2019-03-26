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

const int lamda = 1000000;
#define random(a,b) (rand()%(b-a+1)+a)
const double RANSACthreshold = 0.0005;

/**
x: 10 unknows: [  (qx0, qx1, qx2, qx3)  (tx1, tx2, tx3)  (tb1, tb2, tb3)  ]
Fi = [f1(x), f2(x), f3(3), f4(x)]
F = [ F1,F2,F3......, PENALTY ]
*/

//#########################################################################################
double sigma1(const Eigen::Vector4d& qi) {
    return qi(0) * qi(0) - qi(1) * qi(1) - qi(2) * qi(2) + qi(3) * qi(3);
}

double sigma2(const Eigen::Vector4d& qi) {
    return qi(0) * qi(0) - qi(1) * qi(1) + qi(2) * qi(2) - qi(3) * qi(3);
}

double sigma3(const Eigen::Vector4d& qi) {
    return qi(0) * qi(0) + qi(1) * qi(1) - qi(2) * qi(2) - qi(3) * qi(3);
}

double sigma4(const Eigen::Vector4d& qi) {
    return 2 * qi(0) * qi(3) - 2 * qi(1) * qi(2);
}

double sigma5(const Eigen::Vector4d& qi) {
    return 2 * qi(0) * qi(2) - 2 * qi(1) * qi(3);
}

double sigma6(const Eigen::Vector4d& qi) {
    return 2 * qi(0) * qi(1) - 2 * qi(2) * qi(3);
}

double sigma7(const Eigen::Vector4d& qi) {
    return 2 * qi(0) * qi(3) + 2 * qi(1) * qi(2);
}

double sigma8(const Eigen::Vector4d& qi) {
    return 2 * qi(0) * qi(2) + 2 * qi(1) * qi(3);
}

double sigma9(const Eigen::Vector4d& qi) {
    return 2 * qi(0) * qi(1) + 2 * qi(2) * qi(3);
}
//#########################################################################################

struct f1 {
    f1 (Eigen::Vector4d ti, Eigen::Vector4d tj, Eigen::Vector4d qi) :
        _ti(ti), _tj(tj), _qi(qi) {}

    template <typename T>
    bool operator() (const T* const x,   // arguments, 10 dimension
                     T* residual) const  // residual error
    {
        T qx[4] = {x[0], x[1], x[2], x[3]};
        T tx[4] = {T(0.0), x[4], x[5], x[6]};
        T tb[4] = {T(0.0), x[7], x[8], x[9]};

        residual[0] = T(
                    - qx[1] * (_ti[1] - _tj[1]) - qx[2] * (_ti[2] - _tj[2]) - qx[3] * (_ti[3] - _tj[3])
                - qx[1] * tx[1] - qx[2] * tx[2] - qx[3] * tx[3]
                - tb[1] * (qx[1] * sigma3(_qi) + qx[2] * sigma7(_qi) - qx[3] * sigma5(_qi))
                - tb[2] * (qx[2] * sigma2(_qi) - qx[1] * sigma4(_qi) + qx[3] * sigma9(_qi))
                - tb[3] * (qx[3] * sigma1(_qi) + qx[1] * sigma8(_qi) - qx[2] * sigma6(_qi))
                );

        return true;
    }

    const Eigen::Vector4d _ti, _tj, _qi;
};


struct f2 {
    f2 (Eigen::Vector4d ti, Eigen::Vector4d tj, Eigen::Vector4d qi) :
        _ti(ti), _tj(tj), _qi(qi) {}

    template <typename T>
    bool operator() (const T* const x,  // arguments, 10 dimension
                     T* residual) const // residual error
    {
        T qx[4] = {x[0], x[1], x[2], x[3]};
        T tx[4] = {T(0.0), x[4], x[5], x[6]};
        T tb[4] = {T(0.0), x[7], x[8], x[9]};

        residual[0] = T(
                    qx[0] * (_ti[1] - _tj[1]) + qx[0] * tx[1] - qx[2] * tx[3] + qx[3] * tx[2]
                - qx[3] * (_ti[2] + _tj[2]) + qx[2] * (_ti[3] + _tj[3])
                - tb[1] * (qx[2] * sigma5(_qi) - qx[0] * sigma3(_qi) + qx[3] * sigma7(_qi))
                - tb[2] * (qx[3] * sigma2(_qi) + qx[0] * sigma4(_qi) - qx[2] * sigma9(_qi))
                + tb[3] * (qx[2] * sigma1(_qi) + qx[0] * sigma8(_qi) + qx[3] * sigma6(_qi))
                );

        return true;
    }
    const Eigen::Vector4d _ti, _tj, _qi;
};


struct f3 {
    f3 (Eigen::Vector4d ti, Eigen::Vector4d tj, Eigen::Vector4d qi) :
        _ti(ti), _tj(tj), _qi(qi) {}

    template <typename T>
    bool operator() (const T* const x,  // arguments, 10 dimension
                     T* residual) const // residual error
    {
        T qx[4] = {x[0], x[1], x[2], x[3]};
        T tx[4] = {T(0.0), x[4], x[5], x[6]};
        T tb[4] = {T(0.0), x[7], x[8], x[9]};

        residual[0] = T(
                    qx[0] * (_ti[2] - _tj[2]) + qx[0] * tx[2] + qx[1] * tx[3] - qx[3] * tx[1]
                + qx[3] * (_ti[1] + _tj[1]) - qx[1] * (_ti[3] + _tj[3])
                + tb[1] * (qx[3] * sigma3(_qi) + qx[0] * sigma7(_qi) + qx[1] * sigma5(_qi))
                - tb[2] * (qx[1] * sigma9(_qi) - qx[0] * sigma2(_qi) + qx[3] * sigma4(_qi))
                - tb[3] * (qx[1] * sigma1(_qi) + qx[0] * sigma6(_qi) - qx[3] * sigma8(_qi))
                );

        return true;
    }
    const Eigen::Vector4d _ti, _tj, _qi;
};


struct f4 {
    f4 (Eigen::Vector4d ti, Eigen::Vector4d tj, Eigen::Vector4d qi) :
        _ti(ti), _tj(tj), _qi(qi) {}

    template <typename T>
    bool operator() (const T* const x,
                     T* residual) const
    {
        T qx[4] = {x[0], x[1], x[2], x[3]};
        T tx[4] = {T(0.0), x[4], x[5], x[6]};
        T tb[4] = {T(0.0), x[7], x[8], x[9]};

        residual[0] = T(
                    qx[0] * (_ti[3] - _tj[3]) + qx[0] * tx[3] - qx[1] * tx[2] + qx[2] * tx[1]
                - qx[2] * (_ti[1] + _tj[1]) + qx[1] * (_ti[2] + _tj[2])
                - tb[1] * (qx[2] * sigma3(_qi) + qx[0] * sigma5(_qi) - qx[1] * sigma7(_qi))
                + tb[2] * (qx[1] * sigma2(_qi) + qx[0] * sigma9(_qi) + qx[2] * sigma4(_qi))
                - tb[3] * (qx[1] * sigma6(_qi) - qx[0] * sigma1(_qi) + qx[2] * sigma8(_qi))
                );

        return true;
    }

    const Eigen::Vector4d _ti, _tj, _qi;
};


struct PENALTY {
    PENALTY () {}

    template <typename T>
    bool operator() (const T* const x,
                     T* residual) const
    {
        T qx[4] = {x[0], x[1], x[2], x[3]};
        T qTq = qx[0] * qx[0] + qx[1] * qx[1] + qx[2] * qx[2] + qx[3] * qx[3];
        residual[0] = T( double(sqrt(lamda)) * (1.0 - qTq ) );

        return true;

    }

};


void loadTFPoses(const string& robot_pose_filename,
                 cv::Mat& tfPoses)
{
    FileStorage fsread(robot_pose_filename, FileStorage::READ);
    fsread["robotposes"] >> tfPoses;
    fsread.release();
}

int main(int argc, char** argv )
{

    /**
    1. read icp output and store them in TJs = {{tj1} {tj2} {tj3}...}
    2. read tf from robot base to hand, store rotation in QIs = {{qi1} {qi2} {qi3}...}
    3. read tf from robot base to hand, store translation in TIs = {{ti1} {ti2} {ti3} ...}
    4. calculate qx, tx from chessboard, as initial value for optimization
    5  measure tb as initial value for optimization
    6. construct least square problem
    */



    vector<Eigen::Vector4d> TJs;  // icp translations as pure imaginary quaternion(0;x;y;z)
    vector<Eigen::Vector4d> TIs;  // tf transaltions as pure imaginary quaternion (0;x;y;z)
    vector<Eigen::Vector4d> QIs;  // tf rotations as quaternion: (qw, qx, qy, qz);!!!!


    cout << "hello world" << endl;
    string robot_pose_file = "./test/testdata/robot_pose.xml";
    cv::Mat tfBaseGrippers;
    loadTFPoses(robot_pose_file, tfBaseGrippers);


    int numOfFiles = tfBaseGrippers.rows;

    Eigen::Quaterniond qgc;

    qgc.x() = -0.0213248179092;
    qgc.y() = 0.00884362796331;
    qgc.z() = -0.893675406915;
    qgc.w() = 0.44811974902;
    Eigen::Vector3d tgc(-0.0426610901036,  0.0435682041765,  0.0628709556475);
    Eigen::Isometry3d Tgc = Eigen::Isometry3d::Identity();
    Tgc.rotate(qgc);
    Tgc.pretranslate(tgc);


    Eigen::Vector3d sphereCenterInB(1.5, 1.5, 1.5);

    auto Tcg = Tgc.inverse();
    auto qcg = Eigen::Quaterniond(Tcg.rotation());
    auto tcg = Tcg.translation();


    // fake by adding noise;
    Eigen::AngleAxisd delta_rot(M_PI/8, Eigen::Vector3d(0,1,1));
    Eigen::Vector3d delta_t(0.15, 0.15, 0.10);  // 沿x,y,z轴平移 0.15,0.15,0.1 米
    Eigen::Isometry3d Tdelta = Eigen::Isometry3d::Identity();
    Tdelta.rotate(delta_rot);
    Tdelta.pretranslate(delta_t);

    auto Tcg_init = Tdelta * Tcg;
    auto qcg_init = Eigen::Quaterniond(Tcg_init.rotation());
    auto tcg_init = Tcg_init.translation();
    Eigen::Vector3d sphereCenterInB_init(0, 0, 0);

    auto Tgc_init = Tcg_init.inverse();
    auto qgc_init = Eigen::Quaterniond(Tgc_init.rotation());
    auto tgc_init = Tgc_init.translation();

//    double x[10] = {qcg.w(), qcg.x(), qcg.y(), qcg.z(),
//                    tcg.x(), tcg.y(), tcg.z(),
//                    sphereCenterInB.x(), sphereCenterInB.y(), sphereCenterInB.z()};

    double x[10] = {qcg_init.w(), qcg_init.x(), qcg_init.y(), qcg_init.z(),
                    tcg_init.x(), tcg_init.y(), tcg_init.z(),
                    sphereCenterInB_init.x(), sphereCenterInB_init.y(), sphereCenterInB_init.z()};


    // cout << tfPoses << endl;

     double w_sigma = 0.03;
    cv::RNG rng;
    for (int i = 0; i < numOfFiles; i++)
    {
        cv::Mat& tf = tfBaseGrippers;  // x y z qx qy qz qw
        Eigen::Vector3d translbgi(tf.at<double>(i,0), tf.at<double>(i,1), tf.at<double>(i,2));
        Eigen::Quaterniond quantbgi;

        quantbgi.x() = tf.at<double>(i,3);
        quantbgi.y() = tf.at<double>(i,4);
        quantbgi.z() = tf.at<double>(i,5);
        quantbgi.w() = tf.at<double>(i,6);

        Eigen::Isometry3d Tbgi = Eigen::Isometry3d::Identity();
        Tbgi.rotate(quantbgi);
        Tbgi.pretranslate(translbgi);

        auto Tgbi = Tbgi.inverse();
        auto qgbi = Eigen::Quaterniond(Tgbi.rotation());
        auto tgbi = Tgbi.translation();

        Eigen::Vector4d TI(0.0, tgbi.x(), tgbi.y(), tgbi.z());
        Eigen::Vector4d QI(qgbi.w(), qgbi.x(), qgbi.y(), qgbi.z());
        TIs.push_back(TI);
        QIs.push_back(QI);

        Eigen::Isometry3d Tbci = Tbgi * Tgc;
        Eigen::Isometry3d Tcbi = Tbci.inverse();
        Eigen::Vector3d sphereCenterInCi = Tcbi * sphereCenterInB;
        Eigen::Vector3d daltaCenter(rng.gaussian(w_sigma), rng.gaussian(w_sigma), rng.gaussian(w_sigma));
        cout << "adding purturb to the viewd sphere center: delta:" << daltaCenter.transpose() << endl;
        sphereCenterInCi += daltaCenter;

        Eigen::Vector4d TJ(0.0, sphereCenterInCi.x(), sphereCenterInCi.y(), sphereCenterInCi.z());
        TJs.push_back(TJ);

    }

    cout << "****** generate Qis Tis Tjs complete! total " << numOfFiles << " datas *******" << endl;


    ceres::Problem problem;

    cout<<endl;
    for (int i = 0; i < numOfFiles; i++)
    {
        // add residual Fi {fi1,fi2,fi3,fi4} to Problem,
        problem.AddResidualBlock (
                    // using auto diffrentiate, < error type(f1), output dimension, input dimension >
                    new ceres::AutoDiffCostFunction<f1, 1, 10> (new f1 (TIs[i], TJs[i], QIs[i]) ),
                    nullptr, //  核函数，这里不使用，为空
                    x   // arguments that need estimated.
                    );

        problem.AddResidualBlock (
                    new ceres::AutoDiffCostFunction<f2, 1, 10> (new f2 (TIs[i], TJs[i], QIs[i]) ),
                    nullptr,
                    x
                    );
        problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<f3, 1, 10> (new f3 (TIs[i], TJs[i], QIs[i]) ),
                    nullptr,
                    x
                    );
        problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<f4, 1, 10> (new f4 (TIs[i], TJs[i], QIs[i]) ),
                    nullptr,
                    x
                    );
    }

    problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<PENALTY, 1, 10> (new PENALTY()),
                nullptr,
                x
                );

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_type = ceres::TRUST_REGION;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.num_threads = 4;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    ceres::Solve(options, &problem, &summary);

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;

    cout<<summary.BriefReport() <<endl;
    cout<<endl;

    Eigen::Quaterniond qcg_result;
    qcg_result.w() = x[0];  qcg_result.x() = x[1]; qcg_result.y() = x[2]; qcg_result.z() = x[3];
    Eigen::Vector3d tcg_result(x[4], x[5], x[6]);
    Eigen::Vector3d sphereCenterInB_result(x[7], x[8], x[9]);
    auto Tcg_result = Eigen::Isometry3d::Identity();
    Tcg_result.rotate(qcg_result);
    Tcg_result.pretranslate(tcg_result);
    auto Tgc_result = Tcg_result.inverse();
    auto qgc_result = Eigen::Quaterniond(Tgc_result.rotation());
    auto tgc_result = Tgc_result.translation();


    cout << "##################  HANDEYE is transformation from ee_link to camera_link \n" <<
            "##################  or tf: (/ee_link,  /camera_link) \n" <<
            "##################  or father: /ee_link,  child: /camera_link \n" <<
            "##################  or P3d(in ee_link) = T_handeye * P3d(in camera_link) \n\n";

    cout << "#### handeye real: (x, y, z) " << tgc.transpose()
         << "  |   (qx, qy, qz, qw) " << qgc.coeffs().transpose()
         << "  |   (sphere in base): " << sphereCenterInB.x() << ", " << sphereCenterInB.y() << ", " << sphereCenterInB.z()  << endl;

    cout << "#### handeye init: (x, y, z) " << tgc_init.transpose()
         << "  |   (qx, qy, qz, qw) " << qgc_init.coeffs().transpose()
         << "  |   (sphere in base): " << sphereCenterInB_init.x() << ", " << sphereCenterInB_init.y() << ", " << sphereCenterInB_init.z() << endl;


    cout << "#### handeye estm: (x, y, z) " << tgc_result.transpose()
         << "  |   (qx, qy, qz, qw) " << qgc_result.coeffs().transpose()
         << "  |   (sphere in base): " << sphereCenterInB_result.x() << ", " << sphereCenterInB_result.y() << ", " << sphereCenterInB_result.z()  << endl;


    //cout << T.matrix() << endl;
    //    Mat Mat_handeye_result_1_7(1,7,CV_64F);
    //    Mat_handeye_result_1_7.at<double>(0,0) = qinv.w();
    //    Mat_handeye_result_1_7.at<double>(0,1) = qinv.x();
    //    Mat_handeye_result_1_7.at<double>(0,2) = qinv.y();
    //    Mat_handeye_result_1_7.at<double>(0,3) = qinv.z();
    //    Mat_handeye_result_1_7.at<double>(0,4) = trans_vec(0);
    //    Mat_handeye_result_1_7.at<double>(0,5) = trans_vec(1);
    //    Mat_handeye_result_1_7.at<double>(0,6) = trans_vec(2);

    //    FileStorage fswrite(handeye_result_file, FileStorage::WRITE);
    //    fswrite << "handeyeResult" << Mat_handeye_result_1_7;
    //    fswrite.release();

    return 0;
}

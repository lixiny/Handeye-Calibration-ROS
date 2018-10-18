#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>
#include <math.h>
#include <iostream>
#include "math.h"
#include <cmath>
#include <vector>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include "sophus/so3.h"
#include "sophus/se3.h"

using namespace std;

const int lamda = 1000000;
#define random(a,b) (rand()%(b-a+1)+a)

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


void generateFakeData( vector<Eigen::Vector4d>& TIs, vector<Eigen::Vector4d>& TJs, vector<Eigen::Vector4d>& QIs);

int main(int argc, char** argv)
{
    vector<Eigen::Vector4d> TJs;  // icp translations as pure imaginary quaternion(0;x;y;z)
    vector<Eigen::Vector4d> TIs;  // tf transaltions as pure imaginary quaternion (0;x;y;z)
    vector<Eigen::Vector4d> QIs;  // tf rotations as quaternion
   
    generateFakeData(TIs,TJs,QIs);

    /**  x: 10 unknows: [  (qx0, qx1, qx2, qx3)    (tx1, tx2, tx3)    (tb1, tb2, tb3)  ]  */

    // double x_real[10] = {0.991445, 0, 0, 0.130526,  0.15,0.15,0.1,  1.0,1.0,1.5};  // 初值 准确
    // double x[10] = {0.991445, 0, 0, 0.130526,  0.10,0.10,0.05,  1.5,0.5,1.3};  // 初值 1
    double x[10] = {1, 0, 0, 0,    0, 0, 0,    0, 0, 0}; 

    cout << "the initial valuse of x is: \n";
    for ( auto i:x ) cout<<i<<" ";

    /**
    1. read icp output and store them in TJs = {{tj1} {tj2} {tj3}...}
    2. read tf from robot base to hand, store rotation in QIs = {{qi1} {qi2} {qi3}...}
    3. read tf from robot base to hand, store translation in TIs = {{ti1} {ti2} {ti3} ...}
    4. calculate qx, tx from chessboard, as initial value for optimization
    5  measure tb as initial value for optimization
    6. construct least square problem
    */

    ceres::Problem problem;

    cout<<endl;
    for (int i = 0; i < TJs.size(); i++)
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
    options.num_threads = 3;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    ceres::Solve(options, &problem, &summary);

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;

    cout<<summary.BriefReport() <<endl;
    cout<<"estimated x = ";
    for ( auto i:x ) cout<<i<<" ";
    cout<<endl;

    return 0;
}

void generateFakeData( vector<Eigen::Vector4d>& TIs, vector<Eigen::Vector4d>& TJs, vector<Eigen::Vector4d>& QIs) 
{

    /** 生成未知量X: 相机光心 到 机器人抓手末端 4x4 转换矩阵 */
    Eigen::AngleAxisd rot_vec(M_PI/12, Eigen::Vector3d(0,0,1)); // 沿z轴旋转15度
    Eigen::Matrix3d XR = rot_vec.toRotationMatrix();
    cout << "XR is " << "\n" << XR << "\n\n";
    Eigen::Quaterniond qx = Eigen::Quaterniond(rot_vec);
    cout << "qx is: [" << qx.w() << " " << qx.x() << " " << qx.y() << " " << qx.z() << "]" << "\n\n"; 
    Eigen::Vector3d Xt(0.15, 0.15, 0.10);  // 沿x,y,z轴平移 0.15,0.15,0.1 米
    Sophus::SE3 Xse3(XR,Xt);
    Eigen::Matrix4d X = Xse3.matrix();
    cout << "X is " << "\n" << X << "\n\n";

    /** 生成未知量B: 机器人底座 到 标定球心 4x4 转换矩阵 */
    Eigen::Matrix3d BR = Eigen::AngleAxisd(0.0 * M_PI, Eigen::Vector3d(0,0,0)).toRotationMatrix();  // 没有旋转
    Eigen::Vector3d Bt(1.0, 1.0, 1.5);   // 沿x,y,z轴平移 1.0,1.0,1.5 米
    Sophus::SE3 Bse3(BR,Bt);
    Eigen::Matrix4d B = Bse3.matrix();
    cout << "B is " << "\n" << B << "\n\n";

    double ERROR = 0;
    int count = 0;
    cout << "###########################################" << endl;
    for(int i=0;i<5;i++){
        for (int j=0;j<5;j++){
            for (int k=0;k<5;k++){

                Eigen::Vector3d I_rotVec(-1+(double)i*0.4,-1+(double)j*0.4,-1+(double)k*0.4);
                I_rotVec.normalize();

                int num = random(10,45);
                double I_rotAngle = num * M_PI / 180;
                Eigen::Vector3d IRvec = I_rotAngle * I_rotVec;
                Eigen::Vector3d It((-200.0+random(0,400))/1000.0,
                                    (-200.0+random(0,400))/1000.0,
                                    (-200.0+random(0,400))/1000.0);

                Sophus::SO3 IRso3 = Sophus::SO3::exp(IRvec);
                Sophus::SE3 Ise3(IRso3,It);
                Eigen::Matrix3d IR = IRso3.matrix();
                Eigen::Matrix4d I = Ise3.matrix();
                cout << count << "#  I is: \n" << I << "\n\n"; 
                Eigen::Matrix4d J = X * I * B ;
                cout << count << "#  J is: \n" << J << "\n\n"; 

                Eigen::Vector3d Jt(J(0,3),J(1,3),J(2,3));

                // /** 取消注释以给 Jt 生成高斯噪声 */
                // double w_sigma = 1.0;
                // cv::RNG rng;
                // Jt << J(0,3)+rng.gaussian(w_sigma),  J(1,3)+rng.gaussian(w_sigma),  J(2,3)+rng.gaussian(w_sigma);
                

                Eigen::Vector3d ei = (XR * IR * Bt + XR * It + Xt) - Jt;
                double error = ei.transpose() * ei;
                cout << count << "#  error is: " << error << "\n\n";

                Eigen::Quaterniond qi = Eigen::Quaterniond(IR);
                Eigen::Vector3d ev = (qx*qi*Bt + qx*It + Xt) -Jt;
                // cout << count << " ## error vector is: \n"                     // 已经验证 ei = ev
                //      << ev.transpose() << "\n" << ei.transpose() << "\n\n";

                ERROR += error;
                
                
                Eigen::Vector4d tj(0.0, Jt(0), Jt(1), Jt(2));
                Eigen::Vector4d ti(0.0, It(0), It(1), It(2));
                /** 这里要做一次Eigen四元数:(x,y,z,w) 到 general四元数:(w,x,y,z)的转换 */
                Eigen::Vector4d qi_wxyz(qi.w(), qi.x(), qi.y(), qi.z());
                cout << count << "#  qi_wxyz is: \n [" << qi_wxyz.transpose() << "]" << "\n\n";

                TIs.push_back(ti);
                TJs.push_back(tj);
                QIs.push_back(qi_wxyz);

                count++;
                cout << "###########################################" << endl;

            }
        }
    }
    Eigen::Vector4d qx_vec(qx.w(),qx.x(),qx.y(),qx.z());
    cout << "Total ERROR before Penalty is (should be 0): " << ERROR << endl;
    ERROR += 1000000 * (1 - qx_vec.transpose() * qx_vec) * (1 - qx_vec.transpose() * qx_vec) ;
    cout << "Total ERROR after Penalty is (should be 0): " << ERROR << endl;
    cout << "generate done, total " << count << " datas" << "\n\n";
    cout << "the real value of x is: \n";
    cout << qx.w() << " " << qx.x() << " " << qx.y() << " " << qx.z() << " "
         << Xt(0) << " " << Xt(1) << " " << Xt(2) << " " 
         << Bt(0) << " " << Bt(1) << " " << Bt(2) << endl;
}

































#include "common_include.h"
#include "calibration_tool.h"

using namespace std;

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

vector<string> getFilesInDirectory ( string cate_dir ) {
    vector<string> files;
    DIR *dir;
    struct dirent *ptr;

    if ((dir=opendir(cate_dir.c_str())) == NULL)
    {
        perror("Open dir error...");
        exit(1);
    }

    while ((ptr=readdir(dir)) != NULL)
    {
        if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
            continue;
        else if(ptr->d_type == 8)    ///file
            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
            files.push_back(ptr->d_name);
        else if(ptr->d_type == 10)    ///link file
            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
            continue;
        else if(ptr->d_type == 4)    ///dir
        {
            files.push_back(ptr->d_name);
        }
    }
    closedir(dir);

    std::sort(files.begin(), files.end());
    for (int i = 0; i< files.size(); i++) {
        files[i] = cate_dir + files[i];
    }
    return files;
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
    vector<Eigen::Vector4d> QIs;  // tf rotations as quaternion
    string robot_pose_file = "robot_pose.xml";
    string handeye_init_file  = "handeye_init.xml";
    string handeye_result_file = "handeye_result.xml";
    vector<string> allPointCloudFiles = getFilesInDirectory("dataset/");
    int            numOfFiles         = allPointCloudFiles.size();
    if(numOfFiles == 0) {
        cerr << "ERROR: No PointsClouds File Found in ./dataset. Program Shut Down ! " << endl;
        return -1;
    }

    for (int i = 0; i < numOfFiles; i++)
    {

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PCDReader reader;
        reader.read<pcl::PointXYZRGBA>(allPointCloudFiles[i], *cloud);
        std::vector<int> inliers;
        pcl::SampleConsensusModelSphere<pcl::PointXYZRGBA>::Ptr
                model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZRGBA> (cloud));
        pcl::RandomSampleConsensus<pcl::PointXYZRGBA> ransac (model_s);
        ransac.setDistanceThreshold (RANSACthreshold);
        ransac.computeModel();
        Eigen::VectorXf sphereModel;
        ransac.getModelCoefficients(sphereModel);
        float x = sphereModel(0);
        float y = sphereModel(1);
        float z = sphereModel(2);
        Eigen::Vector4d TJ(0.0, x, y, z);

        TJs.push_back(TJ);
    }

    cv::Mat tfPoses;
    calibtool::loadTFPoses(robot_pose_file, tfPoses);
    if(tfPoses.rows != numOfFiles){
        cout << " Error: Number of TFPoses doesn't match number of PointsClouds, Program Shut Down ! " << endl;
        return -1;
    }

    // cout << tfPoses << endl;
    for (int i = 0; i < tfPoses.rows; i++)
    {
        Eigen::Vector4d TI(0.0, tfPoses.at<double>(i,4), tfPoses.at<double>(i,5), tfPoses.at<double>(i,6));
        Eigen::Vector4d QI(tfPoses.at<double>(i,0), tfPoses.at<double>(i,1), tfPoses.at<double>(i,2), tfPoses.at<double>(i,3));
        TIs.push_back(TI);
        QIs.push_back(QI);
    }

    cout << "****** Load Qis Tis Tjs complete! total " << numOfFiles << " datas *******" << endl;

    cv::Mat handeyeInitPoses;
    Eigen::Quaterniond q;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    calibtool::loadTFPoses(handeye_init_file, handeyeInitPoses);
    if(handeyeInitPoses.rows != 1 || handeyeInitPoses.cols != 7) {
        cerr << "WARNING: handeye pose in [ handeye_init.xml ] is invalid, Will use ZERO initiation instead" << endl;
        q.x() = 0.0;
        q.y() = 0.0;
        q.z() = 0.0;
        q.w() = 1.0;
        Eigen::AngleAxisd rot_vec = Eigen::AngleAxisd(q);       
        T.rotate(rot_vec);
        T.pretranslate(Eigen::Vector3d (0.0, 0.0, 0.0 ));
        cout <<"Initial handeye pose: \n" <<  T.matrix() << endl;
    } else {
        cout << "Use handeye pose in [ handeye_init.xml ] as init value" << endl;
        q.x() = handeyeInitPoses.at<double>(0,1);
        q.y() = handeyeInitPoses.at<double>(0,2);
        q.z() = handeyeInitPoses.at<double>(0,3);
        q.w() = handeyeInitPoses.at<double>(0,0);
        Eigen::AngleAxisd rot_vec = Eigen::AngleAxisd(q);
        T.rotate(rot_vec);
        T.pretranslate(Eigen::Vector3d (handeyeInitPoses.at<double>(0,4), handeyeInitPoses.at<double>(0,5), handeyeInitPoses.at<double>(0,6)));
        cout << "Initial handeye pose: \n" <<  T.matrix() << endl;
    }

    Eigen::Isometry3d Tinv = T.inverse();
    cout <<"init transformation matrix is: \n" <<  Tinv.matrix() << "\n\n";
    Eigen::Matrix3d rot_mat = Tinv.matrix().block(0,0,2,2);
    cout << "init rotation matrix is: \n" << rot_mat << endl;
    Eigen::Vector3d trans_vec = Tinv.matrix().block(0,3,2,3);
    cout  << "init translation matrix is: \n" << trans_vec.transpose() << "\n\n";
    Eigen::Quaterniond qinv = Eigen::Quaterniond(rot_mat);
    double x[10] = {qinv.w(), qinv.x(), qinv.y(), qinv.z(),    trans_vec(0), trans_vec(1), trans_vec(2),    1, 1, 1};

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
    cout<<endl;

    cout << "qx(x,y,z,w) estimated: " << x[1] << " " << x[2] << " " << x[3] << " " << x[0] << endl;
    cout << "tx(x,y,z) estimated: "   << x[4] << " " << x[5] << " " << x[6] << endl;
    cout << "tb(x,y,z) estimated: "   << x[7] << " " << x[8] << " " << x[9] << endl;

    
    q.x() = x[1];
    q.y() = x[2];
    q.z() = x[3];
    q.w() = x[0];


    Eigen::AngleAxisd rot_vec = Eigen::AngleAxisd(q);
    T = Eigen::Isometry3d::Identity();
    T.rotate(rot_vec);
    T.pretranslate(Eigen::Vector3d (x[4], x[5], x[6] ));
    //cout << T.matrix() << endl;
    Tinv = T.inverse();
    //cout << Tinv.matrix() << endl;
    rot_mat = Tinv.matrix().block(0,0,2,2);
    // cout << rot_mat << endl;
    trans_vec = Tinv.matrix().block(0,3,2,3);
    cout << " ********  translation from hand to camrea is: ********* \n " << trans_vec.transpose() << endl;
    qinv = Eigen::Quaterniond(rot_mat);
    cout << " ********  quaternions from hand to camera is: ********* \n" << qinv.x() << "" << qinv.y() << " " << qinv.z() << " " << qinv.w() << endl;

    Mat Mat_handeye_result_1_7(1,7,CV_64F); 
    Mat_handeye_result_1_7.at<double>(0,0) = qinv.w();
    Mat_handeye_result_1_7.at<double>(0,1) = qinv.x();
    Mat_handeye_result_1_7.at<double>(0,2) = qinv.y();
    Mat_handeye_result_1_7.at<double>(0,3) = qinv.z();
    Mat_handeye_result_1_7.at<double>(0,4) = trans_vec(0);
    Mat_handeye_result_1_7.at<double>(0,5) = trans_vec(1);
    Mat_handeye_result_1_7.at<double>(0,6) = trans_vec(2);

    FileStorage fswrite(handeye_result_file, FileStorage::WRITE);
    fswrite << "handeyeResult" << Mat_handeye_result_1_7;
    fswrite.release();

    return 0;
}

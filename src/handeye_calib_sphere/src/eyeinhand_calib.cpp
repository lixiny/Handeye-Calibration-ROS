//pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

// cv
#include <opencv2/core/eigen.hpp>

//STD C++ INCLUDES
#include <iostream>
#include <cstring>
#include <termios.h>
#include <memory>
#include <mutex>

// ros
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// tf
#include <tf/transform_listener.h>

//Eigen

#include <eigen3/Eigen/Core>
// ceres
#include <ceres/ceres.h>
#include "ceres_extensions.h"

#include <boost/make_shared.hpp>

using namespace std;
using namespace cv;

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;

string cameraTFname, ARTagTFname;
string EETFname, baseTFname;

string initHandToEyeTFPath;
string finetuneHandToEyeTFPath;
string transformPairsRecordPath;

vector<Eigen::Isometry3d> vTb2e;
vector<Eigen::Vector3d> vSphereInCam;

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
    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

int getch()
{
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt); // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);               // disable buffering
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // apply new settings

    int c = getchar(); // read character (non-blocking)

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore old settings
    return c;
}

void tf2Isometry3d(const tf::StampedTransform &tfpose, Eigen::Isometry3d &T3d)
{
    tf::Quaternion quat(0, 0, 0, 0);
    tf::Vector3 transl(0, 0, 0);
    quat = tfpose.getRotation();
    transl = tfpose.getOrigin();
    Eigen::Quaterniond q(quat.w(), quat.x(), quat.y(), quat.z());
    Eigen::AngleAxisd rot_vec = Eigen::AngleAxisd(q);
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(rot_vec);
    T.pretranslate(Eigen::Vector3d(transl.x(), transl.y(), transl.z()));
    T3d = T;
}

void loadTF(const string &tfPath, cv::Mat &hand2eyeTF)
{
    FileStorage fsread(tfPath, FileStorage::READ);
    fsread["handToEyeTF"] >> hand2eyeTF;
    fsread.release();
}

class PointCloudExtractor
{
public:
    ros::NodeHandle nh;
    ros::Subscriber pc_sub;
    ros::Publisher pc_pub;
    string inPointcloudTopic;
    string outPointcloudTopic;
    tf::TransformListener tfListener;
    tf::StampedTransform m2cTF;
    tf::StampedTransform b2eTF;
    bool hasCam = true, hasEE = true;
    float planeSideLen;
    PointCloud::Ptr cloudInCube;
    mutex datamutex;

    PointCloudExtractor(
        ros::NodeHandle &nh_,
        string inPointcloudTopic_,
        string outPointcloudTopic_,
        float planeSideLen_) : nh(nh_),
                               inPointcloudTopic(inPointcloudTopic_),
                               outPointcloudTopic(outPointcloudTopic_),
                               planeSideLen(planeSideLen_)

    {
        pc_sub = nh.subscribe(inPointcloudTopic, 1, &PointCloudExtractor::pointcloudCB, this);
        pc_pub = nh.advertise<PointCloud>(outPointcloudTopic, 1);
        cloudInCube = boost::make_shared<PointCloud>();
    }

    tf::StampedTransform getBase2EETF()
    {
        unique_lock<mutex> lock(datamutex);
        if (hasEE)
        {
            return b2eTF;
        }
        else
        {
            tf::Transform zeroTF;
            zeroTF.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
            zeroTF.setRotation(tf::Quaternion(0, 0, 0, 1));

            return tf::StampedTransform(zeroTF,
                                        ros::Time::now(),
                                        baseTFname,
                                        EETFname);
        }
    }

    PointCloud::Ptr getCloudInCube()
    {
        unique_lock<mutex> lock(datamutex);
        return cloudInCube;
    }

    bool getHasCam()
    {
        unique_lock<mutex> lock(datamutex);
        return hasCam;
    }

    bool getHasEE()
    {
        unique_lock<mutex> lock(datamutex);
        return hasEE;
    }

    void pointcloudCB(const PointCloud::Ptr &cloud)
    {
        unique_lock<mutex> lock(datamutex);
        cloudInCube->clear();
        cloudInCube->header.frame_id = cameraTFname;
        hasCam = true;
        hasEE = true;

        try
        {
            /** get the latest tfTransform from /ar_marker to camera_link */
            tfListener.lookupTransform(ARTagTFname, cameraTFname, ros::Time(0), m2cTF);
            tf::Vector3 t = m2cTF.getOrigin();
            if (abs(t.x()) + abs(t.y()) + abs(t.z()) < 1e-6)
            {
                hasCam = false;
            }
        }
        catch (tf::TransformException ex)
        {
            ROS_WARN("Fail to lookup TF transform between %s to %s",
                     ARTagTFname.c_str(), cameraTFname.c_str());
            hasCam = false;
        }

        try
        {
            /** get the latest tfTransform from base_link(source) to ee_link(target) */
            tfListener.lookupTransform(baseTFname, EETFname, ros::Time(0), b2eTF);
            tf::Vector3 t = b2eTF.getOrigin();
            if (abs(t.x()) + abs(t.y()) + abs(t.z()) < 1e-6)
            {
                hasEE = false;
            }
        }
        catch (tf::TransformException ex)
        {
            ROS_WARN("Fail to lookup TF transform between %s to %s",
                     baseTFname.c_str(), EETFname.c_str());
            hasEE = false;
        }

        if (!hasCam)
        {
            pc_pub.publish(cloudInCube);
            return;
        }

        // extract point cloud based on the aruco plane tranform.
        Eigen::Isometry3d Tm2c;
        tf2Isometry3d(m2cTF, Tm2c);

        for (size_t i = 0; i < cloud->points.size(); i++)
        {
            Eigen::Vector4d point;
            point << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, 1.0;
            Eigen::Vector4d pointInAruco = Tm2c * point;

            if (pointOutCube(pointInAruco))
                continue;
            cloudInCube->points.push_back(cloud->points[i]);
        }

        cloudInCube->height = 1;
        cloudInCube->width = cloudInCube->points.size();
        cloudInCube->header.frame_id = cameraTFname;
        pc_pub.publish(cloudInCube);
    }

    bool pointOutCube(const Eigen::Vector4d &pt)
    {
        return ((abs(pt[0]) > planeSideLen / 2) || (abs(pt[1]) > planeSideLen / 2) || (pt[2] > planeSideLen) || pt[2] < -0.02);
    }
};

class ReprojectionError3D
{
public:
    ReprojectionError3D(const Eigen::Vector3d pInCami_, // spherecenter Point viewd by camera position i
                        const Eigen::Vector3d pInCamj_,
                        const Eigen::Isometry3d Tb2ei_,
                        const Eigen::Isometry3d Tb2ej_) : pInCami(pInCami_), pInCamj(pInCamj_), Tb2ei(Tb2ei_), Tb2ej(Tb2ej_)
    {
    }

    template <typename T>
    bool operator()(const T *const handeye_rotation, const T *const handeye_translation, T *residuals) const
    {

        Eigen::Quaternion<T> Q_e2c = Eigen::Map<const Eigen::Quaternion<T>>(handeye_rotation);
        Eigen::Matrix<T, 3, 1> t_e2c = Eigen::Map<const Eigen::Matrix<T, 3, 1>>(handeye_translation);

        // Make sure the Eigen::Vector world point is using the ceres::Jet type as it's Scalar type !!!
        Eigen::Matrix<T, 3, 1> Pi_c;
        Pi_c << T(pInCami.x()), T(pInCami.y()), T(pInCami.z());
        /** Transform pointInCamra to pointInGripper for position i */

        // Rotate the point using Eigen rotations
        // cout << "Pi_c :" << T(Pi_c[0]) << " " << T(Pi_c[1]) << " " << T(Pi_c[2]) << endl;
        Eigen::Matrix<T, 3, 1> Pi_e = Q_e2c * Pi_c + t_e2c;
        // Map T* to Eigen Vector3 with correct Scalar type

        /** Transform pointInGripper to pointInBase for position i */
        auto Ri = Tb2ei.rotation();
        auto ti = Tb2ei.translation();
        Eigen::Quaterniond Qi(Ri);
        Eigen::Quaternion<T> Qi_b2e(T(Qi.w()), T(Qi.x()), T(Qi.y()), T(Qi.z()));
        Eigen::Matrix<T, 3, 1> ti_b2e;
        ti_b2e << T(ti.x()), T(ti.y()), T(ti.z());

        Eigen::Matrix<T, 3, 1> Pi_b = Qi_b2e * Pi_e + ti_b2e;

        /** ###################################################################*/
        Eigen::Matrix<T, 3, 1> Pj_c;
        Pj_c << T(pInCamj.x()), T(pInCamj.y()), T(pInCamj.z());

        /** Transform pointInCamra to pointInGripper for position j */
        Eigen::Matrix<T, 3, 1> Pj_e = Q_e2c * Pj_c + t_e2c;

        /** Transform pointInGripper to pointInBase for position j */
        auto Rj = Tb2ej.rotation();
        auto tj = Tb2ej.translation();
        Eigen::Quaterniond Qj(Rj);
        Eigen::Quaternion<T> Qj_b2e(T(Qj.w()), T(Qj.x()), T(Qj.y()), T(Qj.z()));
        Eigen::Matrix<T, 3, 1> tj_b2e;
        tj_b2e << T(tj.x()), T(tj.y()), T(tj.z());

        Eigen::Matrix<T, 3, 1> Pj_b = Qj_b2e * Pj_e + tj_b2e;

        residuals[0] = T(T(Pi_b[0]) - T(Pj_b[0])); //   deltaX = Xi - Xj
        residuals[1] = T(T(Pi_b[1]) - T(Pj_b[1]));
        residuals[2] = T(T(Pi_b[2]) - T(Pj_b[2]));

        return true;
    }

    const Eigen::Vector3d pInCami, pInCamj;
    const Eigen::Isometry3d Tb2ei, Tb2ej;
};

void addFrame(PointCloudExtractor &extractor)
{
    if (!extractor.getHasEE())
    {
        ROS_ERROR("No valid hand transform. SKip");
        return;
    }
    if (!extractor.getHasCam())
    {
        ROS_ERROR("No valid eye transform. SKip");
        return;
    }
    PointCloud::Ptr cloud = extractor.getCloudInCube();
    if (cloud->points.empty())
    {
        ROS_ERROR("No valid point cloud inside your aruco cube. SKip");
        return;
    }

    pcl::SampleConsensusModelSphere<pcl::PointXYZRGBA>::Ptr
        modelSphere(new pcl::SampleConsensusModelSphere<pcl::PointXYZRGBA>(cloud));

    pcl::RandomSampleConsensus<pcl::PointXYZRGBA> ransac(modelSphere);

    float radius = 0.75;
    double threshold = 0.001; // 1mm
    ransac.setDistanceThreshold(threshold);
    ransac.computeModel();

    Eigen::VectorXf sphereCoeff;
    ransac.getModelCoefficients(sphereCoeff);
    float r = sphereCoeff(3);

    if (abs(r - radius) > 0.005)
    {
        ROS_ERROR("\e[1;35mInvalid sphere sample (r:=%f) Skip.\e[0m", r);
        return;
    }

    Eigen::Vector3d sphere(sphereCoeff(0), sphereCoeff(1), sphereCoeff(2));
    vSphereInCam.push_back(sphere);

    tf::StampedTransform b2eTF = extractor.getBase2EETF();
    Eigen::Isometry3d Tb2e;
    tf2Isometry3d(b2eTF, Tb2e);
    vTb2e.push_back(Tb2e);

    ROS_INFO("\e[1;34mAdding Sample # %u \e[0m",
             (unsigned int)vSphereInCam.size());
}

void finetuneHandEye(Eigen::Quaterniond &Q_e2c, Eigen::Vector3d &t_e2c)
{
    ceres::Problem problem;
    ceres::LocalParameterization *quaternion_parameterization = new ceres_ext::EigenQuaternionParameterization();
    int counter = 0;
    int nSample = vSphereInCam.size();
    for (int i = 0; i < nSample; i++)
    {
        for (int j = i + 1; j < nSample; j++)
        {
            problem.AddResidualBlock(
                new ::ceres::AutoDiffCostFunction<ReprojectionError3D, 3, 4, 3>(
                    new ReprojectionError3D(vSphereInCam[i], vSphereInCam[j], vTb2e[i], vTb2e[j])),
                NULL,
                Q_e2c.coeffs().data(),
                t_e2c.data());
            counter++;
        }
    }
    cout << "\e[1;33mTotal (C_" << nSample << "^2): " << counter << " number of error edges added.\e[0m " << endl;
    problem.SetParameterization(Q_e2c.coeffs().data(), quaternion_parameterization);
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
    ceres::Solve(options, &problem, &summary);
    cout << summary.BriefReport() << "\n\n";
}

void reportCalibration(const Eigen::Quaterniond &Q_e2c, const Eigen::Vector3d &t_e2c)
{

    cout << "##################  HANDEYE is transformation from ee_link to camera_link \n"
         << "##################  or tf: (/ee_link,  /camera_link) \n"
         << "##################  or father: /ee_link,  child: /camera_link \n"
         << "##################  or P3d(in ee_link) = T_handeye * P3d(in camera_link) \n\n";

    std::cout << "\e[1;34m\n\n";
    std::cerr << "Result from " << EETFname << " to " << cameraTFname << ":\e[0m" << std::endl;

    std::cerr << "\e[1;33m";
    std::cerr << "Translation (x,y,z) : "
              << t_e2c.transpose() << std::endl;

    std::stringstream ss;
    ss << Q_e2c.x() << " "
       << Q_e2c.y() << " "
       << Q_e2c.z() << " "
       << Q_e2c.w();
    std::cerr << "Rotation q(x,y,z,w): " << ss.str() << "\e[0m" << std::endl;

    std::cout << "\e[1;34m"
              << "Now you can publish tf in: [ Translation, Rotation] "
              << EETFname << " " << cameraTFname << "\e[0m"
              << "\n\n";
    return;
}

void writeCalibration(const Eigen::Quaterniond &Q_e2c, const Eigen::Vector3d &t_e2c, const string &filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    std::cerr << "Writing calibration to \"" << filename << "\"...\n";
    if (fs.isOpened())
    {
        // in tf format (x,y,z,qx,qy,qz,qw)
        cv::Mat Mat_tfpose_1_7(1, 7, CV_64F); // row:1 coloum:7
        Mat_tfpose_1_7.at<double>(0, 0) = t_e2c.x();
        Mat_tfpose_1_7.at<double>(0, 1) = t_e2c.y();
        Mat_tfpose_1_7.at<double>(0, 2) = t_e2c.z();
        Mat_tfpose_1_7.at<double>(0, 3) = Q_e2c.x();
        Mat_tfpose_1_7.at<double>(0, 4) = Q_e2c.y();
        Mat_tfpose_1_7.at<double>(0, 5) = Q_e2c.z();
        Mat_tfpose_1_7.at<double>(0, 6) = Q_e2c.w();
        fs << "handToEyeTF" << Mat_tfpose_1_7;
        fs.release();
    }
}

/// @return 0 on success, otherwise error code
int writeTransformPairsToFile(const vector<Eigen::Isometry3d> &vT,
                              const vector<Eigen::Vector3d> & vP,
                              std::string filename)
{
    std::cerr << "Writing pairs to \"" << filename << "\"...\n";
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    int frameCount = vT.size();
    fs << "frameCount" << frameCount;
    auto T_it = vT.begin();
    auto P_it = vP.begin();

    if (fs.isOpened())
    {

        for (int i = 0; i < vT.size(); ++i, ++T_it, ++P_it)
        {
            cv::Mat_<double> cvT = cv::Mat_<double>::ones(4, 4);
            cv::eigen2cv(T_it->matrix(), cvT);
            cv::Mat_<double> cvP = cv::Mat_<double>::ones(3, 1);
            cv::eigen2cv(*P_it, cvP);

            std::stringstream ss1;
            ss1 << "T_" << i;
            fs << ss1.str() << cvT;

            std::stringstream ss2;
            ss2 << "P_" << i;
            fs << ss2.str() << cvP;
        }
        fs.release();
    }
    else
    {
        std::cerr << "failed to open output file " << filename << "\n";
        return 1;
    }
    return 0;
}

int main(int argc, char **argv)
{
    cout << "\n\n";

    ros::init(argc, argv, "eyeinhand_calib");
    ros::NodeHandle n_("~");
    float planeSideLen;
    string inPointcloudTopic, outPointcloudTopic;

    n_.param("planeSideLen", planeSideLen, 0.25f);
    n_.param("cameraTF", cameraTFname, string("/camera_link"));
    n_.param("ARTagTF", ARTagTFname, string("/ar_marker"));
    n_.param("EETF", EETFname, string("/ee_link"));
    n_.param("baseTF", baseTFname, string("/base_link"));
    n_.param("inPointcloudTopic", inPointcloudTopic, string("/realsense/cloud"));
    n_.param("outPointcloudTopic", outPointcloudTopic, string("aruco_extractor/cloud"));
    n_.param("initHandToEyeTFPath", initHandToEyeTFPath, string("CalibratedTransform.yml"));
    n_.param("finetuneHandToEyeTFPath", finetuneHandToEyeTFPath, string("FinetunedTransform.yml"));
    n_.param("transformPairsRecordPath", transformPairsRecordPath, string("TransformPairsRecord.yml"));

    ros::Rate r(30); // 30 hz

    tf::TransformListener tfListener;
    tf::StampedTransform tfTransform;

    // load init value of handToEye transformation.
    cv::Mat e2cTF; // ee to camera TF
    loadTF(initHandToEyeTFPath, e2cTF);
    if (e2cTF.empty())
    {
        ROS_ERROR("No init value of TF %s %s in %s.",
                  EETFname.c_str(), cameraTFname.c_str(), initHandToEyeTFPath.c_str());
        return -1;
    }

    // Eigen::Isometry3d Te2c = Eigen::Isometry3d::Identity();
    Eigen::Vector3d t_e2c(e2cTF.at<double>(0, 0), e2cTF.at<double>(0, 1), e2cTF.at<double>(0, 2));
    Eigen::Quaterniond Q_e2c(e2cTF.at<double>(0, 6), e2cTF.at<double>(0, 3), e2cTF.at<double>(0, 4), e2cTF.at<double>(0, 5));
    // Te2c.rotate(Q_e2c);
    // Te2c.pretranslate(t_e2c);

    ros::Duration(1.0).sleep(); // cache the TF transforms
    int key = 0;
    ROS_INFO("\e[1;35m Press s to add the current frame transformation and point cloud to the cache.\e[0m");
    ROS_INFO("\e[1;34m Press d to delete last frame transformation.\e[0m");
    ROS_INFO("\e[1;33m Press q to calibrate frame transformation and exit the application.\e[0m");

    ros::NodeHandle nh;
    PointCloudExtractor extractor(nh, inPointcloudTopic, outPointcloudTopic, planeSideLen);

    // cache data in extractor
    ros::spinOnce();

    while (ros::ok())
    {
        ros::spinOnce();
        if (!kbhit())
        {
            r.sleep();
            continue;
        }

        key = getch();
        if ((key == 's') || (key == 'S'))
        {
            addFrame(extractor);
            writeTransformPairsToFile(vTb2e, vSphereInCam,
                                      transformPairsRecordPath);
        }
        else if ((key == 'd') || (key == 'D'))
        {
            ROS_INFO("Deleted last sample. Number of current "
                     "samples: %u",
                     (unsigned int)vSphereInCam.size());
            vSphereInCam.pop_back();
            vTb2e.pop_back();
        }
        else if ((key == 'q') || (key == 'Q'))
        {
            if (vSphereInCam.size() < 15)
            {
                ROS_ERROR("Number of sample pairs < 15. Please continue to add more");
                continue;
            }
            ROS_INFO("Calculating Calibration...");
            finetuneHandEye(Q_e2c, t_e2c);
            reportCalibration(Q_e2c, t_e2c);
            writeCalibration(Q_e2c, t_e2c, finetuneHandToEyeTFPath);
        }
        else
        {
            std::cerr << key << " pressed.\n";
        }
        r.sleep();
    }

    ros::shutdown();
    return (0);
}

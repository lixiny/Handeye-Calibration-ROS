#include "CommonInclude.h"
#include "CalibrationTool.h"
#include "ceres_extensions.h"

#include <omp.h>

using namespace std;
using namespace cv;

string dataInputDir;
string handeyeInitFile;
string handeyeResultFile;
string robotPoseInput;
double sphereModelRadius;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt,  Eigen::Matrix4f &final_transform, bool downsample = false);

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

    ros::init ( argc, argv, "eyeinhand_calib_node" );
    ros::NodeHandle nh("~");

    nh.param("dataInputDir", dataInputDir, std::string("./dataset") );
    nh.param("handeyeInitFile", handeyeInitFile, std::string("./dataset/handeye_init.xml"));
    nh.param("handeyeResultFile", handeyeResultFile, std::string("./dataset/handeye_result.xml"));
    nh.param("robotPoseInput", robotPoseInput, std::string("./dataset/robot_pose.xml") );
    nh.param("sphereModelRadius", sphereModelRadius, 0.075 );

    cout << "\n\n>>>> SETTING >>>>" << endl;
    cout << ">>>> dataInputDir:  " << dataInputDir << endl;
    cout << ">>>> handeyeInitFile:  " << handeyeInitFile << endl;
    cout << ">>>> handeyeResultFile:  " << handeyeResultFile << endl;
    cout << ">>>> robotPoseInput:  " <<  robotPoseInput << endl;
    cout << ">>>> sphereModelRadius:  " << sphereModelRadius << endl;
    cout << "\n\n" << endl;

    string pointCLoudSuffix = ".pcd";
    vector<string> allPointCloudFiles = calibtool::getFilesInDirectory(dataInputDir, pointCLoudSuffix);
    int nClouds = allPointCloudFiles.size();
    if(nClouds == 0) {
        cerr << "ERROR: No PointsClouds File Found in " << dataInputDir << " Program Shut Down ! " << endl;
        return -1;
    }
    // for(auto each : allPointCloudFiles) cout << each << endl;

    cv::Mat tfBaseGrippers;
    calibtool::loadTFPoses(robotPoseInput, tfBaseGrippers);

    /** tf in format: x y z qx qy qz qw */
    int nPoses = tfBaseGrippers.rows;
    if(tfBaseGrippers.empty() || nPoses != nClouds){
        cout << " Error: No tf_base_gripper data in " << robotPoseInput <<
                "\n or Number of tfBaseGrippers doesn't match number of PointsClouds, Program Shut Down ! " << endl;
        return -1;
    }

    /** tf in format: x y z qx qy qz qw */
    cv::Mat tfGripperCameraInit;
    calibtool::loadTFPoses(handeyeInitFile, tfGripperCameraInit);
    if(tfGripperCameraInit.empty()){
        cout << " Error: No tf_gripper_camera data in " << handeyeInitFile << " Program Shut Down ! " << endl;
        return -1;
    }

    vector<Eigen::Quaterniond> vQuatbg;
    vector<Eigen::Vector3d> vTranslbg;
    vector<Eigen::Isometry3d> vTbg;
    vector<Eigen::Vector3d> vSphereCenterInCamera;


    Eigen::Isometry3d Tgc = Eigen::Isometry3d::Identity();
    cv::Mat& tfgc = tfGripperCameraInit;
    Eigen::Vector3d tgc (tfgc.at<double>(0,0), tfgc.at<double>(0,1), tfgc.at<double>(0,2));
    Eigen::Quaterniond qgc;
    qgc.x() = tfgc.at<double>(0,3);
    qgc.y() = tfgc.at<double>(0,4);
    qgc.z() = tfgc.at<double>(0,5);
    qgc.w() = tfgc.at<double>(0,6);

    Tgc.rotate(qgc);
    Tgc.pretranslate(tgc);

    cout << "Init Hand-Eye Transformation in tf fromat: (x y z qx qy qz qw) is :\n"
            " " << tgc.transpose() << " " << qgc.coeffs().transpose() << endl;
    Eigen::Quaterniond qgcInit(qgc);
    Eigen::Vector3d tgcInit(tgc);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGBA>);
    float radius = (float)sphereModelRadius;
    Eigen::Vector3f center(0.0, 0.0, 0.0);
    calibtool::generateSphereFragment(model, center, radius);

    vector<int> badCloudids;
    cout << "Calculating PointCloud "; cout.flush();

    for(int i = 0; i < nClouds; i++) {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_inliers (new pcl::PointCloud<pcl::PointXYZRGBA>);
        std::vector<int> inliers;

        pcl::PCDReader reader;
        reader.read<pcl::PointXYZRGBA>(allPointCloudFiles[i], *cloud);

        pcl::SampleConsensusModelSphere<pcl::PointXYZRGBA>::Ptr
                model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZRGBA> (cloud));

        pcl::RandomSampleConsensus<pcl::PointXYZRGBA> ransac (model_s);

        double threshold = 0.001; // 0.5mm
        ransac.setDistanceThreshold (threshold);

        ransac.computeModel();
        Eigen::VectorXf sphereModel;
        ransac.getModelCoefficients(sphereModel);
        float r = sphereModel(3);

        if(r >= radius + 0.005 || r <= radius - 0.005) {
            badCloudids.push_back(i);
            cout << "*" ; cout.flush();
            continue;
        }

        Eigen::Vector3d sphereCenter(sphereModel(0), sphereModel(1), sphereModel(2));
        vSphereCenterInCamera.push_back(sphereCenter);
        cout << "."; cout.flush();
        //        cout << " after ICP: (" << sphereCenter.transpose() << ")" << endl;
    }

    cout << "Complete" << "\n\n";

    cout << "Loading tf ...\n";
    for(int i = 0; i < nPoses; i++ ) {
        if(find(badCloudids.begin(), badCloudids.end(), i) != badCloudids.end()){
            cout << " Skip # " << i << " pose because bad PointClouds" << endl;
            continue;
        }

        cv::Mat& tfbg = tfBaseGrippers;
        Eigen::Vector3d tbgi(tfbg.at<double>(i,0), tfbg.at<double>(i,1), tfbg.at<double>(i,2));
        Eigen::Quaterniond qbgi;

        qbgi.x() = tfbg.at<double>(i,3);
        qbgi.y() = tfbg.at<double>(i,4);
        qbgi.z() = tfbg.at<double>(i,5);
        qbgi.w() = tfbg.at<double>(i,6);

        vTranslbg.push_back(tbgi);
        vQuatbg.push_back(qbgi);

        Eigen::Isometry3d Tbgi = Eigen::Isometry3d::Identity();
        Tbgi.rotate(qbgi);
        Tbgi.pretranslate(tbgi);
        vTbg.push_back(Tbgi);
    }
    cout << "Loading tf Complete" << endl;

    int nValidData = vTbg.size();
    cout << "Total " << nValidData << " data record" << endl;

    if(nValidData != int(vSphereCenterInCamera.size())) {
        cout << " Error: or Number of valid tfBaseGrippers doesn't match number of valid sphere center, Program Shut Down ! " << endl;
        return -1;
    }

    ceres::Problem problem;


    // Use my LocalParameterization
    ceres::LocalParameterization *quaternion_parameterization = new ceres_ext::EigenQuaternionParameterization();
    int counter = 0;
    for(int i = 0; i < nValidData; i++) {
        for(int j = i+1; j < nValidData; j++) {
            problem.AddResidualBlock(
                        new::ceres::AutoDiffCostFunction<ReprojectionError3D, 3, 4, 3>(
                            new ReprojectionError3D(vSphereCenterInCamera[i], vSphereCenterInCamera[j], vTbg[i], vTbg[j])),
                        NULL,
                        qgc.coeffs().data(),
                        tgc.data());
            counter++;
        }
    }

    cout << "Total (C_" << nValidData << "_2): " << counter << " number of error edges added! " << endl;
    problem.SetParameterization(qgc.coeffs().data(), quaternion_parameterization );

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

    cout<<summary.BriefReport() <<"\n\n";

    cout << "##################  HANDEYE is transformation from ee_link to camera_link \n" <<
            "##################  or tf: (/ee_link,  /camera_link) \n" <<
            "##################  or father: /ee_link,  child: /camera_link \n" <<
            "##################  or P3d(in ee_link) = T_handeye * P3d(in camera_link) \n\n";

    cout << "init value      in tf fromat:(x y z qx qy qz qw) is : ";
    cout << tgcInit.transpose() << " " << qgcInit.coeffs().transpose() << endl;

    cout << "estm value      in tf fromat:(x y z qx qy qz qw) is : ";
    cout << tgc.transpose() << " " << qgc.coeffs().transpose() << endl;


    Mat Mat_handeye_result_1_7(1,7,CV_64F);
    Mat_handeye_result_1_7.at<double>(0,0) = tgc.x();
    Mat_handeye_result_1_7.at<double>(0,1) = tgc.y();
    Mat_handeye_result_1_7.at<double>(0,2) = tgc.z();
    Mat_handeye_result_1_7.at<double>(0,3) = qgc.x();
    Mat_handeye_result_1_7.at<double>(0,4) = qgc.y()    ;
    Mat_handeye_result_1_7.at<double>(0,5) = qgc.z();
    Mat_handeye_result_1_7.at<double>(0,6) = qgc.w();

    FileStorage fswrite(handeyeResultFile, FileStorage::WRITE);
    fswrite << "handeyeResult" << Mat_handeye_result_1_7;
    fswrite.release();
    cout << "Writing tf data in format: (x y z qx qy qz qw) to " << handeyeResultFile << endl;

    return 0;
}


void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt,  Eigen::Matrix4f &final_transform, bool downsample)
{
    //
    // Downsample for consistency and speed
    // \note enable this for large datasets
    PointCloud::Ptr src (new PointCloud);
    PointCloud::Ptr tgt (new PointCloud);
    pcl::VoxelGrid<PointT> grid;
    if (downsample)
    {
        grid.setLeafSize (0.05, 0.05, 0.05);
        grid.setInputCloud (cloud_src);
        grid.filter (*src);

        grid.setInputCloud (cloud_tgt);
        grid.filter (*tgt);
    }
    else
    {
        src = cloud_src;
        tgt = cloud_tgt;
    }

    //
    // Align
    pcl::IterativeClosestPoint<PointT, PointT> reg;
    reg.setTransformationEpsilon (1e-9);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance (1.5f);
    // Set the point representation

    reg.setInputSource (src);
    reg.setInputTarget (tgt);

    //
    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev;
    PointCloud::Ptr reg_result = src;
    reg.setMaximumIterations (10);

    for (int i = 0; i < 30; ++i)
    {
        //        PCL_INFO ("Iteration Nr. %d.\n", i);

        // save cloud for visualization purpose
        src = reg_result;

        // Estimate
        reg.setInputSource (src);
        reg.align (*reg_result);

        //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation () * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
            reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.2);

        prev = reg.getLastIncrementalTransformation ();
        //        PCL_INFO ("Iteration Nr. %d. finished\n", i);


    }

    //
    // Get the transformation from target to source
    Eigen::Matrix4f targetToSource = Ti.inverse();
    //    cout << "target To Source: \n" << targetToSource.matrix() << "\n\n";

    final_transform = targetToSource;

}

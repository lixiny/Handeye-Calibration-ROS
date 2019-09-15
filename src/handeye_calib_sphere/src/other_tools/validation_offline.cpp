#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <Eigen/Dense>
#include <math.h>
#include "common_include.h"
#include "calibration_tool.h"

#define KEYCODE_SPACE   0x20
#define KEYCODE_ESC     0x1B

using namespace std;
using namespace cv;



const string CAMERA_DATA_FILE = "/home/sirius/rgbd_ws/src/sphere_handeye/camera_intrinsic_depth.xml";
const string robot_base_link = "/base_link";
const string robot_end_link = "/arm_left_link_7_t";

Mat cameraMatrix;
Mat distCoeffs;
bool firstFrame = true;
Eigen::Matrix4d refPose;
Eigen::Vector4d refPoint;
vector<double> Errors;

Eigen::Vector3d getSphereFromPointColud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, double& radius);
double calculateDistance(Eigen::Vector3d& pt1, Eigen::Vector3d& pt2);
vector<string> getFilesInDirectory ( string cate_dir ) ;

int main(int argc, char** argv)
{

//    double x[10] = {0.706926, -0.705006, -0.0521853, -0.0223529, -0.035389, 0.033641, -0.0678332};
     double x[10] = {0.706926, -0.736006, -0.0500853, -0.0243229, -0.032538, 0.0329437, -0.064823};
    Eigen::Quaterniond q;
    q.x() = x[0];
    q.y() = x[1];
    q.z() = x[2];
    q.w() = x[3];

    Eigen::AngleAxisd rot_vec = Eigen::AngleAxisd(q);
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(rot_vec);
    T.pretranslate(Eigen::Vector3d (x[4], x[5], x[6] ));
    Eigen::Matrix4d X = T.matrix();
    Eigen::Matrix3d Rx = X.block(0,0,2,2);
    Eigen::Vector3d tx = X.block(0,3,2,3);
    Eigen::Vector3d tb(0.97099, 0.739276, 0.471574);


    cout << "X is: \n" << X << endl;

    string robot_pose_file = "/home/sirius/rgbd_ws/src/sphere_handeye/robot_pose.xml";
    vector<string> allPointCloudFiles = getFilesInDirectory("/home/sirius/rgbd_ws/src/sphere_handeye/dataset/");
    int            numOfFiles         = allPointCloudFiles.size();

    cv::Mat tfPoses;
    calibtool::loadTFPoses(robot_pose_file, tfPoses);
    double error = 0;
    double counter = 0;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGBA>);

    for (int k = 0; k < numOfFiles; k++)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PCDReader reader;
        reader.read<pcl::PointXYZRGBA>(allPointCloudFiles[k], *cloud);


        Eigen::Matrix4d tfTransformMatrix;
        Eigen::Quaterniond q;
        q.x() = tfPoses.at<double>(k,1);
        q.y() = tfPoses.at<double>(k,2);
        q.z() = tfPoses.at<double>(k,3);
        q.w() = tfPoses.at<double>(k,0);
        Eigen::AngleAxisd rot_vec = Eigen::AngleAxisd(q);
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.rotate(rot_vec);
        T.pretranslate(Eigen::Vector3d (tfPoses.at<double>(k,4), tfPoses.at<double>(k,5), tfPoses.at<double>(k,6) ));
        // Eigen::Isometry3d Tinv = T.inverse();
        tfTransformMatrix = T.matrix();
        cout << "tfTransformMatrix is \n" << tfTransformMatrix << endl;
        Eigen::Matrix3d Ri = tfTransformMatrix.block(0,0,2,2);
        Eigen::Vector3d ti = tfTransformMatrix.block(0,3,2,3);

        Eigen::Vector3d tj_est = Rx * Ri * tb + Rx * ti + tx;
        cout << "tj esitmate is :" << tj_est.transpose() << endl;
        double radius;
        Eigen::Vector3d tj_icp = getSphereFromPointColud(cloud, radius);

        double r = 0.075;


        if (abs(radius - 0.075) > 0.001)
        {
            continue;
        } else {
            cout << "tj icp is :" << tj_icp.transpose() << endl;

            error += calculateDistance(tj_est, tj_icp);
            counter ++;

            if (k % 7 == 0) {
                for (int y = 0; y < cloud->points.size(); y++){
                    final->points.push_back(cloud->points[y]);
                }

                for (int i = 0; i < 360; i++) {
                    for (int j = 0; j < 180; j++) {
                        pcl::PointXYZRGBA p_est;
                        pcl::PointXYZRGBA p_icp;
                        float alpha = i * (360.0 / 360);  // degree 0 ~ 360
                        float beta = j * (180.0 / 180);    // degree 0 ~ 180
                        float alphaRad = alpha * M_PI/180;
                        float betaRad =  beta * M_PI/180;

                        p_est.x = tj_est(0) + r * sin(betaRad) * cos(alphaRad);
                        p_est.y = tj_est(1) + r * sin(betaRad) * sin(alphaRad);
                        p_est.z = tj_est(2) + r * cos(betaRad);
                        p_est.b = (uchar)255;
                        p_est.g = (uchar)102;
                        p_est.r = (uchar)102;
                        final->points.push_back(p_est);

                        p_icp.x = tj_icp(0) + r * sin(betaRad) * cos(alphaRad);
                        p_icp.y = tj_icp(1) + r * sin(betaRad) * sin(alphaRad);
                        p_icp.z = tj_icp(2) + r * cos(betaRad);
                        p_icp.b = (uchar)0;
                        p_icp.g = (uchar)0;
                        p_icp.r = (uchar)240;
                        final->points.push_back(p_icp);

                    }
                }
            }


        }

    }

    cout << "valid data # is: " << counter << endl;
    double averageError = error/double(counter);
    cout << "averageError "  << averageError <<endl;


    final->height = 1;
    final->width = final->points.size();
    final->resize ( final->height * final->width );

    pcl::visualization::PCLVisualizer viewer("find_sphere");
    viewer.addPointCloud(final, "find_sphere");
    viewer.setBackgroundColor(40.0/255,40.0/255,52.0/255);
    viewer.addCoordinateSystem(0.1);
    while (!viewer.wasStopped())
    { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce();
    }


}


double calculateDistance(Eigen::Vector3d& pt1, Eigen::Vector3d& pt2)
{
    double squareError = pow((pt1(0) - pt2(0)),2) + pow((pt1(1)-pt2(1)),2) + pow((pt1(2)-pt2(2)),2);
    return pow(squareError, 0.5);
}


Eigen::Vector3d getSphereFromPointColud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, double& radius)
{
    pcl::SampleConsensusModelSphere<pcl::PointXYZRGBA>::Ptr
            model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZRGBA> (cloud));

    pcl::RandomSampleConsensus<pcl::PointXYZRGBA> ransac (model_s);

    ransac.setDistanceThreshold (0.0005);

    ransac.computeModel();
    Eigen::VectorXf sphereModel;
    ransac.getModelCoefficients(sphereModel);
    cout << "sphere model is: \n" << sphereModel.transpose() << endl;

    float x = sphereModel(0);
    float y = sphereModel(1);
    float z = sphereModel(2);
    radius = sphereModel(3);
    Eigen::Vector3d ptInCamera (x, y, z);
    return ptInCamera;
}

vector<string> getFilesInDirectory ( string cate_dir )
{
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


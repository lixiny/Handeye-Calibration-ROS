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

Eigen::Vector4d getSphereFromPointColud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
double calculateDistance(Eigen::Vector4d& pt1, Eigen::Vector4d& pt2);
int kbhit(void);

int main(int argc, char** argv)
{
    if (! calibtool::initCameraIntrinsic(CAMERA_DATA_FILE, cameraMatrix, distCoeffs)) {
        cout << "cannot locate [camera_intrinsics] XML file || [camera_intrinsics] XML file contains invalid data" << endl;
        return -1;
    }

    Mat rgb_image, rgb_temp;
    Mat depth_image;
    //     w,  qx,  qy,  qz,  x,  y,  z
    double x[10] = {0.0223529, 0.706926, -0.705006, -0.0521853, 0.0263283, -0.0323438, -0.0778302};
    Eigen::Quaterniond q;
    q.x() = x[1];
    q.y() = x[2];
    q.z() = x[3];
    q.w() = x[0];

    cout << "qx(x,y,z,w) estimated: " << x[1] << " " << x[2] << " " << x[3] << " " << x[0] << endl;
    cout << "tx(x,y,z) estimated: "   << x[4] << " " << x[5] << " " << x[6] << endl;
    Eigen::AngleAxisd rot_vec = Eigen::AngleAxisd(q);
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(rot_vec);
    T.pretranslate(Eigen::Vector3d (x[4], x[5], x[6] ));
    Eigen::Matrix4d X = T.matrix();


    ros::init ( argc, argv, "validation_node" );
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<rgbd_srv::rgbd> ( "realsense_server" ); // bind client to server
    rgbd_srv::rgbd srv;   // serivce type
    srv.request.start = true;
    sensor_msgs::Image msg_rgb;
    sensor_msgs::Image msg_depth;
    tf::TransformListener tfListener;
    tf::StampedTransform tfTransform;
    ros::Rate loop_rate ( 60 );

    pcl::visualization::PCLVisualizer viewer ( "3D viewer" );
    viewer.setBackgroundColor ( 40.0/255,40.0/255,52.0/255 );

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_scene ( new pcl::PointCloud<pcl::PointXYZRGBA>() );
    while( ros::ok() )
    {
        if ( client.call( srv )) { // client send request(call server), when success, return a bool true
            try
            {
                msg_rgb = srv.response.rgb_image;
                msg_depth = srv.response.depth_image;

                // use cv_bridge convert sensor_msgs::Image to cv::Mat
                rgb_image = cv_bridge::toCvCopy ( msg_rgb, sensor_msgs::image_encodings::BGR8 )->image;
                rgb_image.copyTo(rgb_temp);

                depth_image = cv_bridge::toCvCopy ( msg_depth, sensor_msgs::image_encodings::TYPE_32FC1 )->image;

                if ( !rgb_image.data || !depth_image.data )
                {
                    printf ( "Fatal: current no image data!  \n" );
                    return -1;
                }

            } catch ( cv_bridge::Exception& e ) {
                ROS_ERROR ( "cv_bridge execption: %s", e.what() );
                return -1;
            }

            try
            {
                /** get the latest tfTransform from robot_base_link(source) to robot_end_link(target) */
                tfListener.lookupTransform (robot_base_link, robot_end_link, ros::Time ( 0 ), tfTransform );

            } catch ( tf::TransformException ex ) {
                ROS_ERROR("%s", ex.what());
                cout << "Warning: current may not have tf data! " << endl;
            }
        }

        if (rgb_temp.empty() || depth_image.empty()) {
            continue;
        }

        imshow("color", rgb_temp);
        imshow("depth", depth_image);
        waitKey(1);
        cloud_scene->clear();
        for (int r=0;r<depth_image.rows;r++)
        {
            for (int c=0;c<depth_image.cols;c++)
            {
                pcl::PointXYZRGBA p;
                if ( ! ( depth_image.ptr<float> ( r ) [c] > 0 ) || depth_image.ptr<float> ( r ) [c] > 2.0 ) {
                    continue;   // unwanted depth at current point
                }

                Eigen::Vector3f scenePoint;
                float depth_in_meter = float ( depth_image.ptr<float> ( r ) [c] );

                calibtool::deprojectPixelToPoint(scenePoint,
                                                 c, r,
                                                 depth_in_meter,
                                                 cameraMatrix,
                                                 distCoeffs );

                p.x = scenePoint(0);
                p.y = scenePoint(1);
                p.z = scenePoint(2);
                p.b = rgb_image.ptr<uchar> ( r ) [c * 3];
                p.g = rgb_image.ptr<uchar> ( r ) [c * 3 + 1];
                p.r = rgb_image.ptr<uchar> ( r ) [c * 3 + 2];
                cloud_scene->points.push_back ( p );

            }
        }

        cloud_scene->height = 1;
        cloud_scene->width = cloud_scene->points.size();
        cloud_scene->resize ( cloud_scene->height * cloud_scene->width );
        // extractPlaneFeatureFromCloud(cloud_scene, cloud_scene);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
        sor.setInputCloud(cloud_scene);
        sor.setMeanK(50);   // mean value
        sor.setStddevMulThresh(1.0);
        sor.filter(*cloud_scene);

        viewer.addPointCloud ( cloud_scene, "cloud_scene" );
        viewer.addCoordinateSystem ( 1.0 );
        viewer.spinOnce ( 1 );
        viewer.removeAllPointClouds();

        if( kbhit())   //space down
        {
            char c = getchar();
            if(c == KEYCODE_ESC)
            {
                cout<<"shuting down..."<<endl;
                viewer.close();

                ros::shutdown();
                break;
            } else {
                Eigen::Matrix4d tfTransformMatrix;
                if (firstFrame){
                    calibtool::TFToTransformMatrix(tfTransform, tfTransformMatrix);
                    refPose = tfTransformMatrix;
                    Eigen::Vector4d ptInCamera = getSphereFromPointColud(cloud_scene);
                    refPoint << ptInCamera(0), ptInCamera(1), ptInCamera(2), 1.0;
                    firstFrame = false;
                } else {
                    calibtool::TFToTransformMatrix(tfTransform, tfTransformMatrix);
                    Eigen::Matrix4d currentA = refPose.inverse() * tfTransformMatrix;
                    Eigen::Matrix4d currentB = X.inverse() * currentA * X;
                    Eigen::Vector4d newP    = currentB * refPoint;
                    Eigen::Vector4d ptInCamera = getSphereFromPointColud(cloud_scene);
                    double error = calculateDistance(ptInCamera, newP);
                    cout << error << endl;
                    Errors.push_back(error);

                }

            }
        }

    }

}


double calculateDistance(Eigen::Vector4d& pt1, Eigen::Vector4d& pt2)
{
    double squareError = pow((pt1(0) - pt2(0)),2) + pow((pt1(1)-pt2(1)),2) + pow((pt1(3)-pt2(3)),2) + pow((pt1(4) - pt2(4)),2);
    return pow(squareError, 0.5);
}


Eigen::Vector4d getSphereFromPointColud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
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
    float r = sphereModel(3);
    Eigen::Vector4d ptInCamera (x, y, z, 1.0);
    return ptInCamera;
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


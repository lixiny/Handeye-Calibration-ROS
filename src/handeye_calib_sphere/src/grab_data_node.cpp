#include "CommonInclude.h"
#include "CalibrationTool.h"
#include "ArucoPlane.h"
#include "rgbd_srv/rgbd.h"

using namespace std;
using namespace cv;

#define KEYCODE_SPACE   0x20
#define KEYCODE_ESC     0x1B

string  robotPoseOutput;
string  cameraIntrinsicInput;
string  EETFname;
string  baseTFname;
string  RGBDserviceName;
string  dataOutputDir;
bool    useQrExtractor;
bool    recordingTF;
int     numOfQRcodeOnSide;

Mat cameraMatrix;
Mat distCoeffs;

enum class USAGE
{
    STANDALONE                          ,  /** basic usage: only grab images and pointclouds */
    WITH_QR_EXTRACTOR                   ,  /** grab images and pointclouds extracted from the region inside the cuboid */
    WITH_ROBOT_TF_POSE                  ,  /** grab images, pointclouds and synchronizd tf transformation between two coordinates*/
    WITH_QR_EXTRACTOR_AND_ROBOT_TF_POSE ,  /** advanced usage: grab images, extracted pointclouds and synchronized tf */
    USAGE_COUNT
} usage ;

// ################################### function prototype ############################################

int kbhit(void);
int getch();
void extractPlaneFeatureFromCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud_source,
  									pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud_destin);

// ################################### main function ###########################################
int main(int argc, char** argv)
{
    cout << "\n\n";

    ros::init ( argc, argv, "grab_data_node" );
    ros::NodeHandle nh("~");

    nh.param("useQrExtractor", useQrExtractor, false);
    nh.param("recordingTF", recordingTF, false);
    nh.param("robotPoseOutput" , robotPoseOutput, std::string("./robot_pose.xml"));
    nh.param("cameraIntrinsicInput", cameraIntrinsicInput, std::string("./camera_intrinsic_color.xml"));
    nh.param("numOfQRcodeOnSide", numOfQRcodeOnSide, 0);
    nh.param("EETF", EETFname, std::string("/ee_link"));
    nh.param("baseTF", baseTFname, std::string("/base_link"));
    nh.param("RGBDserviceName",  RGBDserviceName,  std::string("realsense2_server"));
    nh.param("dataOutputDir", dataOutputDir, std::string("./dataset") );

    cout << "robotPoseOutput " << robotPoseOutput << endl;
    cout << "dataOutputDir " << dataOutputDir << endl;
    cout << "RGBDserviceName" <<  RGBDserviceName << endl;

    cout << "Reading Camera Intrinsic File from : " << cameraIntrinsicInput << endl;

    if (! calibtool::initCameraIntrinsic(cameraIntrinsicInput, cameraMatrix, distCoeffs)) {
        cerr << "FATAL:  Cannot locate [camera_intrinsics] XML file Or [camera_intrinsics] XML file contains invalid data. Program will exit." << endl;
        return -1;
    }

    if(useQrExtractor) {
        if(numOfQRcodeOnSide != 6) {
            cerr << "WARNING: Wrong settings to Aruco plate!  will not use QrExtractor" << endl;
            useQrExtractor = false;
        }
    }

    cout << "\n Init Complete ! \n" << endl;
    int fileSavedCounter = 1;

    Mat rgb_image, rgb_temp;
    Mat depth_image;
    ros::NodeHandle nh2;
    ros::ServiceClient client = nh2.serviceClient<rgbd_srv::rgbd> ( RGBDserviceName ); // bind client to server
    rgbd_srv::rgbd srv;   // serivce type
    srv.request.start = true;
    sensor_msgs::Image msg_rgb;
    sensor_msgs::Image msg_depth;

    tf::TransformListener tfListener;
    tf::StampedTransform  tfTransform;

    ros::Rate loop_rate ( 30 );

    pcl::visualization::PCLVisualizer viewer ( "3D viewer" );
    viewer.setBackgroundColor ( 40.0/255,40.0/255,52.0/255 );

    handeye::ArucoPlanePtr pArucoPlane;

    float cubeSide = 0;
    bool  validAruco = false;
    int key = 0;

    if( useQrExtractor ) 
    {
        auto pArucoPlaneTemp =  make_shared<handeye::ArucoPlane>(numOfQRcodeOnSide);
        pArucoPlane = pArucoPlaneTemp;
        cubeSide   = pArucoPlane->getCubeSide();
        pArucoPlane->setCameraIntrinsic(cameraMatrix, distCoeffs);
        cout << "INFO:  Using aruco extractor " <<endl;
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_scene ( new pcl::PointCloud<pcl::PointXYZRGBA>() );

    while( ros::ok() )
    {

        if ( ! client.call( srv )) {  // client send request(call server), when success, return a bool true
            cerr << "ERROR:  Client want to get new image from realsense2_server but FAILED !" << endl;
            loop_rate.sleep();
            continue;
        }

        try
        {
            msg_rgb = srv.response.rgb_image;
            msg_depth = srv.response.depth_image;

            // use cv_bridge convert sensor_msgs::Image to cv::Mat
            rgb_image = cv_bridge::toCvCopy ( msg_rgb, sensor_msgs::image_encodings::BGR8 )->image;
            rgb_image.copyTo(rgb_temp);

            depth_image = cv_bridge::toCvCopy ( msg_depth, sensor_msgs::image_encodings::TYPE_32FC1 )->image;

            if (rgb_temp.empty() || depth_image.empty()) {
                cout << "WARNING:  Current no image data!  \n" << endl;
                loop_rate.sleep();
                continue;
            }

        } catch ( cv_bridge::Exception& e ) {
            ROS_ERROR ( "cv_bridge execption: %s", e.what() );
            return -1;
        }

        if ( recordingTF ) {
            try
            {
                /** get the latest tfTransform from robot_base_link(source) to robot_end_link(target) */
                tfListener.lookupTransform ( EETFname, baseTFname, ros::Time ( 0 ), tfTransform );

            } catch ( tf::TransformException ex ) {
                ROS_ERROR("%s", ex.what());
                cout << "Warning: Current may not have tf data! " << endl;
                loop_rate.sleep();
                continue;
            }
        }

        Eigen::Matrix4f transform, transform_inv;

        chrono::steady_clock::time_point t3 = chrono::steady_clock::now();
        if ( useQrExtractor ) 
        {
            validAruco = pArucoPlane->calculateExtrinsicFromImage( rgb_image ) ;
            if ( validAruco ) {
                transform     = pArucoPlane->getTransform().cast<float>();
                transform_inv = transform.inverse();
                pArucoPlane->drawingAxis(rgb_temp);
            } else {
                cout << "WARNING:  No Aruco Plane been Viewed !" << endl;
            }
        }

        vector<pcl::PointXYZRGBA, Eigen::aligned_allocator<pcl::PointXYZRGBA>>().swap(cloud_scene->points);
    	cloud_scene->clear();


        imshow("color", rgb_temp);
        imshow("depth", depth_image);
        waitKey(1);

        
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

                if ( useQrExtractor && validAruco) {
                    Eigen::Vector4f scene_point;
                    scene_point << scenePoint(0),scenePoint(1),scenePoint(2),1;
                    Eigen::Matrix<float,4,1> world_point;
                    world_point = transform_inv * scene_point;

                    if( (abs(world_point(0)) > cubeSide/2)  ||  // outside of cube bundary
                        (abs(world_point(1)) > cubeSide/2) ||
                        (world_point(2) > cubeSide) ||
                        (world_point(2) < 0.0f))
                    {
                        continue;
                    }
                }

                Eigen::Vector4f scene_point;
                scene_point << scenePoint(0),scenePoint(1),scenePoint(2),1;
                Eigen::Matrix<float,4,1> world_point;
                world_point = transform_inv * scene_point;

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
        cloud_scene->width  = cloud_scene->points.size();
        cloud_scene->resize ( cloud_scene->height * cloud_scene->width );
        extractPlaneFeatureFromCloud(cloud_scene, cloud_scene);

        // ##  VERY TIME CONSUMING !!! 
        // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
        // sor.setInputCloud(cloud_scene);
        // sor.setMeanK(50);   // mean value
        // sor.setStddevMulThresh(1.0);
        // sor.filter(*cloud_scene);

        chrono::steady_clock::time_point t6 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t6-t3 );
        // cout << "##### time used by processing pointclouds: " << time_used.count() << endl;


        viewer.removeAllPointClouds();
        viewer.addPointCloud ( cloud_scene, "cloud_scene" );
        viewer.addCoordinateSystem ( 1.0 );
        viewer.spinOnce();

       if( kbhit())   //space down
       {
           char c = getchar();
           if(c == KEYCODE_ESC)
           {
               cout<<"\n Finishing grab data ..."<<endl;
               viewer.close();
                
               ros::shutdown();
               break;
           } else {
                
               cout<<"saving data..." << endl;
               if (recordingTF) {
                   calibtool::saveStampedImgPointcloudTFPose(cloud_scene,
                       rgb_temp, tfTransform, fileSavedCounter, robotPoseOutput, dataOutputDir);
               
               } else {
                   calibtool::saveStampedImgPointcloud(cloud_scene, rgb_temp, fileSavedCounter, dataOutputDir);
               }

               fileSavedCounter++;
           }

       }

       loop_rate.sleep();
        // ros::spinOnce();
        
    }

    return 0;

}


//##############################################################################################


/**
 * @brief extractPlaneFeatureFromCloud
 * @param cloud_source
 * @param cloud_destin
 */

void extractPlaneFeatureFromCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud_source,
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud_destin)
{
    // creat the filtering object
    pcl::ModelCoefficients::Ptr coefficient(new pcl::ModelCoefficients());
    // creat the inliers indices object
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    // create the extract object
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud_source);
    seg.segment(*inliers, *coefficient);

    extract.setInputCloud(cloud_source);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_destin);
}

int getch() {
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt); // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);               // disable buffering
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // apply new settings

    int c = getchar(); // read character (non-blocking)

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore old settings
    return c;
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

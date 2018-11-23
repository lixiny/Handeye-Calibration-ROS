#include "common_include.h"
#include "calibration_tool.h"
#include "aruco_plane.h"
#include "rgbd_srv/rgbd.h"

using namespace std;
using namespace cv;

#define KEYCODE_SPACE   0x20
#define KEYCODE_ESC     0x1B

string  robotPoseOutput;
string  cameraIntrinsicInput;
string  EETFname;
string  baseTFname;
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

void extractPlaneFeatureFromCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud_source,
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud_destin);
int kbhit(void);

// ################################### main function ###########################################
int main(int argc, char** argv)
{
    cout << "\n\n";

    ros::init ( argc, argv, "grab_data_node" );
    ros::NodeHandle nh("~");

    nh.param("useQrExtractor", useQrExtractor, false);
    nh.param("recordingTF", recordingTF, false);
    nh.param("robotPoseOutput" , robotPoseOutput, std::string("robot_pose.xml"));
    nh.param("cameraIntrinsicInput", cameraIntrinsicInput, std::string("camera_intrinsic_color.xml"));
    nh.param("numOfQRcodeOnSide", numOfQRcodeOnSide, 0);
    nh.param("EETF", EETFname, std::string("/ee_link"));
    nh.param("baseTF", baseTFname, std::string("/base_link"));

    cout << useQrExtractor <<" " << recordingTF << " " << robotPoseOutput << " " << cameraIntrinsicInput <<endl;

    cout << "Reading Camera Intrinsic File from : " << cameraIntrinsicInput << endl;

    if (! calibtool::initCameraIntrinsic(cameraIntrinsicInput, cameraMatrix, distCoeffs)) {
        cerr << "cannot locate [camera_intrinsics] XML file || [camera_intrinsics] XML file contains invalid data. Program will exit." << endl;
        return -1;
    }

    if(useQrExtractor) {
        if(numOfQRcodeOnSide != 6) {
            cerr << "wrong settings to Aruco plate! Program will exit. try to set [numOfQRcodeOnSide] to 0 in [.launch] instead)" << endl;
            return -2;
        }
    }

    // if (argc == 2) {
    //     usage = USAGE::STANDALONE;
    //     cout << "USAGE::STANDALONE" << endl;
    // } else if (argc == 3) {  
    //     numOfQRcodeOnSide = atoi(argv[2]);
    //     if (numOfQRcodeOnSide == 0) {
    //         usage = USAGE::STANDALONE;
    //         cout << "USAGE::STANDALONE" << endl;
    //     } else {
    //         usage = USAGE::WITH_QR_EXTRACTOR;
    //         cout << "USAGE::WITH_QR_EXTRACTOR" << endl;
    //         qrFLAG = true;
    //     }
    // } else if (argc == 4) {
    //     usage = USAGE::WITH_ROBOT_TF_POSE;
    //     cout << "USAGE::WITH_ROBOT_TF_POSE" << endl;
    //     robot_end_link = argv[2];
    //     robot_base_link = argv[3];
    //     tfFLAG = true;
    // } else if (argc == 5) { 
    //     numOfQRcodeOnSide = atoi(argv[2]);
    //     robot_end_link = argv[3];
    //     robot_base_link = argv[4];
    //     tfFLAG = true;
    //     if (numOfQRcodeOnSide == 0) {
    //         usage = USAGE::WITH_ROBOT_TF_POSE;
    //         cout << "USAGE::WITH_ROBOT_TF_POSE" << endl;
    //     } else {
    //         usage = USAGE::WITH_QR_EXTRACTOR_AND_ROBOT_TF_POSE;
    //         cout << "USAGE::WITH_QR_EXTRACTOR_AND_ROBOT_TF_POSE" << endl;
    //         qrFLAG = true;
    //     }
    // } else {

    //     cerr << "Arguments ERROR!" << endl;
    //     cerr << "Usage:  rosrun  sphere_handeye  grab_data_node  [camera_intrinsics_file].xml \n";
    //     cerr << "Or   :  rosrun  sphere_handeye  grab_data_node  [camera_intrinsics_file].xml  [#_of_qr_codes_on_each_side? 0 if_none] \n";
    //     cerr << "Or   :  rosrun  sphere_handeye  grab_data_node  [camera_intrinsics_file].xml  [robot_end_link]  [robot_base_link]  \n";
    //     cerr << "Or   :  rosrun  sphere_handeye  grab_data_node  [camera_intrinsics_file].xml  [#_of_qr_codes_on_each_side? 0 if_none]  [robot_end_link]  [robot_base_link] \n";
    //     return -1;
    // }

    cout << "Init Complete" << endl;
    int fileSavedCounter = 1;

    Mat rgb_image, rgb_temp;
    Mat depth_image;
    ros::ServiceClient client = nh.serviceClient<rgbd_srv::rgbd> ( "realsense2_server" ); // bind client to server
    rgbd_srv::rgbd srv;   // serivce type
    srv.request.start = true;
    sensor_msgs::Image msg_rgb;
    sensor_msgs::Image msg_depth;

    tf::TransformListener tfListener;
    tf::StampedTransform  tfTransform;

    ros::Rate loop_rate ( 60 );

    pcl::visualization::PCLVisualizer viewer ( "3D viewer" );
    viewer.setBackgroundColor ( 40.0/255,40.0/255,52.0/255 );

    handeye::ArucoPlane::Ptr arucoPlane;

    float cubeSide = 0;

    if( useQrExtractor ) 
    {

        //################## cube extractor #######################
        handeye::ArucoPlane::Ptr arucoPlaneTemp ( new handeye::ArucoPlane(numOfQRcodeOnSide) );
        arucoPlane = arucoPlaneTemp;
        cubeSide   = arucoPlane->getCubeSide();
        arucoPlane->setCameraIntrinsic(cameraMatrix, distCoeffs);
        cout << "using aruco plane " <<endl;
        //#########################################################
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_scene ( new pcl::PointCloud<pcl::PointXYZRGBA>() );


    while( ros::ok() ) 
    {
        cout << "calling service" << endl;
        if ( client.call( srv )) { // client send request(call server), when success, return a bool true
            cout << "calling service success" << endl;
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
                    cout << "Fatal: current no image data!  \n" << endl;
                    continue;
                }

            } catch ( cv_bridge::Exception& e ) {
                ROS_ERROR ( "cv_bridge execption: %s", e.what() );
                return -1;
            }
            /** ---------------------------------------------------------------------------------------- */
            if ( recordingTF ) {
                try
                {
                    /** get the latest tfTransform from robot_base_link(source) to robot_end_link(target) */
                    tfListener.lookupTransform ( EETFname, baseTFname, ros::Time ( 0 ), tfTransform );  
                    
                } catch ( tf::TransformException ex ) {
                    ROS_ERROR("%s", ex.what());
                    cout << "Warning: current may not have tf data! " << endl;
                    continue;
                }
            }


        }

        if (rgb_temp.empty() || depth_image.empty()) {
            continue;
        }

        cout << "showing image" << endl;
        imshow("color", rgb_temp);
        imshow("depth", depth_image);
        waitKey(1);


        Eigen::Matrix4f transform, transform_inv;

        if ( useQrExtractor ) 
        {

            if ( !arucoPlane->calculateExtrinsicFromImage( rgb_image ) ) {
                cout << "3" << endl;
                continue;
            }
            transform     = arucoPlane->getTransform().cast<float>();
            transform_inv = transform.inverse();
            arucoPlane->drawingAxis(rgb_temp);
        }
        
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


                if ( useQrExtractor ) {
                    Eigen::Vector4f scene_point;
                    scene_point << scenePoint(0),scenePoint(1),scenePoint(2),1;
                    Eigen::Matrix<float,4,1> world_point;
                    world_point = transform_inv * scene_point;

                    if( (abs(world_point(0)) > cubeSide/2)  ||  // outside of cube bundary
                        (abs(world_point(1)) > cubeSide/2) ||
                        (world_point(2) > cubeSide) ||
                        (world_point(2) < -0.05f))
                    {
                        continue;
                    }
                }
                // Eigen::Vector4f scene_point;
                // scene_point << scenePoint(0),scenePoint(1),scenePoint(2),1;
                // Eigen::Matrix<float,4,1> world_point;
                // world_point = transfrom_inv * scene_point;

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
                cout<<"\n Finishing grab data ..."<<endl;
                viewer.close();
                
                ros::shutdown();
                break;
            } else {
                
                cout<<"saving data..." << endl;
                if (recordingTF) {
                    calibtool::saveStampedImgPointcloudTFPose(cloud_scene, 
                        rgb_temp, tfTransform, fileSavedCounter, robotPoseOutput);
                } else {
                    calibtool::saveStampedImgPointcloud(cloud_scene, rgb_temp, fileSavedCounter);
                }

                fileSavedCounter++;
            }

        }

        // ros::spinOnce();
        loop_rate.sleep();
        
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

#include "CommonInclude.h"
#include "CalibrationTool.h"
#include "ArucoPlane.h"
#include "rgbd_srv/rgbd.h"

using namespace std;
using namespace cv;

#define KEYCODE_SPACE   0x20
#define KEYCODE_ESC     0x1B
#define KEYCODE_s       0x73
#define KEYCODE_S       0x53

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


// ################################### function prototype ############################################

int kbhit(void);
int getch();
void concatImage(cv::Mat& img1, cv::Mat& img2, cv::Mat& des);

bool pointsOutCube(Eigen::Matrix<float,4,1>& pt, float cubeSideLen)
{
    return (  (abs(pt[0]) > cubeSideLen/2)
              || (abs(pt[1]) > cubeSideLen/2)
              || (pt[2] > cubeSideLen)
//              || (pt[2] < 0.03f)
            );
}
// ################################### main function ###########################################
int main(int argc, char** argv)
{
    cout << "\n\n";

    ros::init ( argc, argv, "grab_data_node" );
    ros::NodeHandle nh("~");

    nh.param("useQrExtractor", useQrExtractor, false);
    nh.param("recordingTF", recordingTF, false);
    nh.param("cameraIntrinsicInput", cameraIntrinsicInput, std::string("./camera_intrinsic_color.xml"));
    nh.param("numOfQRcodeOnSide", numOfQRcodeOnSide, 0);
    nh.param("EETF", EETFname, std::string("/ee_link"));
    nh.param("baseTF", baseTFname, std::string("/base_link"));
    nh.param("RGBDserviceName",  RGBDserviceName,  std::string("realsense2_server"));
    nh.param("dataOutputDir", dataOutputDir, std::string("./dataset") );
    nh.param("robotPoseOutput" , robotPoseOutput, std::string("./dataset/robot_pose.xml"));

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
    int fileSavedCounter = 0;

    Mat rgb_image, rgb_temp;
    Mat depth_image;
    Mat perview;
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
    viewer.addCoordinateSystem ( 0.1 );
    viewer.setCameraPosition(-0.2,-0.2,-0.8, 0, 0, 0, 0, -1, 0);
    viewer.setBackgroundColor ( 40.0/255,40.0/255,52.0/255 );

    handeye::ArucoPlanePtr pArucoPlane;

    float cubeSide = 0;
    bool  validAruco = false;
    bool  validArucoLast = false;
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
                tfListener.lookupTransform ( baseTFname, EETFname, ros::Time ( 0 ), tfTransform );

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
                pArucoPlane->drawingCube(rgb_temp);
                if (!validArucoLast)
                    cout <<"\e[1;35m" << "Valid Aruco Segmentation"<< "\e[0m" << endl;;
            } else {
                if (validArucoLast)
                    cout <<"\e[1;33m" << "No Aruco Found" << "\e[0m" << endl;
            }
            validArucoLast = validAruco;
        }

        vector<pcl::PointXYZRGBA, Eigen::aligned_allocator<pcl::PointXYZRGBA>>().swap(cloud_scene->points);
        cloud_scene->clear();
        concatImage(rgb_temp, depth_image, perview);
        imshow("perview", perview);
        waitKey(1);

        if(useQrExtractor && !validAruco) {
            viewer.spinOnce();
            continue;
        }


        for (int r=0;r<depth_image.rows;r++)
        {
            for (int c=0;c<depth_image.cols;c++)
            {
                pcl::PointXYZRGBA p;
                if ( ! ( depth_image.ptr<float> ( r ) [c] > 0 )
                     || depth_image.ptr<float> ( r ) [c] > 2.0 )
                {
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

                    if( pointsOutCube(world_point, cubeSide)) continue;
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
            } else if (c == KEYCODE_s || c == KEYCODE_S ) {

                cout<<"...... saving data ...... #" << fileSavedCounter << endl;
                cloud_scene->height = 1;
                cloud_scene->width  = cloud_scene->points.size();
                cloud_scene->resize ( cloud_scene->height * cloud_scene->width );
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

void concatImage(cv::Mat& clr, cv::Mat& dep32f, cv::Mat& des) {
    int height = clr.rows;
    int width = clr.cols;
    Mat gray = Mat(dep32f.rows, dep32f.cols, CV_8UC1);
    float max = 2.0;
    float min = 0.5;
    for (int r=0;r<dep32f.rows;r++)
    {
        for (int c=0;c<dep32f.cols;c++)
        {
            float d = dep32f.ptr<float> ( r ) [c];
            if (d<max && d>min) {//max,min表示需要的深度值的最大/小值
                gray.ptr<uchar>(r)[c] = (max - d) / (max - min) * 255.0f;
            }else{
                gray.ptr<uchar>(r)[c] = 0;
            }

        }
    }
    applyColorMap(gray, gray, COLORMAP_JET);
    des.create(height, width * 2, clr.type());
    Mat r1 = des(Rect(0, 0, width, height));
    clr.copyTo(r1);
    Mat r2 = des(Rect(width, 0, width, height));
    gray.copyTo(r2);
}


/**
 * @brief extractPlaneFeatureFromCloud
 * @param cloud_source
 * @param cloud_destin
 */

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

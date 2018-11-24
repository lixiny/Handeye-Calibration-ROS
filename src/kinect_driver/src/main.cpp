#include "OpenNI2Interface.h"
#include "CameraInterface.h"
//ros
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
//pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace cv;

int const OUTPUT_WIDTH      = 640;
int const OUTPUT_HEIGHT     = 480;
int const FPS               = 30;
float const depth_scale = 1;
float const fx = 570.3;
float const fy = 570.3;
float const cx = 319.5;
float const cy = 239.5;

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;

int main(int argc,char **argv)
{
    ros::init(argc, argv, "kinect_driver");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub;
    image_transport::Publisher image_pub_depth;
    image_pub = it.advertise("/kinect/rgb", 1);
    image_pub_depth = it.advertise("/kinect/depth", 1);
    ros::Publisher cloud_pub = nh.advertise<PointCloud> ("/kinect/cloud", 1);
    PointCloud::Ptr cloud_scene (new PointCloud);
    Mat src_rgb_image = cv::Mat::zeros(OUTPUT_HEIGHT,OUTPUT_WIDTH,CV_8UC3);
    Mat rgb_image = cv::Mat::zeros(OUTPUT_HEIGHT,OUTPUT_WIDTH,CV_8UC3);
    Mat depth_image = cv::Mat::zeros(OUTPUT_HEIGHT,OUTPUT_WIDTH,CV_16UC1);

    std::cout << "START!Creating live capture... "; std::cout.flush();

    CameraInterface * cam;
    cam = new OpenNI2Interface(OUTPUT_WIDTH,OUTPUT_HEIGHT,FPS);

    if(!cam || !cam->ok())
    {
        std::cout << "failed!" << std::endl;
        std::cout << cam->error();
    }
    else
    {
        std::cout << "success!" << std::endl;

        std::cout << "Waiting for first frame"; std::cout.flush();

        int lastDepth = cam->latestDepthIndex.getValue();

        do
        {
            std::this_thread::sleep_for(std::chrono::microseconds(33333));
            std::cout << "."; std::cout.flush();
            lastDepth = cam->latestDepthIndex.getValue();
        } while(lastDepth == -1);

        cout<<"OK!";
        cout << "\n\n";
        cout << "########### camera_intrinsics ###########" << endl;
        cout << "width:      " << OUTPUT_WIDTH << endl;
        cout << "height:     " << OUTPUT_HEIGHT << endl;
        cout << "cx:         " << cx << endl;
        cout << "cy:         " << cy << endl;
        cout << "fx:         " << fx  << endl;
        cout << "fy:         " << fy  << endl;
        cout << "########### camera_intrinsics ###########" << endl;
        cout << "\n";
        cout << "########### camera_state ###########" << endl;
        cout << "fps:               " << FPS << endl;
        cout << "auto exposure:     "<< cam->cameraAutoExposure <<endl;
        cout << "auto white balance:"<< cam->cameraAutoWhiteBalance <<endl;
        cout << "########### camera_state ###########" << endl;
        cout << "\n";
        cout << "Kinect is running!"<<"\n\n";
    }

    ros::Rate loop_rate(60);
    while(ros::ok())
    {
    	// memcpy(rgb_image.data, cam->frameBuffers[0].first.second, OUTPUT_WIDTH * OUTPUT_HEIGHT * 3);
        // memcpy(depth_image.data, cam->frameBuffers[0].first.first, OUTPUT_WIDTH * OUTPUT_HEIGHT * 2);
        src_rgb_image = cv::cvarrToMat(cam->_rgb_image);
        depth_image = cv::cvarrToMat(cam->_depth_image);

        cv::Mat depth32f(OUTPUT_HEIGHT,OUTPUT_WIDTH,CV_32F);

    	cv::cvtColor(src_rgb_image, rgb_image, CV_BGR2RGB);

        for(int y = 0; y < depth_image.rows; y++) {
            for(int x = 0; x < depth_image.cols; x++) {
                auto pixels_distance = depth_image.at<short>(y,x) * depth_scale/1000;
                depth32f.ptr<float>(y)[x] = pixels_distance;
            }
        }

        cloud_scene->clear();
        for (int r=0;r<OUTPUT_HEIGHT;r++)
        {
            for (int c=0;c<OUTPUT_WIDTH;c++)
            {
                if (!(depth_image.ptr<short>(r)[c] > 0.f)) 
                {
                    continue;   // no depth at current point
                }
                pcl::PointXYZRGBA p;
                uint16_t depth_value = depth_image.at<short>(r,c);
                float depth_in_meter = depth_value * depth_scale/1000;
                double scene_z = double(depth_in_meter);
                double scene_x = (c - cx)*scene_z / fx;
                double scene_y = (r - cy)*scene_z / fy;
                p.x = scene_x;
                p.y = scene_y;
                p.z = scene_z;
                p.r = rgb_image.ptr<uchar>(r)[c*3+2];
                p.g = rgb_image.ptr<uchar>(r)[c*3+1];
                p.b = rgb_image.ptr<uchar>(r)[c*3];
                cloud_scene->points.push_back(p);
            }
        }

        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "camera_joint";

        cv_bridge::CvImage cv;
        cv.header = header;
        cv.image = rgb_image;
        cv.encoding = sensor_msgs::image_encodings::BGR8;

        cv_bridge::CvImage cv_depth;
        cv_depth.header = header;
        cv_depth.image = depth32f;
        cv_depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

        image_pub.publish(cv.toImageMsg());
        image_pub_depth.publish(cv_depth.toImageMsg());

        // cloud_scene->header.stamp = ros::Time::now().toSec();
        cloud_scene->header.frame_id = "camera_joint";
        cloud_pub.publish(cloud_scene);

        // imshow("rgb",rgb_image);
        // imshow("depth",depth_image);
        // waitKey(1);

        ros::spinOnce();
        loop_rate.sleep();
    }

    delete cam;
    return 0;
    
}

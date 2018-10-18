//ros
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Transform.h>

// realsense2
#include <librealsense2/rs.hpp>

//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

//pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;

// Window size and frame rate
int const OUTPUT_WIDTH      = 640;
int const OUTPUT_HEIGHT     = 480;
int const FRAME_RATE        = 30;

float get_depth_scale(rs2::device dev);

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);

bool profile_changed(const std::vector<rs2::stream_profile>& current,
                     const std::vector<rs2::stream_profile>& prev);



//########################################################################################

int main(int argc, char** argv)
{
    ros::init(argc, argv, "realsense_driver");
    ros::NodeHandle nh;
    ros::Rate loop_rate(FRAME_RATE);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub_color;
    image_transport::Publisher image_pub_depth;
    ros::Publisher cloud_pub;

    image_pub_color = it.advertise("/realsense_sr300/ylx/rgb", 1);
    image_pub_depth = it.advertise("/realsense_sr300/ylx/depth", 1);
    cloud_pub       = nh.advertise<PointCloud>("/realsense_sr300/ylx/cloud",1);

    // Create a pipeline to easily configure and start the camera
    rs2::pipeline pipe;
    //Calling pipeline's start() without any additional parameters will start the first device
    // with its default streams.
    //The start function returns the pipeline profile which the pipeline used to start the device

    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, OUTPUT_WIDTH, OUTPUT_HEIGHT, RS2_FORMAT_BGR8, FRAME_RATE);
    config.enable_stream(RS2_STREAM_DEPTH, OUTPUT_WIDTH, OUTPUT_HEIGHT, RS2_FORMAT_ANY, FRAME_RATE);
    rs2::pipeline_profile profile = pipe.start(config);
    //   rs2::pipeline_profile profile = pipe.start(); // this is the default config, cause some delay.
    rs2_intrinsics intrinsics;
    rs2::colorizer color_map;

    // Each depth camera might have different units for depth pixels, so we get it here
    // Using the pipeline's profile, we can retrieve the device that the pipeline uses
    float depth_scale = get_depth_scale(profile.get_device());

    // These units are expressed as depth in meters corresponding to a depth value of 1
    // For example if we have a depth pixel with a value of 2 and the depth scale units are 0.5
    // then that pixel is 2 X 0.5 = 1 meter away from the camera.

    const std::vector<rs2::stream_profile>& streams = profile.get_streams();
    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();

        if (profile_stream == RS2_STREAM_COLOR)
        {
            
            rs2::video_stream_profile vsp(sp);
            intrinsics = vsp.get_intrinsics();
            cout << "\n\n";
            cout << "########### camera_intrinsics ###########" << endl;
            cout << "width:      " << intrinsics.width << endl;
            cout << "height:     " << intrinsics.height << endl;
            cout << "px:         " << intrinsics.ppx << endl;
            cout << "py:         " << intrinsics.ppy << endl;
            cout << "fx:         " << intrinsics.fx  << endl;
            cout << "fy:         " << intrinsics.fy  << endl;
            cout << "disCoeff:   ";
            cout << intrinsics.coeffs[0] << " "
                                         << intrinsics.coeffs[1] << " "
                                         << intrinsics.coeffs[2] << " "
                                         << intrinsics.coeffs[3] << " "
                                         << intrinsics.coeffs[4] << " " << endl;
            cout << "########### camera_intrinsics ###########" << "\n\n";
        }

    }

    //Pipeline could choose a device that does not have a color stream
    //If there is no color stream, choose to align depth to another stream
    rs2_stream align_to = find_stream_to_align(profile.get_streams());  //color stream
    
    // Create a rs2::align object.
    // rs2::align allows us to perform alignment of depth frames to others frames
    // The "align_to" is the stream type to which we plan to align depth frames.
    rs2::align align(align_to);

    cout << "Depth Scale:          " << depth_scale << endl;
    cout << "Frame Rate:           " << FRAME_RATE << " frame/sec" << endl;
    cout << "RGB Encoding:         " << "RGB8" << endl;
    cout << "Depth Encoding:       " << "TYPE_32FC1 (meter)" << "\n\n";
    cout << "Camera is ready, Everyone is happy NOW !" << endl;



    // setup a unRotated unTranslated Transformation between "camera_joint" and "camera_optical_frame" 
    tf::TransformBroadcaster br;
    tf::Transform transform_b;
    transform_b.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    transform_b.setRotation( tf::Quaternion(0, 0, 0, 1) );
    //initiate tf
    br.sendTransform(tf::StampedTransform(transform_b, ros::Time::now(),
                                          "camera_joint", "camera_optical_frame"));


    PointCloud::Ptr cloud_scene (new PointCloud);                                     


    while(ros::ok()) {
        rs2::frameset frameset = pipe.wait_for_frames();

        if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
        {
            //If the profile was changed, update the align object, and also get the new device's depth scale
            profile = pipe.get_active_profile();
            align_to = find_stream_to_align(profile.get_streams());
            align = rs2::align(align_to);
            depth_scale = get_depth_scale(profile.get_device());
        }

        //Get processed aligned frame
        auto proccessed = align.process(frameset);

        //Trying to get both color and aligned depth frames
        rs2::depth_frame aligned_depth_frame = proccessed.get_depth_frame();
        // rs2::frame color_depth = color_map(aligned_depth_frame);
        rs2::video_frame color_frame = proccessed.first(align_to);
        // rs2::video_frame color_frame = proccessed.get_color_frame();

        if (!aligned_depth_frame || !color_frame)
        {
            continue;
        }

        uint16_t* p_depth_frame = reinterpret_cast<uint16_t*>(const_cast<void*>(aligned_depth_frame.get_data()));
        uint8_t* p_color_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(color_frame.get_data()));
        // uint8_t* p_color_depth = reinterpret_cast<uint8_t*>(const_cast<void*>(color_depth.get_data()));
        // const uint16_t* p_depth_frame = reinterpret_cast<const uint16_t*>(aligned_depth_frame.get_data());

        cv::Mat bgr(color_frame.get_height(), color_frame.get_width(), CV_8UC3, p_color_frame);
        cv::Mat rgb;
        cvtColor( bgr, rgb, cv::COLOR_BGR2RGB );
        // cv::Mat rgb_depth(color_frame.get_height(), color_frame.get_width(), CV_8UC3, p_color_depth);

        cv::Mat depth16(aligned_depth_frame.get_height(),
                        aligned_depth_frame.get_width(),
                        CV_16U,
                        p_depth_frame);
    
        int width = color_frame.get_width();
        int height = color_frame.get_height();
        cv::Mat depth32f(OUTPUT_HEIGHT,OUTPUT_WIDTH,CV_32F);
        #pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
        for (int y = 0; y < height; y++)
        {
            auto depth_pixel_index = y * width;
            for (int x = 0; x < width; x++, ++depth_pixel_index)
            {
                // Get the depth value of the current pixel
                auto pixels_distance = depth_scale * p_depth_frame[depth_pixel_index];

                // Check if the depth value is invalid (<=0) or greater than the threashold
                depth32f.ptr<float>(y)[x] = pixels_distance;
            }
        }

		/**
        cv::imshow("rgb", rgb);
        cv::imshow("depth",depth32f);
        cv::waitKey(1);
		*/

        cloud_scene->clear();
        for (int r = 0; r < depth32f.rows; r++) {
			for (int c = 0; c < depth32f.cols; c++) {
				pcl::PointXYZRGBA p;
				if (!(depth32f.ptr<float>(r)[c] > 0.f)) 
				{
					continue;   // no depth at current point
				}
				
				if ((depth32f.ptr<float>(r)[c] >= 1.f))   // don't want depth large than 1 meter
				{
					continue;
				}
				float scene_z = float(depth32f.ptr<float>(r)[c]);  // z direction
				float scene_y = (r - intrinsics.ppy) * scene_z / intrinsics.fy;   // pixel coord to camera coord
				float scene_x = (c - intrinsics.ppx) * scene_z / intrinsics.fx;
				
				p.x = scene_x;
				p.y = scene_y;
				p.z = scene_z;
				p.r = rgb.ptr<uchar>(r)[c * 3];
				p.g = rgb.ptr<uchar>(r)[c * 3 + 1];
				p.b = rgb.ptr<uchar>(r)[c * 3 + 2];
				cloud_scene->points.push_back(p);	
				
			}
		}

        cloud_scene->height = 1;
		cloud_scene->width = cloud_scene->points.size();
		cloud_scene->resize(cloud_scene->height * cloud_scene->width);


        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "camera_joint";

        cv_bridge::CvImage cv_color;
        cv_color.header = header;
        cv_color.image = rgb;
        cv_color.encoding = sensor_msgs::image_encodings::RGB8;

        cv_bridge::CvImage cv_depth;
        cv_depth.header = header;
        cv_depth.image = depth32f;
        cv_depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

        cloud_scene->header.stamp = ros::Time::now().toSec();
        cloud_scene->header.frame_id = "camera_joint";
        cloud_pub.publish(cloud_scene);

        image_pub_color.publish(cv_color.toImageMsg());
        image_pub_depth.publish(cv_depth.toImageMsg());

        br.sendTransform(tf::StampedTransform(transform_b, 
                                              ros::Time::now(), 
                                              "camera_joint", 
                                              "camera_optical_frame"));

        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}

//##################### helper function ###################################################


float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}


rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
    //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    //We prioritize color streams to make the view look better.
    //If color is not available, we take another stream that (other than depth)
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found)         //Prefer color
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR)
            {
                color_stream_found = true;
            }
        }
        else
        {
            depth_stream_found = true;
        }
    }

    if(!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}



bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
    for (auto&& sp : prev)
    {
        //If previous profile is in current (maybe just added another)
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
        if (itr == std::end(current)) //If it previous stream wasn't found in current
        {
            return true;
        }
    }
    return false;
}



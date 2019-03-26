// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    string imgInput = argv[1];
    cv::Mat img = cv::imread(imgInput);
    if (img.channels() != 1) {
        cvtColor(img, img, CV_RGB2GRAY);
    }

    cout << img.channels() << endl;
    int width = img.cols;
    int height = img.rows;

    int nWidth = 7;
    int nHeight = 9;

    int boardSizeW = width / nWidth;
    int boardSizeH = height / nHeight;

    cout << "hi" << endl;
    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {
            if( (i / boardSizeH + j / boardSizeW) % 2 != 0 ) {
                img.ptr<char>(i)[j] = 0;
            }
        }
    }

    cv::imwrite("img.jpg", img);
    return 0;
}

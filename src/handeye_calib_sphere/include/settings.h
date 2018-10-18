#ifndef SETTING_H
#define SETTING_H

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

class Settings
{
public:
    Settings()
    {

    }
    cv::Size boardSize;
    float squareSize;

};

#endif // SETTING_H

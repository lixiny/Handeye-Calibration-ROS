#pragma once

#include <string>
#include <iostream>
#include <algorithm>
#include <map>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ThreadMutexObject.h"

class CameraInterface
{
    public:
		virtual ~CameraInterface() {}

		virtual bool ok() = 0;
		virtual std::string error() = 0;

		static const int numBuffers = 1;
		bool cameraAutoExposure = true;
		bool cameraAutoWhiteBalance = true;
		ThreadMutexObject<int> latestDepthIndex;
		// std::pair<std::pair<uint8_t *,uint8_t *>,int64_t> frameBuffers[numBuffers];
		IplImage* _rgb_image;
		IplImage* _tmp_rgb_image;
		IplImage* _depth_image;

		virtual void setAutoExposure(bool value) = 0;
		virtual void setAutoWhiteBalance(bool value) = 0;
};

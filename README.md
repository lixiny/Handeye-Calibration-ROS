# Robotic Hand-eye Calibration Workspace 
| **`Ubuntu 1604 & ROS Kinetic Kame`** |  

This repo contains a eye-in-hand calibration tool (Cplusplus & ROS) in **JD京东 GRASPING ROBOT CHALLENGE ([News](http://me.sjtu.edu.cn/news/12692.html))**, and the implements of my paper: **Robotic hand-eye calibration with depth camera: A sphere model approach** (**[PDF](https://ieeexplore.ieee.org/document/8384652/)**)

Inside `/src` there are 5 ROS packages:    

* **rgbd_srv**   
used by camera_driver.  
* **camera_driver**  
drive Intel® RealSense™ RGBD cameras in ROS.   
(convert raw stream to ros `sensor_msgs::Image`)  
* **camera_transform_publisher**   
publish the transformation matrix (extrinsics) between camera and a chessboard. 
* **handeye_calib_chessboard**  
handeye calib tools that use RGB camera and chessboard, forked from [link](https://github.com/jhu-lcsr/handeye_calib_camodocal.git)   
**----------------------------------------------**   
with the package above you can calibrate the hand-eye transformation based on RGB images.   

* **handeye_calib_sphere**  
fine-tune the handeye result based on depth.  


## Prerequisit
* Ubunutu 18.04 
* ROS Melodic Morenia (desktop-full install) 
* with OpenCV & Opencv_contrib install in `/usr/local` (both `3.4.0`)
* librealsense if you use **Intel® RealSense™** RGBD cameras (D400 series and the SR300)
* visp_hand2eye_calibration_calibrator  
`sudo apt-get install ros-melodic-visp-hand2eye-calibration`  
* glog at [here](https://github.com/google/glog/releases) (`0.4.0`)
* Ceres-solver download at [here](https://github.com/ceres-solver/ceres-solver/releases) (`1.14.0`) and instruction at [here](http://ceres-solver.org/installation.html) 
* Sophus at [here](https://github.com/strasdat/Sophus) (`1.0.0`)
* ros-melodic-cv-bridge *   
`sudo apt-get install ros-melodic-cv-bridge`



### Install opencv & opencv_contrib
Download [opencv](https://github.com/opencv/opencv/releases) and [opencv_contrib](https://github.com/opencv/opencv_contrib/releases) (both tested on `3.4.0`).   
Install opencv prerequisit:
```
[compiler] sudo apt-get install build-essential
[required] sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
[optional] sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
```
Build opencv & opencv_contrib from source: (we turned off the cuda options)
```
mkdir build && cd build 
cmake \
    -D CMAKE_BUILD_TYPE=Release \
    -D OPENCV_EXTRA_MODULES_PATH= <path/to/opencv_contrib-3.4.0>/modules/ \
    -D BUILD_opencv_cudacodec=OFF \
    -D WITH_CUDA=OFF \
    -D WITH_CUBLAS=OFF \
    -D WITH_CUFFT=OFF \
    -D ENABLE_PRECOMPILED_HEADERS=OFF \
    -D CMAKE_INSTALL_PREFIX=/usr/local ..

make -j5
sudo make install
```

### Install librealsene (if you use Intel® RealSense™ RGBD Camera)
We use SR300/D415 camera and use `camera_driver` package to convert raw RGBD images into ROS topic: `sensor_msgs::Image`. The `librealsense` is required only if you use cameras from RealSense family, otherwise make your own `camera_driver` in order to use other cameras.

Following the librealsense installation guide at [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md).  
We installed: `librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg`

## Build 
```
git clone https://github.com/lixiny/Handeye-Calibration-ROS.git
cd Handeye-Calibration-ROS  
catkin_make
```  

if you only want to calibrate hand-eye transformation based on RGB input, 
```
git clone https://github.com/lixiny/Handeye-Calibration-ROS.git  
cd Handeye-Calibration-ROS  
git checkout rgb_only
catkin_make
```    


## Ready to Use
example on RealSense D415.

1. pluged in 







## Related Publications:    

Yang, Lixin, et al. " **Robotic hand-eye calibration with depth camera: A sphere model approach.** " 2018 4th International Conference on Control, Automation and Robotics (ICCAR). IEEE, 2018. **[PDF](https://ieeexplore.ieee.org/document/8384652/)**

    @inproceedings{yang2018robotic,
      title={Robotic hand-eye calibration with depth camera: A sphere model approach},
      author={Yang, Lixin and Cao, Qixin and Lin, Minjie and Zhang, Haoruo and Ma, Zhuoming},
      booktitle={2018 4th International Conference on Control, Automation and Robotics (ICCAR)},
      pages={104--110},
      year={2018},
      organization={IEEE}
    }


## License
handeyeCalibWithDepthCamera is freely available for free non-commercial use, and may be redistributed under these conditions. Please, see the [license](LICENSE) for further details
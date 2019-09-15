# Robotic Hand-eye Calibration Workspace 
| **`Ubuntu 1604 & ROS Kinetic Kame`** |  

This repo contains my self-collected, self-developed hand-eye calibration tools (Cplusplus & ROS) in **JINGDONG GRASPING ROBOT CHALLENGE**, and the implements of my paper: **Robotic hand-eye calibration with depth camera: A sphere model approach** (**[PDF](https://ieeexplore.ieee.org/document/8384652/)**)

Inside `/src` there are four ROS packages:    

**1. camera_transform_publisher**  
publish the transformation matrix (relative pose) between camera and marker  
**2. handeye_calib_chessboard**  
common handeye calib tools that use RGB camera and chessboard  
**3. handeye_calib_sphere**  
self-developed fine-tune method tackling handeye calb with RGB-D camera  
**4. rgbd_srv** 
common ROS srv folder


## Prerequisit
* **Ubuntu 16.04** 
* **ROS Kinetic Kame**  // full install   
* **PCL 1.7+** (should be included in ROS Kinetic Kame)
* **OpenCV 3.3** (should be included in ROS Kinetic Kame)
* **cv_bridge**  
`sudo apt-get install ros-kinetic-cv-bridge`
* **visp_hand2eye_calibration_calibrator**  
`sudo apt-get install ros-kinetic-visp-hand2eye-calibration`
* **Eigen3**  (should be included in Ubutnu 16.04)  
`sudo apt-get install libeigen3-dev`
* **Sophus** (https://github.com/stonier/sophus)  // standard make and make install
* **Ceres** (http://ceres-solver.org/) // follow the installation guild


## Install
Standard ROS workspace `catkin_make` procedure; 
```Shell
cd ~/to/your/workspace
git clone https://github.com/lixiny/calib_ws.git 
cd calib_ws	
catkin_make
```


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
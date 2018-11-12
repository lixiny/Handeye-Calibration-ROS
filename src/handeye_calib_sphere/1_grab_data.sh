if [ ! -d "/dataset" ]; then
	mkdir dataset
	echo "make directory /dataset"
fi
if [ ! -d "robot_pose.xml" ]; then
	touch robot_pose.xml
	echo "touch file robot_pose.xml"
fi

## rosrun  sphere_handeye  grab_data_node  \
##				[camera_intrinsics_file].xml				相机内参文件的相对路径
##				[#_of_qr_codes_on_each_side? 0 if_none]     0表示不用二维码板辅助提取点云； 或输入每边多少个二维码，我用的是6x6的二维码板
##				[robot_end_link]  							机器人末端 tf_link 
##				[robot_base_link]							机器人基座 tf_link
##                                                  
rosrun handeye_calib_sphere grab_data_node camera_intrinsic_color.xml 6 /ee_link /base
wait 

echo "Now you have finished grabing data, type  ./2_do_calibration.sh  to continue"

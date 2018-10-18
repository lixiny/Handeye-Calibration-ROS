if [  -d "handeye_result.xml" ]; then
	rm -rf handeye_result.xml
	echo "remove previous file handeye_result.xml"
fi

rm -rf dataset/color_*

touch handeye_result.xml
rosrun  handeye_calib_sphere eyeinhand_calib_node
wait

rm -rf dataset
rm -rf robot_pose.xml

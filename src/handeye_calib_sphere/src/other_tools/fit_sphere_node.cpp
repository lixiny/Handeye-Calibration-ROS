#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <Eigen/Dense>
#include <math.h>

using namespace std;
typedef unsigned char uchar;

string file_directory = "dataset/";

int main(int argc, char** argv) 
{

    double threshold = 0.0005; // 0.5mm
    if (argc != 3 && argc != 2) {
        cerr<<"args error "<<endl;
        cerr<<"Usage:  rosrun sphere_handeye  fit_sphere_node  [pointcloud_filename].pcd  [distanse_threshold] \n";
        cerr<<"or   :  rosrun sphere_handeye  fit_sphere_node  [pointcloud_filename].pcd \n";
        return -1;
    }

    if (argc == 3) {
        threshold = stod(argv[2]);
        cout << "The user-defined threshold is: " << threshold << endl;
    } else {
        cout << "Using default threshold: " << threshold << endl;
    }

    string filename = argv[1];
    
    file_directory = file_directory + filename;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::PCDReader reader;
    reader.read<pcl::PointXYZRGBA>(file_directory, *cloud);

    std::vector<int> inliers;

    pcl::SampleConsensusModelSphere<pcl::PointXYZRGBA>::Ptr
        model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZRGBA> (cloud));

    pcl::RandomSampleConsensus<pcl::PointXYZRGBA> ransac (model_s);

    ransac.setDistanceThreshold (threshold);

    ransac.computeModel();
    Eigen::VectorXf sphereModel;
    ransac.getModelCoefficients(sphereModel);
    cout << "sphere model is: \n" << sphereModel.transpose() << endl;
    ransac.getInliers(inliers);
    pcl::copyPointCloud<pcl::PointXYZRGBA>(*cloud, inliers, *final);

    float x = sphereModel(0);
    float y = sphereModel(1);
    float z = sphereModel(2);
    float r = sphereModel(3);

    /** #### TODO ####
     *  算点云inliers 与模型标准 0.075 之间的误差
     */
    
    for (int i = 0; i < 360; i++) {
        for (int j = 0; j < 180; j++) {
            pcl::PointXYZRGBA p_model;
            float alpha = i * (360.0 / 360);  // degree 0 ~ 360
            float beta = j * (180.0 / 180);    // degree 0 ~ 180
            float alphaRad = alpha * M_PI/180;
            float betaRad =  beta * M_PI/180;

            p_model.x = x + r * sin(betaRad) * cos(alphaRad);
            p_model.y = y + r * sin(betaRad) * sin(alphaRad);
            p_model.z = z + r * cos(betaRad);
            p_model.b = (uchar)255;
            p_model.g = (uchar)102;
            p_model.r = (uchar)102;
            final->points.push_back(p_model);

        }
    }

    final->height = 1;
    final->width = final->points.size();
    final->resize ( final->height * final->width );

	pcl::visualization::PCLVisualizer viewer("find_sphere");
    viewer.addPointCloud(final, "find_sphere");
    viewer.setBackgroundColor(40.0/255,40.0/255,52.0/255);  
	viewer.addCoordinateSystem(0.1);
	while (!viewer.wasStopped())
    { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce();
    }
    return 0;

}

#include "CalibrationTool.h"

using namespace std;
using namespace cv;
typedef unsigned char uchar;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;


void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt,  Eigen::Matrix4f &final_transform, bool downsample = false)
{
    //
    // Downsample for consistency and speed
    // \note enable this for large datasets
    PointCloud::Ptr src (new PointCloud);
    PointCloud::Ptr tgt (new PointCloud);
    pcl::VoxelGrid<PointT> grid;
    if (downsample)
    {
        grid.setLeafSize (0.05, 0.05, 0.05);
        grid.setInputCloud (cloud_src);
        grid.filter (*src);

        grid.setInputCloud (cloud_tgt);
        grid.filter (*tgt);
    }
    else
    {
        src = cloud_src;
        tgt = cloud_tgt;
    }

    //
    // Align
    pcl::IterativeClosestPoint<PointT, PointT> reg;
    reg.setTransformationEpsilon (1e-9);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance (1.5f);
    // Set the point representation

    reg.setInputSource (src);
    reg.setInputTarget (tgt);

    //
    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev;
    PointCloud::Ptr reg_result = src;
    reg.setMaximumIterations (30);
    for (int i = 0; i < 30; ++i)
    {
//        PCL_INFO ("Iteration Nr. %d.\n", i);

        // save cloud for visualization purpose
        src = reg_result;

        // Estimate
        reg.setInputSource (src);
        reg.align (*reg_result);

        //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation () * Ti;



        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
            reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.2);

        prev = reg.getLastIncrementalTransformation ();
        PCL_INFO ("Iteration Nr. %d. finished\n", i);


    }

    //
    // Get the transformation from target to source
    Eigen::Matrix4f targetToSource = Ti.inverse();
    cout << "target To Source: \n" << targetToSource.matrix() << "\n\n";

    final_transform = targetToSource;

}



int main() {

    string pcdfile = "./src/handeye_calib_sphere/dataset/pointcloud_06.pcd";
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_inliers (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::PCDReader reader;
    reader.read<pcl::PointXYZRGBA>(pcdfile, *cloud);

    float radius = 0.075f;
    Eigen::Vector3f center(0.0, 0.0, 0.0);
    calibtool::generateSphereFragment(model, center, radius);
    pcl::visualization::PCLVisualizer viewer("find_sphere");
    viewer.setBackgroundColor(40.0/255,40.0/255,52.0/255);
    viewer.addCoordinateSystem(0.1);


    std::vector<int> inliers;

    pcl::SampleConsensusModelSphere<pcl::PointXYZRGBA>::Ptr
            model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZRGBA> (cloud));

    pcl::RandomSampleConsensus<pcl::PointXYZRGBA> ransac (model_s);

    double threshold = 0.001; // 0.5mm
    ransac.setDistanceThreshold (threshold);

    ransac.computeModel();
    Eigen::VectorXf sphereModel;
    ransac.getModelCoefficients(sphereModel);


    ransac.getInliers(inliers);
    pcl::copyPointCloud<pcl::PointXYZRGBA>(*cloud, inliers, *cloud_inliers);


    *output += *cloud;
    *output += *model;


    Eigen::Matrix4f transform;
    pairAlign(cloud_inliers, model, transform);

    auto Transf = Eigen::Isometry3f(transform);

    auto translation = Transf.translation();
    Eigen::Vector3f newCenter(translation.x(), translation.y(), translation.z());
    cout << "sphere model from RANSAC is (x y z r): " << sphereModel.transpose() << endl;
    cout << "sphere center after ICP is (x y z): " << newCenter.transpose() << endl;


    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model2 (new pcl::PointCloud<pcl::PointXYZRGBA>);

    calibtool::generateSphereFragment(model2, newCenter, radius, false);

    *output += *model2;
    viewer.addPointCloud(output, "find_sphere");

    while (!viewer.wasStopped())
    { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce();
    }
    return 0;





}

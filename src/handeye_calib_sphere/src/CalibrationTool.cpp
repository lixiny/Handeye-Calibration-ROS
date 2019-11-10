#include "settings.h"
#include "CalibrationTool.h"
#include "CommonInclude.h"


using namespace std;
using namespace cv;

namespace calibtool
{


void generateSphereFragment(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model, Eigen::Vector3f& center, float r, bool fragment){
    for(int i = 0; i < 360; i++) {
        for (int j = 0; j < 180; j++) {
            float alpha = i * (360.0 / 360);  // degree 0 ~ 360
            float beta = j * (180.0 / 180);    // degree 0 ~ 180
            float alphaRad = alpha * M_PI/180;
            float betaRad =  beta * M_PI/180;

            pcl::PointXYZRGBA p_model;
            p_model.x = r * sin(betaRad) * cos(alphaRad);
            p_model.y = r * sin(betaRad) * sin(alphaRad);
            p_model.z = r * cos(betaRad);

            if(fragment) {
                if(p_model.x < (r - 2 * r / 3)) continue;
            }

            p_model.b = (uchar)255;
            p_model.g = (uchar)102;
            p_model.r = (uchar)102;

            p_model.x += center(0);
            p_model.y += center(1);
            p_model.z += center(2);
            model->points.push_back(p_model);

        }
    }

    model->height = 1;
    model->width = model->points.size();
    model->resize ( model->height * model->width );


}


void extractPlaneFeatureFromCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud_source,
                                  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud_destin)
{
    // creat the filtering object
    pcl::ModelCoefficients::Ptr coefficient(new pcl::ModelCoefficients());
    // creat the inliers indices object
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    // create the extract object
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud_source);
    seg.segment(*inliers, *coefficient);

    extract.setInputCloud(cloud_source);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_destin);
}

void deprojectPixelToPoint(Eigen::Vector3f& point, int pixel_coloum, int pixel_row, float depth, cv::Mat& cameraMatrix, cv::Mat& distCoeffs)
{
    float m_px, m_py, m_fx, m_fy, coeffs[5];
    m_fx = cameraMatrix.at<float>(0,0);
    m_fy = cameraMatrix.at<float>(1,1);
    m_px = cameraMatrix.at<float>(0,2);
    m_py = cameraMatrix.at<float>(1,2);
    for (int i = 0; i < 5; i++){
        coeffs[i] = distCoeffs.at<float>(0,i);
    }

    float x = (float(pixel_coloum) - m_px) / m_fx;
    float y = (float(pixel_row) - m_py) / m_fy;

    float r2  = x*x + y*y;
    float f = 1 + coeffs[0]*r2 + coeffs[1]*r2*r2 + coeffs[4]*r2*r2*r2;
    float ux = x*f + 2*coeffs[2]*x*y + coeffs[3]*(r2 + 2*x*x);
    float uy = y*f + 2*coeffs[3]*x*y + coeffs[2]*(r2 + 2*y*y);
    x = ux;
    y = uy;

    point(0) = depth * x;
    point(1) = depth * y;
    point(2) = depth;

}

void deprojectPixelToPoint2(Eigen::Vector3f& point, int pixel_coloum, int pixel_row, float depth, cv::Mat& cameraMatrix, cv::Mat& distCoeffs)
{
    float m_px, m_py, m_fx, m_fy, coeffs[5];
    m_fx = cameraMatrix.at<float>(0,0);
    m_fy = cameraMatrix.at<float>(1,1);
    m_px = cameraMatrix.at<float>(0,2);
    m_py = cameraMatrix.at<float>(1,2);
    for (int i = 0; i < 5; i++){
        coeffs[i] = distCoeffs.at<float>(0,i);
    }

    float x = (float(pixel_coloum) - m_px) / m_fx;
    float y = (float(pixel_row) - m_py) / m_fy;


    point(0) = depth * x;
    point(1) = depth * y;
    point(2) = depth;

}




void saveStampedImgPointcloud( const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud,
                               const cv::Mat& image,
                               const int& counter,
                               const string& dataOutputDir)
{
    stringstream ss;
    string numstr;
    ss << counter;
    ss >> numstr;
    if (counter <= 9) {
        numstr = "0" + numstr;
    }

    string imageFilename = dataOutputDir + "/" + "color_image_" + numstr + ".jpg";
    string pcdFilename = dataOutputDir + "/" + "pointcloud_" + numstr + ".pcd";

    cv::imwrite(  imageFilename, image);
    std::cerr << "Saved " << imageFilename << endl;
    pcl::io::savePCDFileASCII (  pcdFilename, *cloud );
    std::cerr << "Saved " << cloud->points.size() << " points to " << pcdFilename << endl;
    std::cerr << "**************************************" << endl;
}


void loadTFPoses(const string& robot_pose_filename,
                 cv::Mat& tfPoses)
{
    FileStorage fsread(robot_pose_filename, FileStorage::READ);
    fsread["robotposes"] >> tfPoses;
    fsread.release();
}


void TFToTransformMatrix(const tf::StampedTransform& tfpose, Eigen::Matrix4d& transform)
{
    tf::Quaternion quat ( 0,0,0,0 );
    tf::Vector3 transl ( 0,0,0 );
    quat = tfpose.getRotation();
    transl = tfpose.getOrigin();
    Eigen::Quaterniond q;
    q.x() = quat.x();
    q.y() = quat.y();
    q.z() = quat.z();
    q.w() = quat.w();
    Eigen::AngleAxisd rot_vec = Eigen::AngleAxisd(q);
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(rot_vec);
    T.pretranslate(Eigen::Vector3d (transl.x(), transl.y(), transl.z() ));
    transform = T.matrix();
}

string getCounterStampInString(const int& counter)
{
    // 定义最长为3位数字
    stringstream ss;
    string s = to_string(counter);
    ss << setw(3) << setfill('0') << s;
    ss >> s;
    return s;
}

/**
 * @brief save stamped image & Pointcloud & TFPose
 */
void saveStampedImgPointcloudTFPose ( const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud, 
                                      const cv::Mat& image,
                                      const tf::StampedTransform& tfTransform,
                                      const int& counter,
                                      const string& robot_pose_filename,
                                      const string& dataOutputDir )
{

    string numstr = getCounterStampInString(counter);

    string imageFilename = dataOutputDir + "/" + "color_image_" + numstr + ".jpg";
    string pcdFilename = dataOutputDir + "/" + "pointcloud_" + numstr + ".pcd";

    cv::imwrite( imageFilename, image);
    std::cerr << "Saved " << imageFilename << endl;
    pcl::io::savePCDFileASCII ( pcdFilename, *cloud );
    std::cerr << "Saved " << cloud->points.size() << " points to " << pcdFilename << endl;
    std::cerr << "**************************************" << endl;

    Mat Mat_tfpose_1_7(1,7,CV_64F);  // row:1 coloum:7
    tf::Quaternion quat = tfTransform.getRotation();
    tf::Vector3 transl = tfTransform.getOrigin();

    Mat_tfpose_1_7.at<double>(0,0) = transl.x();
    Mat_tfpose_1_7.at<double>(0,1) = transl.y();
    Mat_tfpose_1_7.at<double>(0,2) = transl.z();
    Mat_tfpose_1_7.at<double>(0,3) = quat.x();
    Mat_tfpose_1_7.at<double>(0,4) = quat.y();
    Mat_tfpose_1_7.at<double>(0,5) = quat.z();
    Mat_tfpose_1_7.at<double>(0,6) = quat.w();

    if (counter == 0) {  // first time
        FileStorage fswrite(robot_pose_filename, FileStorage::WRITE);
        fswrite << "robotposes" << Mat_tfpose_1_7;
        fswrite.release();
    } else {
        Mat temp;
        FileStorage fsread(robot_pose_filename, FileStorage::READ);
        fsread["robotposes"] >> temp;
        fsread.release();

        Mat robot_poses(counter + 1 , 7, CV_64F);
        for (int i = 0; i < counter; i++) {
            for (int j = 0; j < 7; j++) {
                robot_poses.at<double>(i,j) = temp.at<double>(i,j);
            }
        }

        for (int j = 0; j < 7; j++) {
            robot_poses.at<double>(counter, j) = Mat_tfpose_1_7.at<double>(0,j);
            FileStorage fswrite(robot_pose_filename, FileStorage::WRITE);
            fswrite << "robotposes" << robot_poses;
            fswrite.release();
        }

    }

}


//##############################################################################################
/**
 * @brief read camera intrinsic from xml
 * @param cameraMatrix Camera Intrinsic 3*3
 * @param distCoeffs Distortion Coefficients
 * @return true if read success
 */
bool initCameraIntrinsic(const std::string& CAMERA_DATA_FILE, Mat& cameraMatrix, Mat& distCoeffs)
{
    FileStorage fs(CAMERA_DATA_FILE, FileStorage::READ);
    fs["camera_matrix"]>>cameraMatrix;
    fs["distortion_coeffs"]>>distCoeffs;

    cout<<"Camera instrinsic matrix"<<endl;
    cout<<cameraMatrix<<endl;
    cout<<"camera distortion coefficients"<<endl;
    cout<<distCoeffs<<endl;

    fs.release();

    if(cameraMatrix.empty()) return false;
    if(distCoeffs.empty()) return false;

    return true;
}


Mat rotVecFromTransform(Mat transform)
{
    Mat rotation=transform(Range(0,3),Range(0,3));
    Mat rotVec(3,1,CV_64F);
    Rodrigues(rotation,rotVec);
    return rotVec;
}

Mat transVecFromTransform(Mat transform)
{
    Mat t;
    t=transform(Range(0,3),Range(3,4));
    return t ;
}

Mat calcBoardCornerPositions(Size boardSize, float squareSize)
{
    int width = boardSize.width;
    int height = boardSize.height;
    Mat boardCorners(width * height, 3, CV_32F);
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++){
            boardCorners.at<float>(i * width + j, 0) = float(j * squareSize);
            boardCorners.at<float>(i * width + j, 1) = float(i * squareSize);
            boardCorners.at<float>(i * width + j, 2) = float(0);
        }
    }
    return boardCorners;
}

bool calculate_extrinsic(Mat image,
                         Mat *transform,
                         Mat boardCorners,
                         Mat cameraMatrix,
                         Mat distCoeffs,
                         Settings s)
{
    Size imageSize;
    imageSize = image.size();
    vector<Point2f> cornerPointBuf;
    bool isFound;
    Mat viewTemp;
    cv::cvtColor(image, viewTemp, COLOR_BGR2GRAY);
    cout << "calibration boardsize: " << s.boardSize << endl;

    isFound = cv::findChessboardCorners(image,
                                        s.boardSize,
                                        cornerPointBuf,
                                        CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
    if (isFound)
    {
        cout << "******************** happily found corners ********************" << endl;

        // improve the found corners' coordinate accuracy for chessboard
        Mat viewGray;
        cv::cvtColor(image, viewGray, COLOR_BGR2GRAY);
        cv::cornerSubPix( viewGray, cornerPointBuf, Size(11,11),Size(-1,-1),
                          TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

        cv::drawChessboardCorners(image, s.boardSize, Mat(cornerPointBuf), isFound);

        Mat imageWithCorners = image.clone();
        imshow("image", imageWithCorners);
        waitKey(0);

        Mat imagePoint(cornerPointBuf.size(), 2, CV_32F);
        for (int i = 0; i < cornerPointBuf.size(); i++) {
            imagePoint.at<float>(i,0) = cornerPointBuf[i].x;
            imagePoint.at<float>(i,1) = cornerPointBuf[i].y;
        }

        CvMat* cv_rotarion_vector =  cvCreateMat(3,1,CV_32F);
        CvMat* cv_translation_vector = cvCreateMat(3,1,CV_32F);
        CvMat cv_objectpoint = boardCorners;
        CvMat cv_imagepoint = imagePoint;
        CvMat cv_cameraMatrix = cameraMatrix;
        CvMat cv_distCoeffs = distCoeffs;

        cvFindExtrinsicCameraParams2(&cv_objectpoint,
                                     &cv_imagepoint,
                                     &cv_cameraMatrix,
                                     &cv_distCoeffs,
                                     cv_rotarion_vector,
                                     cv_translation_vector);

        float *rotationPtr = (float*)cvPtr2D(cv_rotarion_vector,0,0);
        float r1,r2,r3;
        r1 = *rotationPtr;
        rotationPtr++;
        r2 = *rotationPtr;
        rotationPtr++;
        r3 = *rotationPtr;

        float *translationPtr = (float*)cvPtr2D(cv_translation_vector,0,0);
        float t1,t2,t3;
        t1 = *translationPtr;
        translationPtr++;
        t2 = *translationPtr;
        translationPtr++;
        t3 = *translationPtr;

        cv::Mat rodri(3,1,CV_64F);
        rodri.at<double>(0,0)=r1;
        rodri.at<double>(1,0)=r2;
        rodri.at<double>(2,0)=r3;

        Mat rotMat(3,3,CV_64F);
        Rodrigues(rodri,rotMat);

        transform->at<double>(0,3)=t1;
        transform->at<double>(1,3)=t2;
        transform->at<double>(2,3)=t3;
        transform->at<double>(3,3)=1;

        transform->at<double>(3,0)=0;
        transform->at<double>(3,1)=0;
        transform->at<double>(3,2)=0;

        for (int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                transform->at<double>(i,j)=rotMat.at<double>(i,j);
            }
        }
        return true;
    } else {
        cout << "Not found chessboard" << endl;
        return false;
    }
}


vector<string> getFilesInDirectory ( const string& cate_dir, const string suffix) {
    bool hasSuffix = ! suffix.empty();

    vector<string> files;
    vector<string> validfiles;
    DIR *dir;
    struct dirent *ptr;

    if ((dir=opendir(cate_dir.c_str())) == NULL)
    {
        perror("Open dir error...");
        exit(1);
    }

    while ((ptr=readdir(dir)) != NULL)
    {
        if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
            continue;
        else if(ptr->d_type == 8)    ///file
            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
            files.push_back(ptr->d_name);
        else if(ptr->d_type == 10)    ///link file
            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
            continue;
        else if(ptr->d_type == 4)    ///dir
        {
            files.push_back(ptr->d_name);
        }
    }
    closedir(dir);

    std::sort(files.begin(), files.end());
    for (size_t i = 0; i< files.size(); i++) {
        if( ! hasSuffix) {
            validfiles.push_back(cate_dir + "/" + files[i]);
        } else {
            string::size_type idx;
            idx=files[i].find(suffix);
            if(idx == string::npos) continue; // 当前文件名中不包含此后缀
            validfiles.push_back(cate_dir + "/" + files[i]);


        }

    }
    return validfiles;
}

}

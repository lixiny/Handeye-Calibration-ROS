#include "ArucoPlane.h"

using namespace std;
using namespace cv;

namespace handeye
{

ArucoPlane::ArucoPlane()
{
    /**
    shared_ptr<QR> qr(new QR());
    _qr = qr;
    */
}

void ArucoPlane::setQRnums(int numOfQRcodeOnSide)
{
    _numOfQRcodeOnSide = numOfQRcodeOnSide;


    if (_numOfQRcodeOnSide == 6)
    {
        SIX_x_SIX _sixxsix;
        _qr = _sixxsix;
    } else {
        cout << "Unimplemented QR side" << endl;
        exit(-1);
    }
}

Eigen::Matrix4d ArucoPlane::getTransform()
{
    return _transform;
}

float ArucoPlane::getCubeSide()
{
    //    _qr = ArucoPlane::chooseQRMode(_numOfQRcodeOnSide);
    return _qr.cubeSide;
}

float ArucoPlane::getCubeHight()
{
    return _qr.cubeHeight;
}

void ArucoPlane::setCameraIntrinsic(cv::Mat &cameraMatrix,
                                    cv::Mat &distCoeffs)
{
    _cameraMatrix = cameraMatrix;
    _distCoeffs = distCoeffs;
    _m_fx = cameraMatrix.ptr<float>(0)[0];
    _m_fy = cameraMatrix.ptr<float>(1)[1];
    _m_px = cameraMatrix.ptr<float>(0)[2];
    _m_py = cameraMatrix.ptr<float>(1)[2];
    _cameraIntrinsic << _m_fx, 0, _m_px, 0,
        0, _m_fy, _m_py, 0,
        0, 0, 1, 0;
}

bool ArucoPlane::calculateExtrinsicFromImage(cv::Mat &_colorImg)
{
    //    _qr = ArucoPlane::chooseQRMode(_numOfQRcodeOnSide);
    // QR code detection

    vector<int> markerIds;
    vector<vector<cv::Point2f>> markerCorners, markerRejected;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
    cv::aruco::detectMarkers(_colorImg, dictionary, markerCorners, markerIds, detectorParams, markerRejected);

    if (markerIds.size() == 0)
    {
        return false;
    }

    vector<Point3f> _objPts; // 3d points in world coordinates
    vector<Point2f> _imgPts; // 2d points in pixel coordinates

    for (int i = 0; i < markerIds.size(); i++)
    {

        float centerX = _qr.basePositions[markerIds[i]](0);
        float centerY = _qr.basePositions[markerIds[i]](1);
        float centerZ = 0;
        float half = _qr.half;

        Point3f upperLeftWorld = Point3f(centerX - half, centerY + half, centerZ);
        Point3f upperRightWorld = Point3f(centerX + half, centerY + half, centerZ);
        Point3f bottomRightWorld = Point3f(centerX + half, centerY - half, centerZ);
        Point3f bottomLeftWorld = Point3f(centerX - half, centerY - half, centerZ);

        _objPts.push_back(upperLeftWorld);
        _objPts.push_back(upperRightWorld);
        _objPts.push_back(bottomRightWorld);
        _objPts.push_back(bottomLeftWorld);

        float upperLeftPixelu = markerCorners[i][0].x;
        float upperLeftPixelv = markerCorners[i][0].y;
        float upperRightPixelu = markerCorners[i][1].x;
        float upperRightPixelv = markerCorners[i][1].y;
        float bottomRightPixelu = markerCorners[i][2].x;
        float bottomRightPixelv = markerCorners[i][2].y;
        float bottomLeftPixelu = markerCorners[i][3].x;
        float bottomLeftPixelv = markerCorners[i][3].y;

        _imgPts.push_back(cv::Point2f(upperLeftPixelu, upperLeftPixelv));
        _imgPts.push_back(cv::Point2f(upperRightPixelu, upperRightPixelv));
        _imgPts.push_back(cv::Point2f(bottomRightPixelu, bottomRightPixelv));
        _imgPts.push_back(cv::Point2f(bottomLeftPixelu, bottomLeftPixelv));
    }

    cv::solvePnP(_objPts, _imgPts, _cameraMatrix, _distCoeffs, _rvec, _tvec);
    // cout << _rvec << " " << _tvec << endl;
    ArucoPlane::buildTransformMatrix(_rvec, _tvec, _rotation, _translation, _transform);

    return true;
}

void ArucoPlane::drawingAxis(cv::Mat &_tempImg)
{
    cv::aruco::drawAxis(_tempImg, _cameraMatrix, _distCoeffs, _rvec, _tvec, 0.2);
}

void ArucoPlane::drawingCube(cv::Mat &_tempImg)
{
    //    _qr = ArucoPlane::chooseQRMode(_numOfQRcodeOnSide);

    cv::aruco::drawAxis(_tempImg, _cameraMatrix, _distCoeffs, _rvec, _tvec, 0.2);
    Eigen::Matrix<float, 3, 1> KTP[8];
    Point2f cubeVertexInImage[8];
    for (int k = 0; k < 8; k++)
    {
        KTP[k] = _cameraIntrinsic * _transform.cast<float>() * _qr.cubeVertexInWorld[k];
        cubeVertexInImage[k].x = KTP[k](0) / KTP[k](2); // u
        cubeVertexInImage[k].y = KTP[k](1) / KTP[k](2); // v
    }

    cv::line(_tempImg, cubeVertexInImage[0], cubeVertexInImage[1], Scalar(0, 255, 0), 3);
    cv::line(_tempImg, cubeVertexInImage[1], cubeVertexInImage[2], Scalar(0, 255, 0), 3);
    cv::line(_tempImg, cubeVertexInImage[2], cubeVertexInImage[3], Scalar(0, 255, 0), 3);
    cv::line(_tempImg, cubeVertexInImage[3], cubeVertexInImage[0], Scalar(0, 255, 0), 3);

    cv::line(_tempImg, cubeVertexInImage[4], cubeVertexInImage[5], Scalar(0, 0, 255), 3);
    cv::line(_tempImg, cubeVertexInImage[5], cubeVertexInImage[6], Scalar(0, 0, 255), 3);
    cv::line(_tempImg, cubeVertexInImage[6], cubeVertexInImage[7], Scalar(0, 0, 255), 3);
    cv::line(_tempImg, cubeVertexInImage[7], cubeVertexInImage[4], Scalar(0, 0, 255), 3);

    cv::line(_tempImg, cubeVertexInImage[0], cubeVertexInImage[4], Scalar(255, 255, 0), 3);
    cv::line(_tempImg, cubeVertexInImage[1], cubeVertexInImage[5], Scalar(255, 255, 0), 3);
    cv::line(_tempImg, cubeVertexInImage[2], cubeVertexInImage[6], Scalar(255, 255, 0), 3);
    cv::line(_tempImg, cubeVertexInImage[3], cubeVertexInImage[7], Scalar(255, 255, 0), 3);
}

/** private: */

void ArucoPlane::buildTransformMatrix(const cv::Mat &rvec,
                                      const cv::Mat &tvec,
                                      Eigen::Matrix3d &rotation,
                                      Eigen::Vector3d &translation,
                                      Eigen::Matrix4d &transform)
{

    //    cv::Matx33d r;
    cv::Mat rVec(3, 1, CV_64F);
    rVec.at<double>(0, 0) = rvec.at<double>(0, 0);
    rVec.at<double>(0, 1) = rvec.at<double>(0, 1);
    rVec.at<double>(0, 2) = rvec.at<double>(0, 2);
    cv::Mat r(3, 3, CV_64F);
    cv::Rodrigues(rVec, r);

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            rotation(i, j) = r.at<double>(i, j);
        }
    }

    translation(0, 0) = (double)tvec.at<double>(0, 0);
    translation(1, 0) = (double)tvec.at<double>(0, 1);
    translation(2, 0) = (double)tvec.at<double>(0, 2);

    Sophus::SE3<double> SE3Rt(rotation, translation);

    transform = SE3Rt.matrix();
}

void ArucoPlane::notification()
{

    cout << "--- Successfully set QR plate arguments! ---" << endl;
    cout << "QR TAGs on each side is: " << _numOfQRcodeOnSide << endl;
    cout << "Extraction cuboid is:    " << _qr.cubeSide << " * " << _qr.cubeSide << " * " << _qr.cubeHeight << endl;
    if (_numOfQRcodeOnSide == 10)
    {
        cout << " ▆ ▆ ▆ ▆ ▆ ▆ ▆ ▆ ▆ ▆ \n";
        cout << " ▆ ▆ ▆ ▆ ▆ ▆ ▆ ▆ ▆ ▆ \n";
        cout << " ▆ ▆             ▆ ▆ \n";
        cout << " ▆ ▆             ▆ ▆ \n";
        cout << " ▆ ▆             ▆ ▆ \n";
        cout << " ▆ ▆             ▆ ▆ \n";
        cout << " ▆ ▆             ▆ ▆ \n";
        cout << " ▆ ▆             ▆ ▆ \n";
        cout << " ▆ ▆ ▆ ▆ ▆ ▆ ▆ ▆ ▆ ▆ \n";
        cout << " ▆ ▆ ▆ ▆ ▆ ▆ ▆ ▆ ▆ ▆ \n";
    }
    else if (_numOfQRcodeOnSide == 6)
    {
        cout << " ▆ ▆ ▆ ▆ ▆ ▆ \n";
        cout << " ▆         ▆ \n";
        cout << " ▆         ▆ \n";
        cout << " ▆         ▆ \n";
        cout << " ▆         ▆ \n";
        cout << " ▆ ▆ ▆ ▆ ▆ ▆ \n";
    }
}

} // namespace handeye

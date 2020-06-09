#ifndef ARUCOPLANE_H
#define ARUCOPLANE_H

#include "CommonInclude.h"

using namespace std;
using namespace cv;

namespace handeye
{

class ArucoPlane
{
public:
    
    ArucoPlane ();
    ArucoPlane (int numOfQRcodeOnSide) :
        _numOfQRcodeOnSide ( numOfQRcodeOnSide )
    {

        if (_numOfQRcodeOnSide == 6) {
            SIX_x_SIX _sixxsix;
            _qr = _sixxsix;
        }

        notification();

    }

    /** Member Variables */

    class QR
    {
    public:
        QR(){}
        float QR_TAG_SIZE; //meter
        float half;
        float length1;
        float length2;
        float length3;
        float length4;
        float length5;
        float cubeSide;
        float cubeHeight;
        vector<Eigen::Vector3f> basePositions;
        vector<Eigen::Vector4f> cubeVertexInWorld;

    } _qr;

//    shared_ptr<QR> _qr;

    double _m_fx;
    double _m_fy;
    double _m_px;
    double _m_py;

    cv::Mat _cameraMatrix;
    cv::Mat _distCoeffs;
    cv::Mat _rvec, _tvec;

    Eigen::Matrix<float,3,4> _cameraIntrinsic;
    Eigen::Matrix3d _rotation;
    Eigen::Vector3d _translation;
    Eigen::Matrix4d _transform;
    int _numOfQRcodeOnSide;

    /** Member Fuctions */

    void setQRnums(int numOfQRcodeOnSide);

    void setCameraIntrinsic (cv::Mat& cameraMatrix, cv::Mat& distCoeffs);

    bool calculateExtrinsicFromImage(cv::Mat& _colorImg);

    void drawingCube(cv::Mat& _tempImg);
    void drawingAxis(cv::Mat& _tempImg);

    Eigen::Matrix4d getTransform();

    float getCubeSide();
    float getCubeHight();



private:

    void buildTransformMatrix (const cv::Mat& rvec,
                               const cv::Mat& tvec,
                               Eigen::Matrix3d& rotation,
                               Eigen::Vector3d& translation,
                               Eigen::Matrix4d& transform);

    void notification();

    class SIX_x_SIX : public QR
    {
    public:
        SIX_x_SIX()
        {
            QR_TAG_SIZE = __QR_TAG_SIZE;
            half        = __half;
            length1     = __length1;
            length2     = __length2;
            length3     = __length3;
            length4     = 0;
            length5     = 0;
            cubeSide    = __cubeSide;
            cubeHeight  = __cubeHeight;

            for(int i = 0; i < 20; i++) {
                basePositions.push_back(__basePositions[i]);
            }

            for(int j = 0; j < 8; j++) {
                cubeVertexInWorld.push_back(__cubeVertexInWorld[j]);
            }
        }

    private:
        float __QR_TAG_SIZE = 0.035;  //meter
        float __half = 0.035/2.0;
        float __length1 = 0.1075;
        float __length2 = 0.0645;
        float __length3 = 0.0215;
        float __cubeSide = 0.26;
        float __cubeHeight = 0.30;


        Eigen::Vector3f __basePositions[20] = {{-__length1, __length1, 0}, //0
                                               {-__length2, __length1, 0},//1
                                               {-__length3, __length1, 0},//2
                                               {__length3, __length1, 0},//3
                                               {__length2, __length1, 0},//4
                                               {__length1, __length1, 0},//5
                                               {-__length1, __length2, 0},//6
                                               {__length1, __length2, 0},//7
                                               {-__length1, __length3, 0},//8
                                               {__length1, __length3, 0},//9
                                               {-__length1, -__length3, 0},//10
                                               {__length1, -__length3, 0},//11
                                               {-__length1, -__length2, 0},//12
                                               {__length1, -__length2, 0},//13
                                               {-__length1, -__length1, 0},//14
                                               {-__length2, -__length1, 0},//15
                                               {-__length3, -__length1, 0},//16
                                               {__length3, -__length1, 0},//17
                                               {__length2, -__length1, 0},//18
                                               {__length1, -__length1, 0}//19
                                              };

        Eigen::Vector4f __cubeVertexInWorld[8] = {{-__cubeSide/2, __cubeSide/2, 0, 1},
                                                  {__cubeSide/2, __cubeSide/2, 0, 1},
                                                  {__cubeSide/2, -__cubeSide/2, 0, 1},
                                                  {-__cubeSide/2, -__cubeSide/2, 0, 1},
                                                  {-__cubeSide/2, __cubeSide/2, __cubeHeight, 1},
                                                  {__cubeSide/2, __cubeSide/2, __cubeHeight, 1},
                                                  {__cubeSide/2, -__cubeSide/2, __cubeHeight, 1},
                                                  {-__cubeSide/2, -__cubeSide/2, __cubeHeight, 1}
                                                 };
    };
};

typedef shared_ptr<ArucoPlane> ArucoPlanePtr;

}


#endif // CUBEECTRACTOR_H

#include "edge_calib.h"

EdgeCalib::EdgeCalib() : BaseUnaryEdge<6, SE3Type, VertexSE3>(){}

void EdgeCalib::computeError()
{
    // error = inverse(wTo_k) * oTc * c_k-1_T_c_k * inverse(oTc), oTc refers to a transform from camera frame to odom fram.
    const VertexSE3* extrinsic = static_cast<const VertexSE3*>(_vertices[0]);

    SE3Type stereo_measure = measurement();
    SE3Type lidar_measure = LidarMotion;

    Eigen::Matrix<double, 6, 1> vSe3 = internal::toVectorMQT(extrinsic->estimate());
    Point so3 = Point(vSe3.segment(0,3));
    Point se3 = Point(vSe3.segment(3,3));
    SE3Type extrinSE3 = SE3Type(SO3Type::exp(so3), se3);
    SE3Type lidarMotionInCameraFrame = extrinSE3 * lidar_measure * extrinSE3.inverse();
    SE3Type delta = stereo_measure.inverse() * lidarMotionInCameraFrame;
    _error = delta.log();
}

bool EdgeCalib::read(std::istream &is) {return false;}

bool EdgeCalib::write(std::ostream& os) const {return false;}

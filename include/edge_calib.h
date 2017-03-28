#ifndef EDGE_CALIB_H
#define EDGE_CALIB_H

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/core/base_unary_edge.h"

#include "se3.hpp"

/**
 * \brief calibrate odometry and stereo based on a set of measurements
 */

using namespace g2o;
using SE3Type = Sophus::SE3<double>;
using SO3Type = Sophus::SO3<double>;
using Point = SE3Type::Point;

// used to be the calibration measurement.
// struct LidarAndStereoMotion
// {
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
//   SE3Type LidarMotion;
//   SE3Type StereoMotion;
// };

// An edge used to calibrate the offset between odometry and stereo
class EdgeCalib : public BaseUnaryEdge<6, SE3Type, VertexSE3>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeCalib();

    void computeError();

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
    
    SE3Type LidarMotion;
};

#endif

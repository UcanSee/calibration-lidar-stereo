#include "g2o/core/base_unary_edge.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "se3.hpp"

using namespace g2o;
using SE3Type = Sophus::SE3<double>;
using SO3Type = Sophus::SO3<double>;
using Point = SE3Type::Point;

typedef Eigen::Matrix<double, 6, 1> Vector6D;
typedef Eigen::Matrix<double, 6, 6> Matrix6D;

class EdgePrior : public BaseUnaryEdge<6, SE3Type, VertexSE3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgePrior();

    bool read(std::istream &is){return true;}

    bool write(std::ostream &os) const{return true;}

    void computeError();

};
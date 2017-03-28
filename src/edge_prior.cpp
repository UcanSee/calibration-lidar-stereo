#include "edge_prior.h"

EdgePrior::EdgePrior() : BaseUnaryEdge<6, SE3Type, VertexSE3>(){}

void EdgePrior::computeError()
{
   const VertexSE3* vertexcur = static_cast<const VertexSE3*>(_vertices[0]);
   Vector6D vse3 = internal::toVectorMQT(vertexcur->estimate());
   SE3Type extrinsic_cur = SE3Type(SO3Type::exp(Point(vse3.segment(0,3))), Point(vse3.segment(3,3)));
   SE3Type extrinsic_last = measurement();
   SE3Type delta = extrinsic_last.inverse() * extrinsic_cur;
   _error = delta.log();
}
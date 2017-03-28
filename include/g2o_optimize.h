#ifndef G2O_OPTIMIZE_H
#define G2O_OPTIMIZE_H

#include <vector>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3_prior.h"

#include "g2o/core/eigen_types.h"

#include "edge_calib.h"
#include "edge_prior.h"

using namespace std;
using namespace g2o;

typedef Eigen::Matrix<double, 6, 1> Vector6D;
typedef Eigen::Matrix<double, 6, 6> Matrix6D;

class Optimizer
{
public:

    void static initOptimizer(SparseOptimizer &, bool _verbose = false);
    void static addVertexSE3(SparseOptimizer &, const SE3Type &, const int , bool _fixed = false);
    void static addEdgeCalib(SparseOptimizer &, int , const SE3Type &, const SE3Type &, const Matrix6D &);
    void static addEdgePrior(SparseOptimizer &, int , const SE3Type &, const Matrix6D &);

    // build structure and optimize
    void static optimize(SparseOptimizer &, int );

    SE3Type static getEstimation(SparseOptimizer &);
    Matrix6D static getMargInfo(SparseOptimizer &);

    void static removeEdges(SparseOptimizer &);
private:
    static vector<g2o::OptimizableGraph::Edge*> vEdges;
};


#endif

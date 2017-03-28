#include "g2o_optimize.h"

vector<g2o::OptimizableGraph::Edge*> Optimizer::vEdges = vector<g2o::OptimizableGraph::Edge*>();

void Optimizer::initOptimizer(SparseOptimizer & _opt, bool _verbose)
{
    typedef BlockSolver< BlockSolverTraits<-1, -1> >  SclamBlockSolver;
    typedef LinearSolverCholmod<SclamBlockSolver::PoseMatrixType> SclamLinearSolver;

    SclamLinearSolver* linearSolver = new SclamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SclamBlockSolver* blockSolver = new SclamBlockSolver(linearSolver);
    OptimizationAlgorithm* solver = 0;
    solver = new OptimizationAlgorithmLevenberg(blockSolver);

    _opt.setAlgorithm(solver);
    _opt.setVerbose(_verbose);
    
    vEdges.clear();
}

void Optimizer::addVertexSE3(SparseOptimizer & _opt, const SE3Type & _initv, const int _id, bool _fixed)
{
    VertexSE3* stereoOffset = new VertexSE3;
    stereoOffset->setId(_id);
    Eigen::Matrix3d rotationMat = _initv.rotationMatrix();
    Eigen::Vector3d translation = _initv.translation();
    Isometry3D t;
    t = rotationMat;
    t.translation() = translation;
    stereoOffset->setEstimate(t);
    stereoOffset->setFixed(_fixed);
    _opt.addVertex(stereoOffset);
}

void Optimizer::addEdgeCalib(SparseOptimizer & _opt, int _id, const SE3Type & _measure, const SE3Type & _lidar, const Matrix6D & _infomat)
{
    EdgeCalib* calibEdge = new EdgeCalib;
    calibEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(_opt.vertex(_id)));
    calibEdge->setInformation(_infomat);
    calibEdge->setMeasurement(_measure);
    calibEdge->LidarMotion = _lidar;
    _opt.addEdge(calibEdge);
    vEdges.push_back(calibEdge);
}

void Optimizer::addEdgePrior(SparseOptimizer & _opt, int _id, const SE3Type & _initv, const Matrix6D & _infomat)
{
    EdgePrior* priorEdge = new EdgePrior;
    priorEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(_opt.vertex(_id)));
    priorEdge->setInformation(_infomat);
    priorEdge->setMeasurement(_initv);
    _opt.addEdge(priorEdge);
    vEdges.push_back(priorEdge);
}

// build structure and optimize
void Optimizer::optimize(SparseOptimizer & _opt, int _numiter)
{
    _opt.initializeOptimization(0);
    _opt.optimize(_numiter);
}

SE3Type Optimizer::getEstimation(SparseOptimizer & _opt)
{
    const VertexSE3* v = static_cast<const VertexSE3*>(_opt.vertex(0));
    Isometry3D se3 = v->estimate();
    Vector6D ksi = internal::toVectorMQT(se3);
    Point so3 = Point(ksi.segment(0,3));
    Point se3_ = Point(ksi.segment(3,3));
    SE3Type est = SE3Type(SO3Type::exp(so3), se3_);
    return est;
}

Matrix6D Optimizer::getMargInfo(SparseOptimizer & _opt)
{
    std::vector<g2o::OptimizableGraph::Vertex*> margVerteces;
    margVerteces.push_back(_opt.vertex(0));
    
    g2o::SparseBlockMatrixXd spinv;
    _opt.computeMarginals(spinv, margVerteces);

    Matrix6D margCov = Matrix6D::Zero();
    margCov = spinv.block(0,0)->eval();
    
    return margCov.inverse();
}

void Optimizer::removeEdges(SparseOptimizer & _opt)
{
    for(auto it:vEdges)
    {
	_opt.removeEdge(it);
    }
    vEdges.clear();
}
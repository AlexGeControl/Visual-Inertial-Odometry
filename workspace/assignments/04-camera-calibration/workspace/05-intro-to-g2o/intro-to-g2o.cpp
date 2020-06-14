#include <iostream>
#include <cmath>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

using namespace std;

// vertex for optimization params:
class CurveFittingVertex: public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // operation reset:
    virtual void setToOriginImpl() {
        _estimate << 0.0, 0.0, 0.0;
    }
    // operation update:
    virtual void oplusImpl(const double *update) {
        _estimate += Eigen::Vector3d(update);
    }
    // operation read:
    virtual bool read(istream &in) {}
    // operation write:
    virtual bool write(ostream &out) const {}
};

// egde for error terms:
class CurveFittingEdge: public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // constructor:
    CurveFittingEdge(double x): BaseUnaryEdge(), _x(x) {}
    // operation compute error:
    void computeError() {
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex *>(_vertices[0]);
        const Eigen::Vector3d estimate = v->estimate();
        _error(0, 0) = _measurement - std::exp(estimate(0, 0)*_x*_x + estimate(1, 0)*_x + estimate(2, 0));
    }
    // operation read:
    virtual bool read(istream &in) {}
    // operation write:
    virtual bool write(ostream &out) const {}
private:
    double _x;
};

int main(int argc, char **argv) {
    // ground truth params:
    double ground_truth[3] = {1.0, 2.0, 1.0};

    // num points: 
    int N = 100;
    // noise standard deviation:                                 
    double w_sigma = 1.0;                        
    // OpenCV random number generator:
    cv::RNG rng;                                 

    // observations:
    vector<double> x_data, y_data;      
    for (int i = 0; i < N; ++i) {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ground_truth[0]*x*x + ground_truth[1]*x + ground_truth[2]) + rng.gaussian(w_sigma));
    }

    // solver:
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> Block;
    Block::LinearSolverType *linear_solver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block *block_solver = new Block(linear_solver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(block_solver);
    // optimizer:
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // init params:
    CurveFittingVertex *v = new CurveFittingVertex();
    v->setEstimate(Eigen::Vector3d::Zero());
    v->setId(0);
    optimizer.addVertex(v);

    // init losses:
    for (int i = 0; i < N; ++i) {
        CurveFittingEdge *e = new CurveFittingEdge(x_data[i]);
        e->setId(i);
        e->setVertex(0, v);
        e->setMeasurement(y_data[i]);
        e->setInformation((1.0/(w_sigma*w_sigma)*Eigen::Matrix<double, 1, 1>::Identity()));
        optimizer.addEdge(e);
    }

    // optimize:
    const int MAX_ITER = 100;
    chrono::steady_clock::time_point start_time = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(MAX_ITER);
    chrono::steady_clock::time_point end_time = chrono::steady_clock::now();

    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(end_time - start_time);
    cout << "Ceres solver costs " << time_used.count() << " seconds." << endl;

    // display:
    cout << "Ground Truth: " << endl;
    for (auto g: ground_truth) {
        cout << g << ", ";
    }
    cout << "Estimation: " << endl;
    Eigen::Vector3d estimation = v->estimate();
    cout << estimation.transpose() << endl;
    return 0;
}
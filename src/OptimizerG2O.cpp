#include <iostream>
#include <fstream>
#include <string>

#include <g2o/types/slam2d/types_slam2d.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
// #include <g2o/solvers/cholmod/linear_solver_cholmod.h>
// #include <g2o/solvers/csparse/linear_solver_csparse.h>
// #include <g2o/solvers/dense/linear_solver_dense.h>


using namespace std;

int main(int argc, char **argv) {
    if (argc != 2) {
        cout << "Usage: pose_graph_g2o_SE3 sphere.g2o" << endl;
        return 1;
    }
    ifstream fin(argv[1]);
    if (!fin) {
        cout << "file " << argv[1] << " does not exist." << endl;
        return 1;
    }

    // 2D block solver
    // Choose the block solver of 3 × 3, using the Levenberg-Marquardt descent method,
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 3>> BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
    //  typedef g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType> LinearSolverType;
     
    /*
    The ringcity dataset works better with Gauss Newton rather than levenberg Marquardt
    */
    // auto solver = new g2o::OptimizationAlgorithmLevenberg(
    //     std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));

    auto solver = new g2o::OptimizationAlgorithmGaussNewton(
        std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;     // Optimizer
    optimizer.setAlgorithm(solver);   
    optimizer.setVerbose(true);       


    int vertexCnt = 0, edgeCnt = 0; // Number of vertices and edges
    while (!fin.eof()) {
        string name;
        fin >> name;
        if (name == "VERTEX_SE2") {
            // SE2 vertex
            g2o::VertexSE2* v = new g2o::VertexSE2();
            int index = 0;
            double x, y, theta;
            fin >> index >> x >> y >> theta;

            v->setId(index);
            v->setEstimate(g2o::SE2(x, y, theta));
            optimizer.addVertex(v);
            vertexCnt++;
            
            if (index == 0)
                v->setFixed(true);
        } else if (name == "EDGE_SE2") {
            // SE2-SE2 edge
            g2o::EdgeSE2* e = new g2o::EdgeSE2();

            int idx1, idx2;
            double dx, dy, dtheta;
            double info01, info02, info12;
            double info00, info11, info22;

            // Initialize to zero
            Eigen::Matrix3d info = Eigen::Matrix3d::Zero();

            fin >> idx1 >> idx2 >> dx >> dy >> dtheta
                >> info00 >> info01>> info02
                >> info11 >> info12 >> info22;

            info(0,0) = info00;
            info(0,1) = info01;
            info(0,2) = info02;
            info(1,1) = info11;
            info(1,2) = info12;
            info(2,2) = info22;

            e->setId(edgeCnt++);
            e->setVertex(0, optimizer.vertices()[idx1]);
            e->setVertex(1, optimizer.vertices()[idx2]);
            optimizer.addEdge(e);
            e->setInformation(info);
            e->setMeasurement(g2o::SE2(dx, dy, dtheta));
        }
        if (!fin.good()) {
            if (fin.eof()) {
                std::cout << "Reached end of file.\n";
            } else if (fin.fail()) {
                std::cerr << "Input format error!\n";
            } else if (fin.bad()) {
                std::cerr << "Serious IO error!\n";
            }
            break;
        }

    }

    cout << "read total " << vertexCnt << " vertices, " << edgeCnt << " edges." << endl;

    cout << "optimizing ..." << endl;
    optimizer.initializeOptimization();
    optimizer.optimize(200);

    cout << "saving optimization results ..." << endl;
    optimizer.save("optimized_2d.g2o");

    return 0;
}
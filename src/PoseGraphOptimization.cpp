#include "PoseGraphOptimization.h"
#include <glog/logging.h>


PoseGraphCeres::PoseGraphCeres(int max_iterations) : max_iterations_(max_iterations) {}
PoseGraphCeres::PoseGraphCeres() : max_iterations_(50) {}

PoseGraphCeres::~PoseGraphCeres() {}

void PoseGraphCeres::runOptimization(map<int, std::shared_ptr<Poses>>& poses, std::vector<shared_ptr<Constraints>>& constraints){

    ceres::Problem problem;
    ceres::Manifold* quaternion_manifold = new ceres::EigenQuaternionManifold;

    for (const auto& c : constraints) {
        // Use an iterator to find the associated poses that connect into an edge
        auto firstPose = poses.find(c->firstPoseIndex);
        auto secondPose = poses.find(c->secondPoseIndex);

        PoseGraphOptimization* pg_optim = new PoseGraphOptimization(c->edge, c->information);
        ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<PoseGraphOptimization, 6, 3, 4, 3, 4>(pg_optim);
        problem.AddResidualBlock(cost_function, nullptr /* squared loss */,
            firstPose->second->pose.position.data(),
            firstPose->second->pose.quaternion.coeffs().data(),
            secondPose->second->pose.position.data(),
            secondPose->second->pose.quaternion.coeffs().data());

        problem.SetManifold(firstPose->second->pose.quaternion.coeffs().data(), quaternion_manifold);
        problem.SetManifold(secondPose->second->pose.quaternion.coeffs().data(), quaternion_manifold);
    }

      
    // Set the first pose as a constant. More detail in notes.md as to why.
    auto poseStart = poses.begin();
    problem.SetParameterBlockConstant(poseStart->second->pose.position.data());
    problem.SetParameterBlockConstant(poseStart->second->pose.quaternion.coeffs().data());
    
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = max_iterations_;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    LOG(INFO) << "Ceres optimizer summary:";
    LOG(INFO) << summary.BriefReport();   
}
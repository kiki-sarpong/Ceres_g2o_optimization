#ifndef _POSE_GRAPH_OPTIMIZATION_H
#define _POSE_GRAPH_OPTIMIZATION_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>
#include "Constraints.h"
#include "Poses.h"

using namespace Eigen;
using namespace std;

using Matrix6d = Matrix<double, 6, 6>;

/*
https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/slam/pose_graph_3d/pose_graph_3d_error_term.h
*/


struct PoseGraphOptimization {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Proper memory alignment allocation

        PoseGraphOptimization(const Pose3d& edge_constraint, const Matrix6d& information)
            : edge_constraint_(edge_constraint) {
                // This is to compute square root information. See notes for further explanation
                Eigen::LLT<Matrix<double, 6, 6>> llt(information);
                sqrt_information = llt.matrixL().transpose();
            }

        template <typename T>
        bool operator()(const T* const p_a_ptr, const T* const q_a_ptr, const T* const p_b_ptr,
            const T* const q_b_ptr, T* residuals_ptr) const {

            Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_ptr);
            Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);
            Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

            // Compute the relative transformation between the two frames.
            Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
            Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

            // Represent the displacement between the two frames in the A frame.
            Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

            // Compute the error between the two orientation estimates.
            Eigen::Quaternion<T> delta_q =
                edge_constraint_.quaternion.template cast<T>() * q_ab_estimated.conjugate();

            // Compute the residuals.
            // [ position         ]   [ delta_p          ]
            // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
            Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
            residuals.template block<3, 1>(0, 0) =
                p_ab_estimated - edge_constraint_.position.template cast<T>();
            residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();

            // Scale the residuals by the measurement uncertainty.
            residuals.applyOnTheLeft(sqrt_information.template cast<T>());
            return true;
        }

    private:
            Pose3d edge_constraint_;
            Matrix6d sqrt_information;
};


class PoseGraphCeres{
    public:
        PoseGraphCeres(int max_iterations);
        PoseGraphCeres();
        ~PoseGraphCeres();

        void runOptimization(map<int, shared_ptr<Poses>>& poses, vector<shared_ptr<Constraints>>& constraints);
        
    private:
        int max_iterations_;
};

#endif  /*_POSE_GRAPH_OPTIMIZATION_H*/
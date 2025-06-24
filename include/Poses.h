#ifndef _POSES_H
#define _POSES_H

#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

struct Pose3d{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Proper memory alignment allocation
        Eigen::Vector3d position;
        Eigen::Quaterniond quaternion;

        // Default constructor
        Pose3d() : position(Eigen::Vector3d::Zero()), quaternion(Eigen::Quaterniond::Identity()) {}
};

class Poses {
    public:
        Pose3d pose;
        int id;

        Poses();
        ~Poses();

        void read(std::istream& is);
        void write(std::ostream& os) const;
};

#endif /*_POSES_H*/
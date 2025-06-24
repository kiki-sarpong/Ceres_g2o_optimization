#ifndef _CONSTRAINTS_H
#define _CONSTRAINTS_H

#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Poses.h>



class Constraints {
    public:
        Pose3d edge;  // This is the edge constraint b/n poses
        int id; // Edge id
        Eigen::Matrix<double, 6, 6> information;  // Information matrix

        // Associated vertices/poses indices for the edge
        int firstPoseIndex;
        int secondPoseIndex;

        Constraints(int& index1, int& index2);
        ~Constraints();

        void read(std::istream& is);
        void write(std::ostream& os) const;
};

#endif /*_CONSTRAINTS_H*/
#include "Constraints.h"
#include "Poses.h"


/* Constructor */
Constraints::Constraints(int& index1, int& index2) : firstPoseIndex(index1), secondPoseIndex(index2),
         information(Eigen::Matrix<double, 6, 6>::Identity()) {}
/* Destructor */
Constraints::~Constraints(){}

/* Read Constraints/edge */
void Constraints::read(std::istream& is){
    double data[7];

    // Read stream to data array
    for (int i=0; i<7; ++i){
        is >> data[i];
    }
    // Quaternion is qw, qx, qy, qz 
    edge.quaternion = Eigen::Quaterniond(data[6], data[3], data[4], data[5]);
    // Normalize the quaternion to account for precision loss due toserialization.
    edge.quaternion.normalize();
    edge.position = Eigen::Vector3d(data[0], data[1], data[2]);

    for (int i = 0; i < information.rows() && is.good(); i++)
            for (int j = i; j < information.cols() && is.good(); j++) {
                is >> information(i, j);
                if (i != j)
                    information(j, i) = information(i, j);
            }
}

/* Write Constraints/edge */
void Constraints::write(std::ostream& os) const {
    os << firstPoseIndex << " " << secondPoseIndex << " ";

    // Write position
    os << edge.position.x() << " " << edge.position.y() << " " << edge.position.z() << " ";
    
    // Write quaternion (x, y, z, w format for output)
    os << edge.quaternion.x() << " " << edge.quaternion.y() << " " 
       << edge.quaternion.z() << " " << edge.quaternion.w() << std::endl;

    // information matrix 
    for (int i = 0; i < information.rows(); i++)
        for (int j = i; j < information.cols(); j++) {
            os << information(i, j) << " ";
        }
    os << std::endl;

}
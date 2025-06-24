#include "Poses.h"


/* Constructor */
Poses::Poses(){}
/* Destructor */
Poses::~Poses(){}

/* Read poses/vertices */
void Poses::read(std::istream& is){
    double data[7];

    // Read stream to data array
    for (int i=0; i<7; ++i){
        is >> data[i];
    }
    // Quaternion is qw, qx, qy, qz  // vector x, y, z
    pose.position = Eigen::Vector3d(data[0], data[1], data[2]);
    pose.quaternion = Eigen::Quaterniond(data[6], data[3], data[4], data[5]);
    // Normalize the quaternion to account for precision loss due toserialization.
    pose.quaternion.normalize();
}

/* Write poses/vertices */
void Poses::write(std::ostream& os) const {
    os << id << " ";
    os << pose.position.x() << " " << pose.position.y() << " " << pose.position.z() << " ";
    // Write quaternion (x, y, z, w format for output)
    os << pose.quaternion.x() << " " << pose.quaternion.y() << " " 
       << pose.quaternion.z() << " " << pose.quaternion.w() << std::endl;
}
#ifndef ICPCLOSEDFORM_H
#define ICPCLOSEDFORM_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

namespace ICP_Closedform {
    Eigen::Isometry3d pointToPlane(std::vector<Eigen::Vector3d> &src,std::vector<Eigen::Vector3d> &dst,std::vector<Eigen::Vector3d> &nor);
    Eigen::Isometry3d pointToPoint(std::vector<Eigen::Vector3d> &src,std::vector<Eigen::Vector3d> &dst);
}

#endif // ICPCLOSEDFORM_H


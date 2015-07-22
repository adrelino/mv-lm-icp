#ifndef ICPG2O
#define ICPG2O

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <frame.h>

namespace ICP_G2O {
    //pairwise
    Eigen::Isometry3d pointToPlane(std::vector<Eigen::Vector3d> &src,std::vector<Eigen::Vector3d> &dst,std::vector<Eigen::Vector3d> &nor);
    Eigen::Isometry3d pointToPoint(std::vector<Eigen::Vector3d> &src,std::vector<Eigen::Vector3d> &dst);

    //multiview
    void g2oOptimizer(std::vector< std::shared_ptr<Frame> >& frames, bool pointToPlane);
}

#endif // ICPG2O


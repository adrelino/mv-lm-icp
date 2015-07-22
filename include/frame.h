#ifndef FRAME_H
#define FRAME_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include "nanoflann.hpp"
#include <gflags/gflags.h>

#define GLFW_INCLUDE_GLU
//#include <OpenGL/glu.h>
#include <GLFW/glfw3.h>

#include "common.h"

//DEFINE_bool(pointToPlane, false, "use point to plane distance metric");


using namespace Eigen;
using namespace std;

struct Correspondance{
    int first;
    int second;
    double dist;
};

struct OutgoingEdge{
    int neighbourIdx;
    float weight; //==error of correspondances
    vector<Correspondance> correspondances;  //srcIdx, dstIdx, dist
    Isometry3f P_relative;
};

class Frame
{
public:
    Frame();
//    Frame(const std::string filename);
    ~Frame();

    vector<Vector3d> pts;
    vector<Vector3d> nor;

    bool fixed;

    Isometry3d pose = Isometry3d::Identity();
    Isometry3d poseGroundTruth = Isometry3d::Identity();

    vector<OutgoingEdge> neighbours;

    vector<Vector3d> getNeighbours(int queryIdx, size_t num_results);
    void recomputeNormals();


    void draw();

    void computePoseNeighboursKnn(vector< shared_ptr<Frame> >* frames, int i, int k);
    void computeClosestPointsToNeighbours(vector< shared_ptr<Frame> >* frames, float thresh);
    double getClosestPoint(const Vector3d& query_pt, size_t& ret_index); //query_pt must be in dstFrame

    typedef nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, Frame > ,
        Frame,
        3 /* dim */
        > my_kd_tree_t;

    my_kd_tree_t* indexPtr;
    bool indexComputed = false;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
    inline double kdtree_distance(const double *p1, const size_t idx_p2,size_t /*size*/) const
    {
        const double d0=p1[0]-pts[idx_p2].x();
        const double d1=p1[1]-pts[idx_p2].y();
        const double d2=p1[2]-pts[idx_p2].z();
        return d0*d0+d1*d1+d2*d2;
    }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline double kdtree_get_pt(const size_t idx, int dim) const
    {
        if (dim==0) return pts[idx].x();
        else if (dim==1) return pts[idx].y();
        else return pts[idx].z();
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }

private:
    //Create a variable to hold the VBO identifier
    GLuint triangleVBO;

    bool uploaded;

    void upload();

};

#endif // FRAME_H

#ifndef FRAME_H
#define FRAME_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

#define GLFW_INCLUDE_GLU
//#include <OpenGL/glu.h>
#include <GLFW/glfw3.h>

#include "common.h"

using namespace Eigen;
using namespace std;

struct OutgoingEdge{
    int neighbourIdx;
    float weight; //==error of correspondances
    vector< std::pair<int,int> > correspondances;  //src dst in camera coordinate frame
    Isometry3f P_relative;
};

class Frame
{
public:
    Frame();
    Frame(const std::string filename);
    ~Frame();

    vector<Vector3d> pts;
    vector<Vector3d> nor;

    Isometry3d pose = Isometry3d::Identity();
    Isometry3d poseGroundTruth = Isometry3d::Identity();

    vector<OutgoingEdge> neighbours;

    void draw();
private:
    //Create a variable to hold the VBO identifier
    GLuint triangleVBO;

    void upload();

};

#endif // FRAME_H

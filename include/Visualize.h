#ifndef __PointPairFeatures__Visualize__
#define __PointPairFeatures__Visualize__

#define GLFW_INCLUDE_GLU
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <math.h>
#include <vector>
#include <thread>
#include <Eigen/Dense>
#include "frame.h"


using namespace Eigen;
using namespace std;

namespace Colormap {

static Vector4d MAG1=Vector4d(0.3,0,0.8,1);
static Vector4d MAG2=Vector4d(0.1,0.0,0.5,0.5);
static Vector4d RED1=Vector4d(0.8,0,0.4,1);
static Vector4d RED2=Vector4d(0.4,0.0,0.1,0.5);
static Vector4d GREEN1=Vector4d(0.0,1.0,0.5,1);
static Vector4d GREEN2=Vector4d(0.0,.4,0.1,0.5);
static Vector3d BLUE1=Vector3d(0.0,0,0.5);
static Vector3d BLUE2=Vector3d(0.0,0,0.8);
static Vector3d ORANGE1=Vector3d(1,0.6,0);
static Vector3d ORANGE2=Vector3d(1,0.6,0);
}

class Visualize {

public:
    static bool waitKey(unsigned char key); //wait in another thread for keypress in opengl window

    //mimics opencv's Viz
    static void spin();
    static void spin(int iterations);
    static void spinToggle(int iterations);
    static void spinLast();

    static Visualize* getInstance();

    static void setClouds(vector< shared_ptr<Frame> >* mypair);

    static void setCentroid(Vector3d cent);


    static void setCallbackForKey(char key, std::function<void()> f);

    static void setSelectedIndex(int i);

    int selectedFrame=255;
    int selectedOutgoingEdgeIdx=255;
    int ingoingEdgeFrame=255;

    //const static int WINDOW_SIZE = 800;
    const static int WINDOW_WIDTH = 768;//1024;
    const static int WINDOW_HEIGHT = 768;

private:
    int refPtIdx=0;
    Visualize(); // singleton, acces via factory

    vector< shared_ptr<Frame> >* frames;

    Vector3d centroid;//=Vector3d::Zero();

    void drawFrustumIntrinsics(Vector4d colorLine, Vector4d colorPlane);

    bool keyToggle[256];//key toggle states
    std::function<void()> functions[256]; //functions to call on keys

    int modifier;
    
    GLdouble angle;   /* in degrees */
    GLdouble angle2;   /* in degrees */
    GLdouble angle3;   /* in degrees */
    GLdouble zoom;
    GLdouble offsetY;
    GLdouble offsetX;
    GLdouble offsetZ;

    int mouseButton;
    int moving;
    double startx, starty;

    void setWindowFPS();

    int nbFrames;
    double lastTime;
    
    void bucketInfo();

    static Visualize *instance;

    GLFWwindow* window;
    
    unsigned char lastKey;

    bool waitKeyInst(unsigned char key);

    void glColorHex(int rgbHex);

    void drawCylinderAdvanced(double r, double l, bool coverback, bool coverfront, bool normalInwards);
    void drawCylinder(double r, double l);
    void drawOrigin();

    void drawNormals(const Frame* m, Vector3d& color);
    void drawFrame(Frame* C, int i);

    void drawPoints(const vector<Vector3d>& pts, const Vector3d& color, float pointSize = 4.0f);
    void drawPoints(const vector<Vector3d>& pts, const vector<Vector3d>& colors, float pointSize = 4.0f);

    void drawLines(const vector<Vector3d>& v1, const vector<Vector3d>& v2);
    void drawCubes(const vector<Vector3d>& C, double size);
    void drawSpheres(const vector<Vector3d>& C, double radius);

    //void drawLines(const vector<int>& vertices);

    void drawCameraPose(Isometry3d& P,int i,Vector4d& colorLine,Vector4d& colorPlane);
    void drawCameraPoses(vector<Isometry3d>& cameraPoses,Vector4d& colorLine, Vector4d& colorPlane);

    void drawEdges(int i);

    void drawAll(Frame* m, Vector3d color, Vector3d colorNormals);

    double getRotationAngleApprox(double xdiff, double ydiff, double x, double y);

    void display(GLFWwindow* window);
    void keyboard(GLFWwindow* window, int key, int scancode, int action, int modifiers);
    void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
    void cursor_position_callback(GLFWwindow* window, double x, double y);
    void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
    void drop_callback(GLFWwindow* window, int count, const char** paths);




    static void displayW(GLFWwindow* window);
    static void keyboardW (GLFWwindow* window, int key, int scancode, int action, int modifiers);
    static void mouse_button_callbackW(GLFWwindow* window, int button, int action, int mods);
    static void cursor_position_callbackW(GLFWwindow* window, double x, double y);
    static void scroll_callbackW(GLFWwindow* window, double xoffset, double yoffset);
    static void drop_callbackW(GLFWwindow* window, int count, const char** paths);

};

#endif /* defined(__PointPairFeatures__Visualize__) */


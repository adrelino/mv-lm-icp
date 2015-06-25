#include "frame.h"

Frame::Frame(const std::string filename)
{
    pts=loadPLY(filename);
    upload();
}

Frame::Frame(){

}

Frame::~Frame()
{
//    delete[] pts;
    cout<<"delete frame: "<<triangleVBO<<endl;
    glDeleteBuffers(1, &triangleVBO);

}

void Frame::upload(){
    //http://www.songho.ca/opengl/gl_vbo.html
    //Vertices of a triangle (counter-clockwise winding)
    //float data[] = {1.0, 0.0, 1.0, 0.0, 0.0, -1.0, -1.0, 0.0, 1.0};
    //try float data[] = {0.0, 1.0, 0.0, -1.0, -1.0, 0.0, 1.0, -1.0, 0.0}; if the above doesn't work.

    //Create a new VBO and use the variable id to store the VBO id
    glGenBuffers(1, &triangleVBO);

    //Make the new VBO active
    glBindBuffer(GL_ARRAY_BUFFER, triangleVBO);

    //Upload vertex data to the video device
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vector3d)*pts.size(), pts.data(), GL_STATIC_DRAW);
}

void Frame::draw(){
    //Make the new VBO active. Repeat here incase changed since initialisation
    glBindBuffer(GL_ARRAY_BUFFER, triangleVBO);

    //Draw Triangle from VBO - do each time window, view point or data changes
    //Establish its 3 coordinates per vertex with zero stride in this array; necessary here
    glVertexPointer(3, GL_DOUBLE, 0, NULL);

    //Establish array contains vertices (not normals, colours, texture coords etc)
    glEnableClientState(GL_VERTEX_ARRAY);

    //Actually draw the triangle, giving the number of vertices provided
    glDrawArrays(GL_POINTS, 0, pts.size());

    glDisableClientState(GL_VERTEX_ARRAY);
}



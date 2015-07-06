#ifndef FRUSTUM_H
#define FRUSTUM_H

#define deg2rad2(d) (M_PI*(d)/180)

///////////////////////////////////////////////////////////////////////////////
// draw frustum
///////////////////////////////////////////////////////////////////////////////
void drawFrustum(float fovY, float aspectRatio, float nearPlane, Vector4f colorLineV, Vector4f colorPlaneV)//, float farPlane)
{
    float tangent = tanf(deg2rad2(fovY/2));
    float nearHeight = nearPlane * tangent;
    float nearWidth = nearHeight * aspectRatio;
//    float farHeight = farPlane * tangent;
//    float farWidth = farHeight * aspectRatio;

    // compute 8 vertices of the frustum
    float vertices[4][3];
    // near top right
    vertices[0][0] = nearWidth;     vertices[0][1] = nearHeight;    vertices[0][2] = -nearPlane;
    // near top left
    vertices[1][0] = -nearWidth;    vertices[1][1] = nearHeight;    vertices[1][2] = -nearPlane;
    // near bottom left
    vertices[2][0] = -nearWidth;    vertices[2][1] = -nearHeight;   vertices[2][2] = -nearPlane;
    // near bottom right
    vertices[3][0] = nearWidth;     vertices[3][1] = -nearHeight;   vertices[3][2] = -nearPlane;
//    // far top right
//    vertices[4][0] = farWidth;      vertices[4][1] = farHeight;     vertices[4][2] = -farPlane;
//    // far top left
//    vertices[5][0] = -farWidth;     vertices[5][1] = farHeight;     vertices[5][2] = -farPlane;
//    // far bottom left
//    vertices[6][0] = -farWidth;     vertices[6][1] = -farHeight;    vertices[6][2] = -farPlane;
//    // far bottom right
//    vertices[7][0] = farWidth;      vertices[7][1] = -farHeight;    vertices[7][2] = -farPlane;

    //float colorLine1[4] = { 0.7f, 0.7f, 0.7f, 0.7f };
    //float colorLine2[4] = { 0.2f, 0.2f, 0.2f, 0.7f };
    //float colorPlane[4] = { 0.5f, 0.5f, 0.5f, 0.5f };

    //glDisable(GL_LIGHTING);
    //glDisable(GL_CULL_FACE);
    //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // draw the edges around frustum
    glLineWidth(1);
    //glColor3f(1,1,1);
    glBegin(GL_LINES);
    glColor4fv(colorLineV.data());
    glVertex3f(0, 0, 0);
    //glColor4fv(colorLine1);
    glVertex3fv(vertices[0]);

    //glColor4fv(colorLine2);
    glVertex3f(0, 0, 0);
    //glColor4fv(colorLine1);
    glVertex3fv(vertices[1]);

    //glColor4fv(colorLine2);
    glVertex3f(0, 0, 0);
    //glColor4fv(colorLine1);
    glVertex3fv(vertices[2]);

    //glColor4fv(colorLine2);
    glVertex3f(0, 0, 0);
    //glColor4fv(colorLine1);
    glVertex3fv(vertices[3]);
    glEnd();

    //glColor4fv(colorLine1);
//    glBegin(GL_LINE_LOOP);
//    glVertex3fv(vertices[4]);
//    glVertex3fv(vertices[5]);
//    glVertex3fv(vertices[6]);
//    glVertex3fv(vertices[7]);
//    glEnd();

    //glColor4fv(colorLine1);
    glBegin(GL_LINE_LOOP);
    glVertex3fv(vertices[0]);
    glVertex3fv(vertices[1]);
    glVertex3fv(vertices[2]);
    glVertex3fv(vertices[3]);
    glEnd();

    // draw near and far plane
    glColor4fv(colorPlaneV.data());
    //glColor3f(0,0,0);
    glBegin(GL_QUADS);
    glVertex3fv(vertices[0]);
    glVertex3fv(vertices[1]);
    glVertex3fv(vertices[2]);
    glVertex3fv(vertices[3]);
//    glVertex3fv(vertices[4]);
//    glVertex3fv(vertices[5]);
//    glVertex3fv(vertices[6]);
//    glVertex3fv(vertices[7]);
    glEnd();

    //glEnable(GL_CULL_FACE);
    //glEnable(GL_LIGHTING);
}


#endif // FRUSTUM_H

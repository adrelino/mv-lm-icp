#include "Visualize.h"
#include <iostream>
#include <iomanip>

#include "frustum.h"

using namespace std;

GLubyte colorEdge[4]={254,122,0,14};
GLubyte colorEdgeSel[4]={122,254,0,15};

#ifndef gluPerspective
//https://stackoverflow.com/questions/12943164/replacement-for-gluperspective-with-glfrustrum
void gluPerspective( GLdouble fovY, GLdouble aspect, GLdouble zNear, GLdouble zFar )
{
    const GLdouble pi = 3.1415926535897932384626433832795;
    GLdouble fW, fH;

    //fH = tan( (fovY / 2) / 180 * pi ) * zNear;
    fH = tan( fovY / 360 * pi ) * zNear;
    fW = fH * aspect;

    glFrustum( -fW, fW, -fH, fH, zNear, zFar );
}
#endif //gluPerspective

void drawText(string s, Vector3d posMiddle){
    const unsigned char *text = (const unsigned char*) s.c_str();
    glRasterPos3dv(posMiddle.data());
//    glutBitmapString(GLUT_BITMAP_HELVETICA_10,text);
}

void Visualize::setWindowFPS ()
{
  // Measure speed
  double currentTime = glfwGetTime ();
  nbFrames++;

  if ( currentTime - lastTime >= 1.0 ){ // If last cout was more than 1 sec ago
    glfwSetWindowTitle (window, std::to_string(1000.0f / (float)nbFrames).c_str());

    nbFrames = 0;
    lastTime += 1.0;
  }
}

Visualize* Visualize::instance = 0;

Visualize* Visualize::getInstance(){
    if(instance == 0){
        instance = new Visualize();
    }
    return instance;
}

Visualize::Visualize()
    : modifier(-1)
    , angle(0.0)
    , angle2(0.0)
    , angle3(0.0)
    , mouseButton(0)
    , zoom(15)
    , offsetY(-0.075)
    , offsetX(0)
    , offsetZ(0)
    , moving(false)
    //, current_object(0)
{
    std::fill(std::begin(keyToggle), std::end(keyToggle), false);
    std::cout<<"Visualize construct"<<std::endl;
    keyToggle['s']=true;
    keyToggle['h']=true;
    keyToggle['c']=true;
    keyToggle['e']=true;
    keyToggle['a']=true;
    keyToggle['m']=true;
    keyToggle['l']=true; //lines for icp
    keyToggle['t']=true; //trajectory
    keyToggle['p']=true; //poses
    keyToggle['d']=true; //downsampled cloud
    keyToggle['r']=true; //keyToggle
    keyToggle['g']=true;

    /* Initialize the library */
    if (!glfwInit()) exit(-1);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "GLFW PointCloud Viewer", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        exit(-1);
    }

    glfwMakeContextCurrent(window);
    gladLoadGL();
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		exit(-1);
	}
	fprintf(stderr, "OpenGL %s\n", glGetString(GL_VERSION));

    glfwSwapInterval(1);

    int window_width, window_height;
    glfwGetFramebufferSize(window, &window_width, &window_height);
    glViewport(0, 0, window_width, window_height);

    // initialize the GL library
    // pixel storage/packing stuff
//    glPixelStorei(GL_PACK_ALIGNMENT, 1); // for glReadPixels​
//    glPixelStorei(GL_UNPACK_ALIGNMENT, 1); // for glTexImage2D​
//    glPixelZoom(1.0, -1.0);

    // enable and set colors
    glEnable(GL_COLOR_MATERIAL);
    glClearColor(1, 1, 1, 1.0);

    // enable and set depth parameters
    glEnable(GL_DEPTH_TEST);
//    glClearDepth(1.0);

    // light parameters
    GLfloat light_pos[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    GLfloat light_amb[] = { .5f, .5f, .5f, .5f};
    GLfloat light_dif[] = { .9f, 0.9f, 0.9f, 1.0f };
    GLfloat light_spec[] = { .5f, 0.5f, 0.5f, 1.0f };

    // enable lighting
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_amb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_dif);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_spec);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glShadeModel(GL_SMOOTH);


    glDisable(GL_CULL_FACE);
//    glEnable(GL_DEPTH_TEST);
//    glDepthMask(GL_TRUE);
    glMatrixMode(GL_PROJECTION);

    gluPerspective( /* field of view in degree */ 40.0,
                   /* aspect ratio */ 1.0,
                   /* Z near */ 1.0, /* Z far */ 80.0);

    glMatrixMode(GL_MODELVIEW);

    glfwSetMouseButtonCallback(window,mouse_button_callbackW);
    glfwSetScrollCallback(window,scroll_callbackW);
    glfwSetCursorPosCallback(window, cursor_position_callbackW);
    glfwSetDropCallback(window, drop_callbackW);
//    glfwSetCharCallback(window,keyboardW);
    glfwSetKeyCallback(window,keyboardW);
}

//draws cylinder towards z direction
void Visualize::drawCylinderAdvanced(double r, double l, bool coverback, bool coverfront, bool normalInwards){
    int i;
	int n = 20;
    
	int z=normalInwards ? -1 : 1;
    
    for(i=0;i<2*n;i++){
        glBegin(GL_POLYGON);
        // Explanation: the normal of the whole polygon is the coordinate of the center of the polygon for a sphere
        glNormal3d(sin((i+0.5)*M_PI/n)*z,cos((i+0.5)*M_PI/n)*z,0); //middle
        glVertex3d(r*sin(i*M_PI/n),r*cos(i*M_PI/n),0);
        glVertex3d(r*sin((i+1)*M_PI/n),r*cos((i+1)*M_PI/n),0);
        glVertex3d(r*sin((i+1)*M_PI/n),r*cos((i+1)*M_PI/n),l);
        glVertex3d(r*sin(i*M_PI/n),r*cos(i*M_PI/n),l);
        glEnd();
        //bottom
        if(coverback){
            glBegin(GL_POLYGON);
            glNormal3d(0,0,-1*z); //middle
            glVertex3d(r*sin(i*M_PI/n),r*cos(i*M_PI/n),0);
            glVertex3d(r*sin((i+1)*M_PI/n),r*cos((i+1)*M_PI/n),0);
            glVertex3d(0,0,0);
            glEnd();
        }
        //top
        if(coverfront){
            glBegin(GL_POLYGON);
            glNormal3d(0,0,1*z); //middle
            glVertex3d(r*sin(i*M_PI/n),r*cos(i*M_PI/n),l);
            glVertex3d(r*sin((i+1)*M_PI/n),r*cos((i+1)*M_PI/n),l);
            glVertex3d(0,0,l);
            glEnd();
        }
    }
    
}

void Visualize::drawCylinder(double r, double l)
{
	drawCylinderAdvanced(r,l,true,true,false);
}

void Visualize::drawOrigin(){
	glPushMatrix();
    glColor3d(0,0,1);
	drawCylinder(0.02,1); //z blue
	glRotatef(90,0,1,0);
    glColor3d(1,0,0);
	drawCylinder(0.02,1); //x  red
	glRotatef(-90,1,0,0);
    glColor3d(0,1,0);
	drawCylinder(0.02,1); //y  green
	glPopMatrix();
}

void Visualize::drawNormals(const Frame* m, Vector3d& color){
    if(m->nor.size()==0) return;
    glLineWidth(1);

    glBegin(GL_LINES);
    glColor3dv(color.data());
    for (int i=0; i<m->pts.size(); i++) {
        const Vector3d& p=m->pts[i];
        const Vector3d& n=m->nor[i];

        Vector3d q=p+n/100;
        
        glVertex3dv(p.data());
        glVertex3dv(q.data());
    }
	glEnd();
    
}

void Visualize::drawFrame(Frame* m, int i){

    glPushMatrix();


    if(keyToggle['g']) drawCameraPose(m->poseGroundTruth,i,Colormap::GREEN1,Colormap::GREEN2);  //those are fixed

       drawCameraPose(m->pose,i,Colormap::RED1,Colormap::RED2);

        Vector3d color(0.6,0.1,0);
        Vector3d colorNormals(0.8,0.2,0);

        if(selectedFrame==i){
            color = Colormap::BLUE1;
            colorNormals = (Colormap::BLUE2);
        }

//        if(ingoingEdgeFrame==i){
//            color = Colormap::ORANGE1;
//            colorNormals = (Colormap::ORANGE2);
//        }


//        if(keyToggle['c'] || selectedFrame==i || ingoingEdgeFrame==i){
        glColor3dv(color.data());
        m->draw();
        glPushMatrix();
               glMultMatrixd(m->pose.matrix().data());
//                if(keyToggle['d']){ //downsampled

//                    drawPoints(m->pts,color);
                   if(keyToggle['n']) drawNormals(m,colorNormals);
//                }
            glPopMatrix();
//        }

    //correct
    //drawPoints(m->getPtsInGlobalFrame(),Vector3d(0,1,0),pointSize+1);

                   glPopMatrix();
}

void Visualize::drawPoints(const vector<Vector3d>& pts, const Vector3d& color, float pointSize){
    glPointSize(pointSize);
    glColor3dv(color.data());
    glBegin(GL_POINTS);
    for (int i=0; i<pts.size(); i++) {
        glVertex3dv(pts[i].data());
    }
	glEnd();
}

void Visualize::drawPoints(const vector<Vector3d>& pts, const vector<Vector3d>& colors, float pointSize){
    glPointSize(pointSize);
    glBegin(GL_POINTS);
    for (int i=0; i<pts.size(); i++) {
        glColor3dv(colors[i].data());
        glVertex3dv(pts[i].data());
    }
    glEnd();
}

void Visualize::drawLines(const vector<Vector3d>& v1, const vector<Vector3d>& v2)
{
    glLineWidth(0.05f);

    int n = v1.size();

    for (int i = 0; i < n; ++i) {
        const Vector3d& p1=v1[i];
        const Vector3d& p2=v2[i];

        glBegin(GL_LINES);
        glColor3d(0.8,0,0.8);
        glVertex3dv(p1.data());  //distance line
        glVertex3dv(p2.data());  //distance line
        glEnd();
//        stringstream ss;
//        ss<<setprecision(2)<<(p1-p2).norm()*1000<<"mm";
//        drawText(ss.str(),0.5f*(p1+p2));

    }

}

void Visualize::drawSpheres(const vector<Vector3d>& pts, double radius){
    glColor3d(0.9,0.9,0.9);
    for (int i=0; i<pts.size(); i++) {
        glPushMatrix();
        glTranslatef(pts[i].x(),pts[i].y(),pts[i].z());
//        glutWireSphere(radius,5,5);
        glPopMatrix();
    }
}

void Visualize::drawFrustumIntrinsics(Vector4d colorLine, Vector4d colorPlane){
//    cv::Matx33d K = cv::Matx33d::zeros();
//    K(0,0) = 542;
//    K(1,1) = 540;
//    K(0,2) = 320;
//    K(1,2) = 240;
    double f_x =524;// K(0,0),
    double f_y = 540; //K(1,1),
    double c_x = 320; //K(1,2);
    double c_y = 240; //K(1,2);

    // Assuming that this is an ideal camera (c_y and c_x are at the center of the image)
    double fovy = 2.0 * atan2(c_y, f_y) * 180 / M_PI;
    double aspect_ratio = c_x / c_y;

    drawFrustum(fovy,aspect_ratio,-0.02f,colorLine.cast<float>(),colorPlane.cast<float>());
}

double Visualize::getRotationAngleApprox(double xdiff, double ydiff, double x, double y){
	int xs=x>0 ? 1 : -1;
	int ys=y>0 ? 1 : -1;
	std::cout<<x<<"+"<<xdiff<<" "<<y<<"+"<<ydiff<<std::endl;
	return ydiff*xs+xdiff*ys;
}

void Visualize::drawCameraPose(Isometry3d& P,int i,Vector4d& colorLine,Vector4d& colorPlane){
    if(keyToggle['p']){
        glEnable(GL_STENCIL_TEST);
        glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

        //Vector3d t = -P.inverse().translation();
        glPushMatrix();

        //glMultMatrixd(Isometry3d(P.linear()).matrix().data());
        //glTranslatef(t(0),t(1),t(2));

        glMultMatrixd(P.matrix().data());
        glStencilFunc(GL_ALWAYS, i, GL_ALL_ATTRIB_BITS);

        if(selectedFrame==i){
            drawFrustumIntrinsics(colorLine,Vector4d(0,0,0.5,0.5));
        }else{
            drawFrustumIntrinsics(colorLine,colorPlane);

        }
        glPopMatrix();
    }

    if(keyToggle['T']){
        std::stringstream ss;
        ss<<i;
        const unsigned char *text = (const unsigned char*) ss.str().c_str();
        Vector3d origin=Vector3d::Zero();
        Vector3d pos = P*origin;

        glColor4dv(colorLine.data());
        if(selectedFrame==i){ //dont know why we need -1 here, but it works
            //cout<<"sel: "<<selectedFrame<<" i"<<i<<" s1="<<cameraPoses.size()<<" s2="<<cameraPosesGroundTruth.size()<<endl;
            glColor3d(0,0,1);
        }
        glRasterPos3dv(pos.data());
        glStencilFunc(GL_ALWAYS, i, GL_ALL_ATTRIB_BITS);
//        glutBitmapString(GLUT_BITMAP_HELVETICA_18,text);


//        glPushMatrix();
//            glMultMatrixd(P.matrix().data());
//            glRasterPos3d(0,0,0);
//            glColor3d(0,1,0);
//            if(selectedFrame-1==i){ //dont know why we need -1 here, but it works
//                //cout<<"sel: "<<selectedFrame<<" i"<<i<<" s1="<<cameraPoses.size()<<" s2="<<cameraPosesGroundTruth.size()<<endl;
//                glColor3d(1,0,0);
//            }
//            glStencilFunc(GL_ALWAYS, i, GL_ALL_ATTRIB_BITS);
//            glutBitmapString(GLUT_BITMAP_HELVETICA_18,text);
//        glPopMatrix();
    }
}

void Visualize::drawEdges(int i){
    shared_ptr<Frame>& v1=(*frames)[i];

    Vector3d origin=Vector3d::Zero();
    Vector3d pos1 = v1->pose*origin;

    for(int k=0; k<v1->neighbours.size(); k++){
        OutgoingEdge pair = v1->neighbours[k];
        int j=pair.neighbourIdx;

        shared_ptr<Frame>& v2=(*frames)[j];
        Vector3d pos2 = v2->pose*origin;



        Vector3d posMiddle(pos1*0.25f+pos2*0.75f);
        if(keyToggle['W']) posMiddle.y() += i>j ? 0.02f : -0.02f;

        if(keyToggle['w']){//weights
            stringstream ss;
            ss<</*setprecision(3)<<*/pair.weight;
            //ss<<pair.src.size();

            const unsigned char *text = (const unsigned char*) ss.str().c_str();

            if(i==selectedFrame && k==selectedOutgoingEdgeIdx){
                GLubyte color[4]={colorEdgeSel[0],colorEdgeSel[1],colorEdgeSel[2],(GLubyte) k};
                glColor4ubv(color);
            }else{
                GLubyte color[4]={colorEdge[0],colorEdge[1],colorEdge[2],(GLubyte) k};
                glColor4ubv(color);
            }
            glRasterPos3dv(posMiddle.data());
    //        if(selectedFrame-1==i){ //dont know why we need -1 here, but it works
    //            //cout<<"sel: "<<selectedFrame<<" i"<<i<<" s1="<<cameraPoses.size()<<" s2="<<cameraPosesGroundTruth.size()<<endl;
    //            glColor3d(0,0,1);
    //        }

            glStencilFunc(GL_ALWAYS, i, GL_ALL_ATTRIB_BITS);
//            glutBitmapString(GLUT_BITMAP_HELVETICA_18,text);
        }



        glLineWidth(i==selectedFrame && k==selectedOutgoingEdgeIdx ? 3 : 1);

        glBegin(GL_LINE_STRIP);
        if(i==selectedFrame && k==selectedOutgoingEdgeIdx){
            glColor4dv(Colormap::MAG2.data());
        }else{
            glColor4dv(Colormap::MAG1.data());
        }
        glVertex3dv(pos1.data());
        glVertex3dv(posMiddle.data());
        glVertex3dv(pos2.data());
        glEnd();


        if(keyToggle['l'] && i==selectedFrame && k==selectedOutgoingEdgeIdx){
            vector<Vector3d> src,dst;
            for(auto corr : pair.correspondances){
                src.push_back(v1->pose*v1->pts[corr.first]);
                dst.push_back(v2->pose*v2->pts[corr.second]);
            }
            //drawPoints(pair.src,Colormap::BLUE1,6.0f);
            //drawPoints(pair.dst,Colormap::ORANGE1,6.0f);
            drawLines(src,dst);
        }
    }
}

void Visualize::mouse_button_callbackW(GLFWwindow* window, int button, int action, int mods){
    instance->mouse_button_callback(window,button,action,mods);
}

void Visualize::mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    //cout<<button<<"\t"<<action<<"\t"<<mods<<endl;

    modifier=mods;

    if (action == GLFW_PRESS) {
        mouseButton = button;
        moving = 1;
        glfwGetCursorPos(window, &startx, &starty);
    }else if (action == GLFW_RELEASE) {
        mouseButton = button;
        moving = 0;
    }
}

void Visualize::scroll_callbackW(GLFWwindow* window, double xoffset, double yoffset){
    instance->scroll_callback(window,xoffset,yoffset);
}

void Visualize::scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    //cout<<xoffset<<","<<yoffset<<endl;
    zoom += (xoffset+yoffset)*0.1;
}

void Visualize::cursor_position_callbackW(GLFWwindow* window, double x, double y){
    instance->cursor_position_callback(window,x,y);
}


void Visualize::cursor_position_callback(GLFWwindow* window, double x, double y)
{
    //cout<<x<<","<<y<<endl;

    if (moving) {
        if (modifier==GLFW_MOD_ALT) { //Panning
            if(mouseButton==GLFW_MOUSE_BUTTON_LEFT){
                offsetY+=(y-starty)*0.001;
                offsetX+=(x-startx)*0.001;
            }else if(mouseButton==GLFW_MOUSE_BUTTON_MIDDLE || mouseButton==GLFW_MOUSE_BUTTON_RIGHT){
                offsetZ+=((x-startx)+(y-starty))*0.001;
            }
        }else{
            if(mouseButton==GLFW_MOUSE_BUTTON_LEFT){
//                if (modifier==GLUT_ACTIVE_CTRL) {
//                    offsetY+=(y-starty)*0.001;
//                    offsetX+=(x-startx)*0.001;
//                }else if (modifier==GLUT_ACTIVE_ALT){
//                    offsetZ+=(x-startx)*0.001;
//                }else{
                    angle = angle + (x - startx);
                    angle2 = angle2 + (y - starty);
//                }
            }else if(mouseButton==GLFW_MOUSE_BUTTON_MIDDLE || (mouseButton==GLFW_MOUSE_BUTTON_RIGHT && modifier==GLFW_MOD_ALT)){ //if we dont have mousewheel
                int xCenter=x-(WINDOW_WIDTH/2);
                int yCenter=y-(WINDOW_HEIGHT/2);
                int startxCenter=startx-(WINDOW_WIDTH/2);
                int startyCenter=starty-(WINDOW_HEIGHT/2);
                angle3 -=(atan2(yCenter,xCenter)-atan2(startyCenter,startxCenter))*(180/M_PI); //rotate object around z axis with the angle corresponding to polar coordinates of mouse displacement
                //angle3 -= getRotationAngleApprox(xCenter-startxCenter,startyCenter-yCenter,xCenter,yCenter);
            }else if(mouseButton==GLFW_MOUSE_BUTTON_RIGHT){
                zoom += (((y-starty)+(x-startx))*0.1); //allows to mirror the object if zoom <0
            }
        }
        startx = x;
        starty = y;
        //glutPostRedisplay();
    }
}

void Visualize::drop_callbackW(GLFWwindow* window, int count, const char** paths){
    instance->drop_callback(window,count,paths);
}

void Visualize::drop_callback(GLFWwindow* window, int count, const char** paths)
{
    int i;
    frames->resize(count);
    for (i = 0;  i < count;  i++){
        cout<<paths[i]<<endl;
//        shared_ptr<Frame> currentFrame(new Frame(paths[i]));//loadPLY(paths[i]);
//        (*frames)[i]=currentFrame;
    }
}

void Visualize::display(GLFWwindow* window)
{
//    glClearStencil(255); //is background for mouse clicks
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glPushMatrix();


    glTranslated(0, 0, -15);

    glRotated(angle3, 0.0, 0.0, 1.0);
    glRotated(angle2, 1.0, 0.0, 0.0);
    glRotated(angle, 0.0, 1.0, 0.0);
    
    double z = zoom;
    glScaled(z,z,z);
//    if(keyToggle['R']) drawOrigin(); //rotation center

//    glTranslated(-centroid.x(),-centroid.y(),-centroid.z());

//    glMultMatrixd(S.matrix().data());

    if(keyToggle['o']) drawOrigin();

//    glBegin(GL_TRIANGLES);
//    glColor3f(1.f, 0.f, 0.f);
//    glVertex3f(-0.6f, -0.4f, 0.f);
//    glColor3f(0.f, 1.f, 0.f);
//    glVertex3f(0.6f, -0.4f, 0.f);
//    glColor3f(0.f, 0.f, 1.f);
//    glVertex3f(0.f, 0.6f, 0.f);
//    glEnd();
    if(frames){
        for(int i=0; i<frames->size(); i++){
//            frames->at(i)->draw();
            shared_ptr<Frame>& it=(*frames)[i];
//            if(it){
                drawFrame(it.get(),i);
//            }
        }
    }

    if(keyToggle['e'] && frames){ //edges in pose graph
        for(int i=0; i<frames->size(); i++){
            drawEdges(i);
        }
    }

	glPopMatrix();
}

void Visualize::displayW(GLFWwindow* window){
    instance->display(window);
}

void Visualize::keyboard(GLFWwindow* window, int keyInt, int scancode, int action, int modifiers)
{
    if(action != GLFW_PRESS) return;
//    if (key == GLFW_KEY_E && action == GLFW_PRESS)
//        activate_airship();
//}

//void Visualize::keyboard (unsigned char key, int x, int y)
//{
//    int modifiers = glutGetModifiers();


    unsigned char key = (unsigned char) keyInt;


    //modifiers & GLUT_ACTIVE_SHIFT
    bool ctrl = modifiers & GLFW_MOD_CONTROL;// GLUT_ACTIVE_CTRL;
    bool alt = modifiers & GLFW_MOD_ALT; //GLUT_ACTIVE_ALT;
    bool shift = modifiers & GLFW_MOD_SHIFT;
    if(!shift) key+=('a'-'A');

    std::cout<<"keyboard: "<<key;//< " at " << x <<"," << y;
    cout<<" val "<<keyInt;
    if(ctrl) cout<<" ctrl";
    if(alt) cout<<" alt ";
    if(shift) cout<<" shift ";
    cout<<endl;
    lastKey=key;

    keyToggle[key] = !keyToggle[key];

    if(functions[key]) functions[key]();

	switch (key) {
        case ' ':
            {   selectedFrame=255;
                selectedOutgoingEdgeIdx=255;
                ingoingEdgeFrame=255;
            }break;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case '0':
            break;
        case 'Q':
            exit(-1);
            break;
        case '+':
            refPtIdx++;
            break;
        case '-':
            refPtIdx--;
        default:
            break;
	}
}

void Visualize::keyboardW (GLFWwindow* window, int key, int scancode, int action, int modifiers){
    instance->keyboard(window,key,scancode,action,modifiers);
}

void Visualize::spin(){
    while(Visualize::waitKey('q') && !glfwWindowShouldClose(instance->window)){
        /* Render here */
        displayW(instance->window);

        /* Swap front and back buffers */
        glfwSwapBuffers(instance->window);

//        std::chrono::milliseconds dura( 5 );
//        std::this_thread::sleep_for( dura );

        /* Poll for and process events */
        glfwPollEvents();
    }
    if(glfwWindowShouldClose(instance->window)){
        glfwTerminate();
        exit(0);
    }
}

void Visualize::spinLast(){
    while(Visualize::waitKey('Q') && !glfwWindowShouldClose(instance->window)){
        /* Render here */
        displayW(instance->window);

        /* Swap front and back buffers` */
        glfwSwapBuffers(instance->window);

//        std::chrono::milliseconds dura( 5 );
//        std::this_thread::sleep_for( dura );

        /* Poll for and process events */
        glfwPollEvents();
    }

    glfwTerminate();
    if(glfwWindowShouldClose(instance->window)) exit(0);

}

void Visualize::spin(int i){
    while(i-- > 0){
        /* Render here */
        displayW(instance->window);

        /* Swap front and back buffers */
        glfwSwapBuffers(instance->window);

//        std::chrono::milliseconds dura( 5 );
//        std::this_thread::sleep_for( dura );

        /* Poll for and process events */
        glfwPollEvents();
    }
    if(glfwWindowShouldClose(instance->window)) exit(0);
}

void Visualize::spinToggle(int i){
    if(getInstance()->keyToggle['r']){
        spin(i);
    }else{
        spin();
    }
}

bool Visualize::waitKeyInst(unsigned char key){
    //cout<<"waiting for "<<key<<endl;
//    std::chrono::milliseconds dura( 10 );
    //while(true){
//        std::this_thread::sleep_for( dura );
//        setWindowFPS();
        //sleep(1);
        if(lastKey==key){
            lastKey=-1;
            //cout<<"ok, exit waiting = "<<endl;
            return false;
        }else{
            return true;
        }
    //}
}

bool Visualize::waitKey(unsigned char key){
    return getInstance()->waitKeyInst(key);
}

void Visualize::setClouds(vector< shared_ptr<Frame> >* mypair){
    getInstance()->frames=mypair;
}

void Visualize::setSelectedIndex(int i){
//    getInstance()->selectedFrame=i;
//    if(i<getInstance()->frames->size()){
//        Frame& cloud = *(getInstance()->frames->at(i));
//        if(cloud.neighbours.size()>0){
//            getInstance()->ingoingEdgeFrame=cloud.neighbours[0].neighbourIdx;
//        }
//    }
}

void Visualize::setCallbackForKey(char key, std::function<void ()> f){
    getInstance()->functions[key]=f;
}

void Visualize::setCentroid(Vector3d cent){
    getInstance()->centroid=cent;
}

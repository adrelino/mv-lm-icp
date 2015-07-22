/** @author Adrian Haarbach
 *
 * Comparison of pairwise ICP.
 * variants:
 * - point to point
 * - point to plane
 * minimizers:
 * - closed form solution/1st order approximation
 * - g2o with SO3 vertices and GICP edges
 * - ceres with angle axis
 * - ceres with eigen quaternion
 */

#include "common.h"
#include "gflags/gflags.h"
#include "CPUTimer.h"
#include "algorithms.h"

using namespace std;

DEFINE_bool(pointToPlane, false, "pointToPlane");

int main(int argc, char * argv[]){
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    vector<Vector3d> pts,nor;
    loadXYZ("../samples/scene.xyz",pts,nor);

    for (int i = 0; i < 10; ++i) {
        cout<<pts[i].transpose()<<"\t";
        cout<<nor[i].transpose()<<endl;
    }

    Matrix3Xd ptsMat = vec2mat(pts);
    Matrix3Xd norMat = vec2mat(nor);

    Quaterniond q=Quaterniond::Identity();
    q = q * AngleAxisd(M_PI_4, Vector3d::UnitX());
    q = q * AngleAxisd(1,Vector3d::UnitY());
    q = q * AngleAxisd(-0.2,Vector3d::UnitZ());

    Vector4d rot(q.x(),q.y(),q.z(),q.w());
    //Vector4d rot(.2,.2,.2,.4);
    //Vector3d tra(0,0,0);//
    Vector3d tra(.01,-0.01,-0.005);//,0.5,0.01);

    Isometry3d P = Translation3d(tra)*Quaterniond(rot);

//    for (int i = 0; i < 5; ++i) {

    Matrix3Xd ptsTra = P * ptsMat;
    vector<Vector3d> ptsTraVec = mat2vec(ptsTra);

    Isometry3d Ptest;
    Isometry3d PtestG2O;
    Isometry3d PtestCeres;
    Isometry3d PtestCeres2;

    CPUTimer timer;

    if(FLAGS_pointToPlane){
        Matrix3Xd norTra = P.linear() * norMat;
        vector<Vector3d> norTraVec = mat2vec(norTra);
        timer.tic();
        Ptest = ICP::pointToPlane(pts,ptsTraVec,norTraVec);
        timer.toc("closed plane");
        timer.tic();
        PtestG2O = ICPG2O::pointToPlane(pts,ptsTraVec,norTraVec);
        timer.toc("g2o plane");
        timer.tic();
        PtestCeres = ICPCeres::pointToPlane(pts,ptsTraVec,norTraVec);
        timer.toc("ceres plane CeresAngleAxis");
        timer.tic();
        PtestCeres2 = ICPCeres::pointToPlane_EigenQuaternion(pts,ptsTraVec,norTraVec);
        timer.toc("ceres plane EigenQuaternion");
    }else{
        timer.tic();
        Ptest = ICP::pointToPoint(pts,ptsTraVec);
        timer.toc("closed");
        timer.tic();
        PtestG2O = ICPG2O::pointToPoint(pts,ptsTraVec);
        timer.toc("g2o");
        timer.tic();
        PtestCeres = ICPCeres::pointToPoint_CeresAngleAxis(pts,ptsTraVec);
        timer.toc("ceres CeresAngleAxis");
        timer.tic();
        PtestCeres2 = ICPCeres::pointToPoint_EigenQuaternion(pts,ptsTraVec);
        timer.toc("ceres EigenQuaternion");
    }


    cout<<"groundtruth:"<<endl<<P.matrix()<<endl;
    cout<<"closed-form:"<<endl<<Ptest.matrix()<<endl;
    cout<<"closed: "<<poseDiff(P,Ptest)<<endl;

    cout<<"g2o:"<<endl<<PtestG2O.matrix()<<endl;
    cout<<"g2o: "<<poseDiff(P,PtestG2O)<<endl;

    cout<<"ceres:"<<endl<<PtestCeres.matrix()<<endl;
    cout<<"ceres: "<<poseDiff(P,PtestCeres)<<endl;

    cout<<"ceres eigen quaternion:"<<endl<<PtestCeres2.matrix()<<endl;
    cout<<"ceres eigen quaternion "<<poseDiff(P,PtestCeres2)<<endl;

    timer.printAllTimings();
}

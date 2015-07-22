#include "icp-g2o.h"

#include <eigen3/Eigen/Dense>
#include <math.h>
#include <unordered_map>
#include <vector>


#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
// #include <g2o/solvers/csparse/linear_solver_csparse.h>

#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>

#include <g2o/types/icp/types_icp.h>

#include <g2o/core/robust_kernel_impl.h>


namespace ICP_G2O {

using namespace g2o;
using namespace Eigen;
using namespace std;

Isometry3d pointToPoint(vector<Vector3d>&src,vector<Vector3d>&dst){
    SparseOptimizer optimizer;
    //optimizer.setVerbose(true);

    BlockSolverX::LinearSolverType* linearSolver = new LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    BlockSolverX* solver_ptr = new BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithm* solver;

    solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    VertexSE3 *vp0 = new VertexSE3(); //dstCloud
    vp0->setId(0);
    VertexSE3 *vp1 = new VertexSE3(); //srcCloud
    vp1->setId(1);

    vp0->setFixed(true);

    // add to optimizer
    optimizer.addVertex(vp0);
    optimizer.addVertex(vp1);

    for (int i = 0; i < src.size(); ++i) {
        Edge_V_V_GICP * e = new Edge_V_V_GICP();


        e->setVertex(0, vp0);      // first viewpoint : dstcloud, fixed
        e->setVertex(1, vp1);      // second viewpoint: srcCloud, moves

        EdgeGICP meas;
        meas.pos0 = dst[i].cast<double>();
        meas.pos1 = src[i].cast<double>();
//      meas.normal0 = Vector3d(0,0,1);
//      meas.normal1 = Vector3d(0,0,1);

//      meas.makeRot0();
//      meas.makeRot1();
        e->setMeasurement(meas);
        e->information().setIdentity();
        optimizer.addEdge(e);
    }

   optimizer.initializeOptimization();
   //optimizer.computeActiveErrors();
   optimizer.optimize(300);


    Isometry3d posefinal= vp1->estimate();//.cast<float>();

    return posefinal;
}

Isometry3d pointToPlane(vector<Vector3d> &src,vector<Vector3d> &dst,vector<Vector3d> &nor){
    SparseOptimizer optimizer;
    //optimizer.setVerbose(true);

    BlockSolverX::LinearSolverType* linearSolver = new LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    BlockSolverX* solver_ptr = new BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithm* solver;

    solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    VertexSE3 *vp0 = new VertexSE3(); //dstCloud
    vp0->setId(0);
    VertexSE3 *vp1 = new VertexSE3(); //srcCloud
    vp1->setId(1);

    vp0->setFixed(true);

    // add to optimizer
    optimizer.addVertex(vp0);
    optimizer.addVertex(vp1);

    for (int i = 0; i < src.size(); ++i) {
        Edge_V_V_GICP * e = new Edge_V_V_GICP();


        e->setVertex(0, vp0);      // first viewpoint : dstcloud, fixed
        e->setVertex(1, vp1);      // second viewpoint: srcCloud, moves

        EdgeGICP meas;
        meas.pos0 = dst[i].cast<double>();
        meas.pos1 = src[i].cast<double>();
        meas.normal0 = nor[i].cast<double>();
//      meas.normal1 = Vector3d(0,0,1);

        meas.makeRot0();
//      meas.makeRot1();
        e->setMeasurement(meas);
        e->information() = meas.prec0(0.01);
        optimizer.addEdge(e);
    }

   optimizer.initializeOptimization();
   //optimizer.computeActiveErrors();
   optimizer.optimize(300);


    Isometry3d posefinal= vp1->estimate();//.cast<float>();

    return posefinal;
}

void g2oOptimizer(vector< std::shared_ptr<Frame> >& frames, bool pointToPlane){

//    g2o::RobustKernelHuber* rk;
//    if(huberWidth>=0){
//        rk = new g2o::RobustKernelHuber();
//        rk->setDelta(huberWidth);
//    }

//    for (int round = 0; round < iter; ++round) {

//    cout<<"g2oOptimizer round "<<round<<endl;
//    Visualize::spin();

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithm* solver;
//    if(useLevenberg){
        solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
//    }else{
//        solver = new g2o::OptimizationAlgorithmDogleg(solver_ptr);
//    }

    optimizer.setAlgorithm(solver);

    for(int i=0; i<frames.size(); i++){
      // set up node
      g2o::VertexSE3 *vc = new g2o::VertexSE3();

      vc->setEstimate(frames[i]->pose);
      vc->setId(i);
      // set first cam pose fixed
      if (i==0){
          frames[i]->fixed=true;
          vc->setFixed(true);
      }

      // add to optimizer
      optimizer.addVertex(vc);
    }


// if(stopStages){
//     cout<<"graph structure romputed, press q to refine"<<endl;
//     Visualize::spin();
// }



    for(int src_id=0; src_id<frames.size(); src_id++){

        Frame& srcCloud = *frames[src_id];

        for (int j = 0; j < srcCloud.neighbours.size(); ++j) {

            OutgoingEdge& dstEdge = srcCloud.neighbours[j];
            Frame& dstCloud = *frames.at(dstEdge.neighbourIdx);

            int dst_id=dstEdge.neighbourIdx;

            g2o::VertexSE3* vp0 =
              dynamic_cast<g2o::VertexSE3*>(optimizer.vertices().find(dst_id)->second); //dstCloud
            g2o::VertexSE3* vp1 =
              dynamic_cast<g2o::VertexSE3*>(optimizer.vertices().find(src_id)->second); //srcCloud


            for(auto corr : dstEdge.correspondances){
                g2o::Edge_V_V_GICP * e = new g2o::Edge_V_V_GICP();
                //ppf_reconstruction::EdgeICP* e = new ppf_reconstruction::EdgeICP();

                e->setVertex(0, vp0);      // first viewpoint : dstcloud, fixed
                e->setVertex(1, vp1);      // second viewpoint: srcCloud, moves

                g2o::EdgeGICP meas;
                meas.pos0 = dstCloud.pts[corr.second];
                meas.pos1 = srcCloud.pts[corr.first];
                meas.normal0 = dstCloud.nor[corr.second];
                meas.normal1 = srcCloud.nor[corr.first];

                meas.makeRot0();
                meas.makeRot1();

//                if(huberWidth>=0){
//                    e->setRobustKernel(rk);
//                }

                e->setMeasurement(meas);

//                bool planeToPlane=false;



/*                if(planeToPlane){
                    e->pl_pl=true;
                }else */
                if(pointToPlane){
                    //meas = e->measurement();
                    // use this for point-plane
                    e->information() = meas.prec0(0.01);
                }else{
//                     use this for point-point
                    e->information().setIdentity();
                }

                optimizer.addEdge(e);

            }
        }
    }

   optimizer.initializeOptimization();

   optimizer.computeActiveErrors();
   double chiInit = optimizer.chi2(); //stop innerround if we get 100% better
   cout << "round: " << "s" << " chi2: " << FIXED(chiInit) << endl;

   double lastChi=chiInit;

   int noImpr=0;

   int iter = 100;


    for (int innerround = 0; innerround < iter; ++innerround) {
       optimizer.optimize(100);

       optimizer.computeActiveErrors();
       double newchi=optimizer.chi2();
       double impr = (lastChi-newchi)/lastChi;  //http://en.wikipedia.org/wiki/Relative_change_and_difference
       lastChi=newchi;

       cout << "round: " << innerround << " chi2: " << FIXED(newchi) << " impr: "<<impr<<endl;

       if(impr>0.0){
           cout<<"impr > 0%"<<endl;
//           if(Params::getInstance()->stopFrames){
//               for (int i = 0; i < frames.size(); ++i) {
//                   frames[i]->pose= dynamic_cast<VertexSE3*>(optimizer.vertices().find(i)->second)->estimate();
//               }
//               cout<<"press q to continue"<<endl;

//               //Visualize::simulateKeypress('k'); //calc error
////               Visualize::spinToggle(1);
//            }
       }else{
           noImpr++;
       }

       if(noImpr>5){
           cout<<"100 times no impr, break";
           break;
       }
    }

    for (int i = 0; i < frames.size(); ++i) {
        frames[i]->pose= dynamic_cast<g2o::VertexSE3*>(optimizer.vertices().find(i)->second)->estimate();
    }

}

}//end namespace

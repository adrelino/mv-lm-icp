#include <vector>
#include "frame.h"
#include "algorithms.h"

//Ceres

#include <ceres/local_parameterization.h>
#include <ceres/autodiff_local_parameterization.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/types.h>
#include <ceres/rotation.h>
#include <ceres/problem.h>
#include "eigen_quaternion.h"
#include <ceres/solver.h>
#include <ceres/loss_function.h>

//G2O

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

//#include "types_icp.h"
//#include "g2oTypeSim3Sophus.h"

//using namespace g2o;

namespace ApproachComponents{

void ceresOptimizer(vector< std::shared_ptr<Frame> >& frames, bool pointToPlane){

    ceres::Problem problem;

//    double* cameras = new double[frames.size()*7]; //4 quaternion, 3 translation

    vector<Eigen::Quaterniond> qs(frames.size());
    vector<Eigen::Vector3d> ts(frames.size());

    //extract initial camera poses
    //ceres::LocalParameterization
    eigen_quaternion::EigenQuaternionParameterization *quaternion_parameterization = new eigen_quaternion::EigenQuaternionParameterization;

    for(int i=0; i<frames.size(); i++){
      Isometry3d originalPose = frames[i]->pose;
      Eigen::Quaterniond q;// = Eigen::Map<Eigen::Quaterniond>(cameras+i*7);
      Eigen::Vector3d t;// = Eigen::Map<Eigen::Vector3d>(cameras+i*7+4);

      q=Eigen::Quaterniond(originalPose.linear());
      t=Eigen::Vector3d(originalPose.translation());

      qs[i]=q;
      ts[i]=t;

      if (i==0){
          frames[i]->fixed=true;
      }
    }

    cout<<"ok ceres"<<endl;

//    Visualize::spin(1);

    //add edges
    for(int src_id=0; src_id<frames.size(); src_id++){

        Frame& srcCloud = *frames[src_id];
        if(srcCloud.fixed) continue;

        Eigen::Quaterniond& srcQ = qs[src_id];
        Eigen::Vector3d& srcT = ts[src_id];

        for (int j = 0; j < srcCloud.neighbours.size(); ++j) {

            OutgoingEdge& dstEdge = srcCloud.neighbours[j];
            Frame& dstCloud = *frames.at(dstEdge.neighbourIdx);

            int dst_id=dstEdge.neighbourIdx;

            Eigen::Quaterniond& dstQ = qs[dst_id];
            Eigen::Vector3d& dstT = ts[dst_id]; //dstCloud

            for(auto corr : dstEdge.correspondances){

                // first viewpoint : dstcloud, fixed
                // second viewpoint: srcCloud, moves

                ceres::CostFunction* cost_function;

                if(pointToPlane){
                    cost_function = ICPCostFunctions::PointToPlaneErrorGlobal::Create(dstCloud.pts[corr.second],srcCloud.pts[corr.first],dstCloud.nor[corr.second]);
                }else{
                    cost_function = ICPCostFunctions::PointToPointErrorGlobal::Create(dstCloud.pts[corr.second],srcCloud.pts[corr.first]);
                }


//                problem.AddResidualBlock(cost_function, new ceres::HuberLoss(1.0), srcQ.coeffs().data(),srcT.data(),dstQ.coeffs().data(), dstT.data());
                problem.AddResidualBlock(cost_function, NULL, srcQ.coeffs().data(),srcT.data(),dstQ.coeffs().data(), dstT.data());

//                problem.AddResidualBlock(cost_function, NULL, &cameras[src_id*7],&cameras[src_id*7+4],&cameras[dst_id*7],&cameras[dst_id*7+4]);

            }
        }
    }

    for (int i = 0; i < frames.size(); ++i) {
//        problem.SetParameterization(&cameras[i*7],quaternion_parameterization);
        problem.SetParameterization(qs[i].coeffs().data(),quaternion_parameterization);
        if(frames[i]->fixed){
            std::cout<<i<<" fixed"<<endl;
//            problem.SetParameterBlockConstant(&cameras[i*7]);
//            problem.SetParameterBlockConstant(&cameras[i*7+4]);
            problem.SetParameterBlockConstant(qs[i].coeffs().data());
            problem.SetParameterBlockConstant(ts[i].data());
        }
    }

    // Set a few options
    ceres::Solver::Options options;

    options.use_nonmonotonic_steps = true;
    options.preconditioner_type = ceres::SCHUR_JACOBI;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.use_explicit_schur_complement=true;
    options.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

//    std::cout << "Final report:\n" << summary.FullReport();

//    std::cout <<"t:"<< t.transpose() << " q:"<<q.coeffs().transpose()<<endl;

//    cout<<"groundtruth: "<<endl<<frames[0]->poseGroundTruth.matrix()<<endl;

    //update camera poses
    for (int i = 0; i < frames.size(); ++i) {
//        if(frames[i]->fixed) continue;
        Isometry3d poseFinal = Isometry3d::Identity();
//        poseFinal.linear() = Eigen::Map<Eigen::Quaterniond>(cameras+i*7).toRotationMatrix();
//        poseFinal.translation() = Eigen::Map<Eigen::Vector3d>(cameras+i*7+4);

        poseFinal.linear() = qs[i].toRotationMatrix();
        poseFinal.translation() = ts[i];

        //cout<<"i: "<<i<<endl<<poseFinal.matrix()<<endl;


        frames[i]->pose=poseFinal;
    }


}


//using namespace ppf_reconstruction;
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

}//end ns



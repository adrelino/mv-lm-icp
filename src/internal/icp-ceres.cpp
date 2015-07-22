#include "icp-ceres.h"

#include <eigen3/Eigen/Dense>
#include <math.h>
#include <unordered_map>
#include <vector>

#include "Visualize.h"


#include <ceres/local_parameterization.h>
#include <ceres/autodiff_local_parameterization.h>
#include <ceres/types.h>
#include <ceres/rotation.h>
#include <ceres/ceres.h>

#include <ceres/loss_function.h>

namespace ICP_Ceres {

//http://ceres-solver.org/faqs.html#solving
ceres::Solver::Options getOptions(){
    // Set a few options
    ceres::Solver::Options options;
    //options.use_nonmonotonic_steps = true;
    options.preconditioner_type = ceres::IDENTITY;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 10;

//    options.use_nonmonotonic_steps = true;
//    options.preconditioner_type = ceres::SCHUR_JACOBI;
//    options.linear_solver_type = ceres::DENSE_SCHUR;
//    options.use_explicit_schur_complement=true;
//    options.max_num_iterations = 100;

    return options;
}
/*
 * For small (a few hundred parameters) or dense problems use DENSE_QR.

For general sparse problems (i.e., the Jacobian matrix has a substantial number of zeros) use SPARSE_NORMAL_CHOLESKY. This requires that you have SuiteSparse or CXSparse installed.

For bundle adjustment problems with up to a hundred or so cameras, use DENSE_SCHUR.

For larger bundle adjustment problems with sparse Schur Complement/Reduced camera matrices use SPARSE_SCHUR. This requires that you build Ceres with support for SuiteSparse, CXSparse or Eigen’s sparse linear algebra libraries.

If you do not have access to these libraries for whatever reason, ITERATIVE_SCHUR with SCHUR_JACOBI is an excellent alternative.

For large bundle adjustment problems (a few thousand cameras or more) use the ITERATIVE_SCHUR solver. There are a number of preconditioner choices here. SCHUR_JACOBI offers an excellent balance of speed and accuracy. This is also the recommended option if you are solving medium sized problems for which DENSE_SCHUR is too slow but SuiteSparse is not available.

Note

If you are solving small to medium sized problems, consider setting Solver::Options::use_explicit_schur_complement to true, it can result in a substantial performance boost.
If you are not satisfied with SCHUR_JACOBI‘s performance try CLUSTER_JACOBI and CLUSTER_TRIDIAGONAL in that order. They require that you have SuiteSparse installed. Both of these preconditioners use a clustering algorithm. Use SINGLE_LINKAGE before CANONICAL_VIEWS.
*/
ceres::Solver::Options getOptionsMedium(){
    // Set a few options
    ceres::Solver::Options options;
    cout<<"linear algebra: "<<options.sparse_linear_algebra_library_type<<endl;
    cout<<"linear solver: "<<options.linear_solver_type<<endl;
//    options.use_nonmonotonic_steps = true;
    options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
//    options.preconditioner_type = ceres::SPARSE_SCHUR;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    //If you are solving small to medium sized problems, consider setting Solver::Options::use_explicit_schur_complement to true, it can result in a substantial performance boost.
//    options.use_explicit_schur_complement=true;
    options.max_num_iterations = 50;

    return options;
}

void solve(ceres::Problem &problem, bool smallProblem=true){
    ceres::Solver::Summary summary;
    ceres::Solve(smallProblem ? getOptions() : getOptionsMedium(), &problem, &summary);
    if(!smallProblem) std::cout << "Final report:\n" << summary.FullReport();
}

void isoToAngleAxis(const Isometry3d& pose, double* cam){
//    Matrix<const double,3,3> rot(pose.linear());
//    cout<<"rotation : "<<pose.linear().data()<<endl;
//    auto begin = pose.linear().data();
    RotationMatrixToAngleAxis(ColumnMajorAdapter4x3(pose.linear().data()), cam);
    Vector3d t(pose.translation());
    cam[3]=t.x();
    cam[4]=t.y();
    cam[5]=t.z();
}

Isometry3d axisAngleToIso(const double* cam){
    Isometry3d poseFinal = Isometry3d::Identity();
    Matrix3d rot;
    ceres::AngleAxisToRotationMatrix(cam,rot.data());
    poseFinal.linear() = rot;
    poseFinal.translation() = Vector3d(cam[3],cam[4],cam[5]);
    return poseFinal;//.cast<float>();
}

Isometry3d eigenQuaternionToIso(const Eigen::Quaterniond& q, const Vector3d& t){
    Isometry3d poseFinal = Isometry3d::Identity();
    poseFinal.linear() = q.toRotationMatrix();
    poseFinal.translation() = t;
    return poseFinal;//.cast<float>();
}


Isometry3d pointToPoint_CeresAngleAxis(vector<Vector3d>&src,vector<Vector3d>&dst){

    double cam[6] = {0,0,0,0,0,0};

    ceres::Problem problem;

    for (int i = 0; i < src.size(); ++i) {
        // first viewpoint : dstcloud, fixed
        // second viewpoint: srcCloud, moves
        ceres::CostFunction* cost_function = ICPCostFunctions::PointToPointError_CeresAngleAxis::Create(dst[i],src[i]);
        problem.AddResidualBlock(cost_function, NULL, cam);
    }

    solve(problem);

    return axisAngleToIso(cam);
}

Isometry3d pointToPoint_EigenQuaternion(vector<Vector3d>&src,vector<Vector3d>&dst){
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d t(0,0,0);

    ceres::Problem problem;

    ceres::LocalParameterization *quaternion_parameterization = new eigen_quaternion::EigenQuaternionParameterization;

    for (int i = 0; i < src.size(); ++i) {
        // first viewpoint : dstcloud, fixed
        // second viewpoint: srcCloud, moves
        ceres::CostFunction* cost_function = ICPCostFunctions::PointToPointError_EigenQuaternion::Create(dst[i],src[i]);
        problem.AddResidualBlock(cost_function, NULL, q.coeffs().data(), t.data());
    }

    problem.SetParameterization(q.coeffs().data(),quaternion_parameterization);

    solve(problem);

    return eigenQuaternionToIso(q,t);
}

Isometry3d pointToPlane(vector<Vector3d> &src,vector<Vector3d> &dst,vector<Vector3d> &nor){

    double cam[6] = {0,0,0,0,0,0};

    ceres::Problem problem;

    for (int i = 0; i < src.size(); ++i) {
        // first viewpoint : dstcloud, fixed
        // second viewpoint: srcCloud, moves
        // nor is normal of dst
        ceres::CostFunction* cost_function = ICPCostFunctions::PointToPlaneError::Create(dst[i],src[i],nor[i]);
        problem.AddResidualBlock(cost_function, NULL, cam);
    }

    solve(problem);

    return axisAngleToIso(cam);
}

Isometry3d pointToPlane_EigenQuaternion(vector<Vector3d>&src,vector<Vector3d>&dst,vector<Vector3d> &nor){
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d t(0,0,0);

    ceres::Problem problem;

    ceres::LocalParameterization *quaternion_parameterization = new eigen_quaternion::EigenQuaternionParameterization;

    for (int i = 0; i < src.size(); ++i) {
        // first viewpoint : dstcloud, fixed
        // second viewpoint: srcCloud, moves
        ceres::CostFunction* cost_function = ICPCostFunctions::PointToPlaneError_EigenQuaternion::Create(dst[i],src[i],nor[i]);
        problem.AddResidualBlock(cost_function, NULL, q.coeffs().data(), t.data());
    }

    problem.SetParameterization(q.coeffs().data(),quaternion_parameterization);

    solve(problem);

    return eigenQuaternionToIso(q,t);
}

void ceresOptimizer(vector< std::shared_ptr<Frame> >& frames, bool pointToPlane, bool robust){

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

                ceres::LossFunction* loss = NULL;
                if(robust) loss = new ceres::SoftLOneLoss(dstEdge.weight);

//                cout<<dstEdge.weight<<endl;
//                problem.AddResidualBlock(cost_function,  srcQ.coeffs().data(),srcT.data(),dstQ.coeffs().data(), dstT.data());
                problem.AddResidualBlock(cost_function, loss, srcQ.coeffs().data(),srcT.data(),dstQ.coeffs().data(), dstT.data());

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

    solve(problem,false);

    //update camera poses
    for (int i = 0; i < frames.size(); ++i) {
//        poseFinal.linear() = Eigen::Map<Eigen::Quaterniond>(cameras+i*7).toRotationMatrix();
//        poseFinal.translation() = Eigen::Map<Eigen::Vector3d>(cameras+i*7+4);
        frames[i]->pose=eigenQuaternionToIso(qs[i],ts[i]);
    }
}

void ceresOptimizer_ceresAngleAxis(vector< std::shared_ptr<Frame> >& frames, bool pointToPlane,bool robust){

    ceres::Problem problem;

    double* cameras = new double[frames.size()*6];

    //extract initial camera poses
    for(int i=0; i<frames.size(); i++){
      isoToAngleAxis(frames[i]->pose,&cameras[i*6]);

//      cout<<"pose "<<i<<endl;
//      cout<<frames[i]->pose.matrix()<<endl;
//      cout<<"pose test "<<i<<endl;
//      cout<<axisAngleToIso(&cameras[i*6]).matrix()<<endl;

//      Visualize::spin();

      if (i==0){
          frames[i]->fixed=true;
      }
    }

    //add edges
    for(int src_id=0; src_id<frames.size(); src_id++){

        Frame& srcCloud = *frames[src_id];
        if(srcCloud.fixed) continue;

        for (int j = 0; j < srcCloud.neighbours.size(); ++j) {

            OutgoingEdge& dstEdge = srcCloud.neighbours[j];
            Frame& dstCloud = *frames.at(dstEdge.neighbourIdx);

            int dst_id=dstEdge.neighbourIdx;

            for(auto corr : dstEdge.correspondances){

                // first viewpoint : dstcloud, fixed
                // second viewpoint: srcCloud, moves

                ceres::CostFunction* cost_function;

                if(pointToPlane){
                    cost_function = ICPCostFunctions::PointToPlaneErrorGlobal_CeresAngleAxis::Create(dstCloud.pts[corr.second],srcCloud.pts[corr.first],dstCloud.nor[corr.second]);
                }else{
                    cost_function = ICPCostFunctions::PointToPointErrorGlobal_CeresAngleAxis::Create(dstCloud.pts[corr.second],srcCloud.pts[corr.first]);
                }

                ceres::LossFunction* loss = NULL;
                if(robust) loss = new ceres::SoftLOneLoss(dstEdge.weight);

                problem.AddResidualBlock(cost_function, loss, &cameras[src_id*6],&cameras[dst_id*6]);

            }
        }
    }

    for (int i = 0; i < frames.size(); ++i) {
        if(frames[i]->fixed){
            std::cout<<i<<" fixed"<<endl;
            problem.SetParameterBlockConstant(&cameras[i*6]);
        }
    }

    solve(problem,false);

    //update camera poses
    for (int i = 0; i < frames.size(); ++i) {
        frames[i]->pose=axisAngleToIso(&cameras[i*6]);
    }
}


} //end namespace


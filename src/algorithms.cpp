#include "algorithms.h"

#include <eigen3/Eigen/Dense>
#include <math.h>
#include <unordered_map>
#include <vector>

namespace ICP {

//K called correlation matrix in eggert_comparison_mva97, is actually a covariance matrix multiplied by N
//http://graphics.stanford.edu/~smr/ICP/comparison/eggert_comparison_mva97.pdf
Isometry3d pointToPoint(vector<Vector3d>&src,vector<Vector3d>&dst){
    int N = src.size(); assert(N==dst.size());
    Map<Matrix3Xd> ps(&src[0].x(),3,N); //maps vector<Vector3d>
    Map<Matrix3Xd> qs(&dst[0].x(),3,N); //to Matrix3Nf columnwise
    Vector3d p_dash = ps.rowwise().mean();
    Vector3d q_dash = qs.rowwise().mean();
    Matrix3Xd ps_centered = ps.colwise() - p_dash;
    Matrix3Xd qs_centered = qs.colwise() - q_dash;
    Matrix3d K = qs_centered * ps_centered.transpose();
    JacobiSVD<Matrix3d> svd(K, ComputeFullU | ComputeFullV);
    Matrix3d R = svd.matrixU()*svd.matrixV().transpose();
    if(R.determinant()<0){
        R.col(2) *= -1;
    }
    Isometry3d T = Isometry3d::Identity();
    T.linear() = R;
    T.translation() = q_dash - R*p_dash; return T;
}

// https://www.comp.nus.edu.sg/~lowkl/publications/lowk_point-to-plane_icp_techrep.pdf
// http://www.cs.princeton.edu/~smr/papers/icpstability.pdf
Isometry3d pointToPlane(vector<Vector3d> &src,vector<Vector3d> &dst,vector<Vector3d> &nor){
    assert(src.size()==dst.size() && src.size()==nor.size());
    Matrix<double,6,6> C; C.setZero();
    Matrix<double,6,1> d; d.setZero();

    for(uint i=0;i<src.size();++i){
        Vector3d cro = src[i].cross(nor[i]);
        C.block<3,3>(0,0) += cro*cro.transpose();
        C.block<3,3>(0,3) += nor[i]*cro.transpose();
        C.block<3,3>(3,3) += nor[i]*nor[i].transpose();
        double sum = (src[i]-dst[i]).dot(nor[i]);
        d.head(3) -= cro*sum;
        d.tail(3) -= nor[i]*sum;
    }
    C.block<3,3>(3,0) = C.block<3,3>(0,3);

    Matrix<double,6,1> x = C.ldlt().solve(d);
    Isometry3d T = Isometry3d::Identity();
    T.linear() = (   AngleAxisd(x(0), Vector3d::UnitX())
                   * AngleAxisd(x(1), Vector3d::UnitY())
                   * AngleAxisd(x(2), Vector3d::UnitZ())
                 ).toRotationMatrix();
    T.translation() = x.block(3,0,3,1);
    return T;
}

}



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
namespace ICPG2O {
using namespace g2o;

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

}//end namespace



#include <ceres/local_parameterization.h>
#include <ceres/autodiff_local_parameterization.h>
#include <ceres/types.h>
#include <ceres/rotation.h>
#include <ceres/ceres.h>

namespace ICPCeres {

ceres::Solver::Options getOptions(){
    // Set a few options
    ceres::Solver::Options options;
    //options.use_nonmonotonic_steps = true;
    options.preconditioner_type = ceres::IDENTITY;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 10;

    return options;
}

void solve(ceres::Problem &problem){
    ceres::Solver::Summary summary;
    ceres::Solve(getOptions(), &problem, &summary);
    //std::cout << "Final report:\n" << summary.FullReport();
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


} //end namespace


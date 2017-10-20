#include "icp-closedform.h"

namespace ICP_Closedform {
using namespace Eigen;
using namespace std;

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

    for(int i=0;i<src.size();++i){
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

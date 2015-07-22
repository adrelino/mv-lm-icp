#ifndef ICPCERES
#define ICPCERES

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

#include "frame.h"

#include <ceres/local_parameterization.h>
#include <ceres/autodiff_local_parameterization.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/types.h>
#include <ceres/rotation.h>

#include "eigen_quaternion.h"

using namespace Eigen;
using namespace std;

namespace ICP_Ceres {

template <typename T>
ceres::MatrixAdapter<T, 1, 4> ColumnMajorAdapter4x3(T* pointer) {
  return ceres::MatrixAdapter<T, 1, 4>(pointer);
}

    //pairwise
    Isometry3d pointToPlane(vector<Vector3d> &src,vector<Vector3d> &dst,vector<Vector3d> &nor);
    Isometry3d pointToPlane_EigenQuaternion(vector<Vector3d> &src,vector<Vector3d> &dst,vector<Vector3d> &nor);

    Isometry3d pointToPoint_EigenQuaternion(vector<Vector3d> &src,vector<Vector3d> &dst);
    Isometry3d pointToPoint_CeresAngleAxis(vector<Vector3d> &src,vector<Vector3d> &dst);

    //multiview
    void ceresOptimizer(std::vector< std::shared_ptr<Frame> >& frames, bool pointToPlane, bool robust);
    void ceresOptimizer_ceresAngleAxis(vector< std::shared_ptr<Frame> >& frames, bool pointToPlane, bool robust);

}

namespace ICPCostFunctions{

struct PointToPointErrorGlobal{

    const Eigen::Vector3d p_dst;
    const Eigen::Vector3d p_src;

    PointToPointErrorGlobal(const Eigen::Vector3d &dst, const Eigen::Vector3d &src) :
        p_dst(dst), p_src(src)
    {

    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d& dst, const Eigen::Vector3d& src) {
        return (new ceres::AutoDiffCostFunction<PointToPointErrorGlobal, 3, 4, 3, 4, 3>(new PointToPointErrorGlobal(dst, src)));
    }

    template <typename T>
    bool operator()(const T* const camera_rotation, const T* const camera_translation, const T* const camera_rotation_dst, const T* const camera_translation_dst, T* residuals) const {

        // Make sure the Eigen::Vector world point is using the ceres::Jet type as it's Scalar type
        Eigen::Matrix<T,3,1> src; src << T(p_src[0]), T(p_src[1]), T(p_src[2]);
        Eigen::Matrix<T,3,1> dst; dst << T(p_dst[0]), T(p_dst[1]), T(p_dst[2]);

        // Map the T* array to an Eigen Quaternion object (with appropriate Scalar type)
        Eigen::Quaternion<T> q = Eigen::Map<const Eigen::Quaternion<T> >(camera_rotation);
        Eigen::Quaternion<T> qDst = Eigen::Map<const Eigen::Quaternion<T> >(camera_rotation_dst);

        // Map T* to Eigen Vector3 with correct Scalar type
        Eigen::Matrix<T,3,1> t = Eigen::Map<const Eigen::Matrix<T,3,1> >(camera_translation);
        Eigen::Matrix<T,3,1> tDst = Eigen::Map<const Eigen::Matrix<T,3,1> >(camera_translation_dst);

        // Rotate the point using Eigen rotations
        Eigen::Matrix<T,3,1> p = q * src;
        p += t;
        Eigen::Matrix<T,3,1> p2 = qDst * dst;
        p2 += tDst;

        // The error is the difference between the predicted and observed position.
        residuals[0] = p[0] - p2[0];
        residuals[1] = p[1] - p2[1];
        residuals[2] = p[2] - p2[2];

        return true;
    }
};

struct PointToPlaneErrorGlobal{

    const Eigen::Vector3d& p_dst;
    const Eigen::Vector3d& p_src;
    const Eigen::Vector3d& p_nor;


    PointToPlaneErrorGlobal(const Eigen::Vector3d& dst, const Eigen::Vector3d& src, const Eigen::Vector3d& nor) :
    p_dst(dst), p_src(src), p_nor(nor)
    {
//        cout<<nor.dot(nor)<<endl;
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d& dst, const Eigen::Vector3d& src, const Eigen::Vector3d& nor) {
        return (new ceres::AutoDiffCostFunction<PointToPlaneErrorGlobal, 1, 4, 3, 4, 3>(new PointToPlaneErrorGlobal(dst, src, nor)));
    }

    template <typename T>
    bool operator()(const T* const camera_rotation, const T* const camera_translation, const T* const camera_rotation_dst, const T* const camera_translation_dst, T* residuals) const {

        // Make sure the Eigen::Vector world point is using the ceres::Jet type as it's Scalar type
        Eigen::Matrix<T,3,1> src; src << T(p_src[0]), T(p_src[1]), T(p_src[2]);
        Eigen::Matrix<T,3,1> dst; dst << T(p_dst[0]), T(p_dst[1]), T(p_dst[2]);
        Eigen::Matrix<T,3,1> nor; nor << T(p_nor[0]), T(p_nor[1]), T(p_nor[2]);

        // Map the T* array to an Eigen Quaternion object (with appropriate Scalar type)
        Eigen::Quaternion<T> q = Eigen::Map<const Eigen::Quaternion<T> >(camera_rotation);
        Eigen::Quaternion<T> qDst = Eigen::Map<const Eigen::Quaternion<T> >(camera_rotation_dst);

        // Map T* to Eigen Vector3 with correct Scalar type
        Eigen::Matrix<T,3,1> t = Eigen::Map<const Eigen::Matrix<T,3,1> >(camera_translation);
        Eigen::Matrix<T,3,1> tDst = Eigen::Map<const Eigen::Matrix<T,3,1> >(camera_translation_dst);

        // Rotate the point using Eigen rotations
        Eigen::Matrix<T,3,1> p = q * src;
        p += t;
        Eigen::Matrix<T,3,1> p2 = qDst.toRotationMatrix() * dst;
        p2 += tDst;
        Eigen::Matrix<T,3,1> n2 = qDst.toRotationMatrix() * nor; //no translation on normal

        // The error is the difference between the predicted and observed position projected onto normal
        residuals[0] = (p - p2).dot(n2);

        return true;
    }
};

struct PointToPointErrorGlobal_CeresAngleAxis{

    const Eigen::Vector3d& p_dst;
    const Eigen::Vector3d& p_src;

    PointToPointErrorGlobal_CeresAngleAxis(const Eigen::Vector3d &dst, const Eigen::Vector3d &src) :
        p_dst(dst), p_src(src)
    {
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d &observed, const Eigen::Vector3d &worldPoint) {
        return (new ceres::AutoDiffCostFunction<PointToPointErrorGlobal_CeresAngleAxis, 3, 6, 6>(new PointToPointErrorGlobal_CeresAngleAxis(observed, worldPoint)));
    }

    template <typename T>
    bool operator()(const T* const camera, const T* const camera2, T* residuals) const {

        T p[3] = {T(p_src[0]), T(p_src[1]), T(p_src[2])};
        ceres::AngleAxisRotatePoint(camera,p,p);

        T p2[3] = {T(p_dst[0]), T(p_dst[1]), T(p_dst[2])};
        ceres::AngleAxisRotatePoint(camera2,p2,p2);

        // camera[3,4,5] are the translation.
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        p2[0] += camera2[3];
        p2[1] += camera2[4];
        p2[2] += camera2[5];

        // The error is the difference between the predicted and observed position.
        residuals[0] = p[0] - p2[0];
        residuals[1] = p[1] - p2[1];
        residuals[2] = p[2] - p2[2];

        return true;
    }
};

struct PointToPlaneErrorGlobal_CeresAngleAxis{

    const Eigen::Vector3d& p_dst;
    const Eigen::Vector3d& p_src;
    const Eigen::Vector3d& p_nor;


    PointToPlaneErrorGlobal_CeresAngleAxis(const Eigen::Vector3d& dst, const Eigen::Vector3d& src, const Eigen::Vector3d& nor) :
    p_dst(dst), p_src(src), p_nor(nor)
    {
//        cout<<nor.dot(nor)<<endl;
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d& dst, const Eigen::Vector3d& src, const Eigen::Vector3d& nor) {
        return (new ceres::AutoDiffCostFunction<PointToPlaneErrorGlobal_CeresAngleAxis, 1, 6, 6>(new PointToPlaneErrorGlobal_CeresAngleAxis(dst, src, nor)));
    }

    template <typename T>
    bool operator()(const T* const cam1, const T* const cam2, T* residuals) const {

        T p1[3] = {T(p_src[0]), T(p_src[1]), T(p_src[2])};
        ceres::AngleAxisRotatePoint(cam1,p1,p1);

        T p2[3] = {T(p_dst[0]), T(p_dst[1]), T(p_dst[2])};
        ceres::AngleAxisRotatePoint(cam2,p2,p2);

        T nor[3] = {T(p_nor[0]), T(p_nor[1]), T(p_nor[2])};
        ceres::AngleAxisRotatePoint(cam2,nor,nor);

        // camera[3,4,5] are the translation.
        p1[0] += cam1[3];
        p1[1] += cam1[4];
        p1[2] += cam1[5];

        p2[0] += cam2[3];
        p2[1] += cam2[4];
        p2[2] += cam2[5];

        //no translation on normal

        // The error is the difference between the predicted and observed position projected onto normal
        residuals[0] = (p1[0] - p2[0]) * nor[0] + \
                       (p1[1] - p2[1]) * nor[1] + \
                       (p1[2] - p2[2]) * nor[2];


        return true;
    }
};

struct PointToPointError_EigenQuaternion{
    const Eigen::Vector3d& p_dst;
    const Eigen::Vector3d& p_src;

    PointToPointError_EigenQuaternion(const Eigen::Vector3d &dst, const Eigen::Vector3d &src) :
        p_dst(dst), p_src(src)
    {
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d &observed, const Eigen::Vector3d &worldPoint) {
        return (new ceres::AutoDiffCostFunction<PointToPointError_EigenQuaternion, 3, 4, 3>(new PointToPointError_EigenQuaternion(observed, worldPoint)));
    }

    template <typename T>
    bool operator()(const T* const camera_rotation, const T* const camera_translation, T* residuals) const {

        //ceres::AngleAxisRotatePoint

        // Make sure the Eigen::Vector world point is using the ceres::Jet type as it's Scalar type
        Eigen::Matrix<T,3,1> point; point << T(p_src[0]), T(p_src[1]), T(p_src[2]);

        // Map the T* array to an Eigen Quaternion object (with appropriate Scalar type)
        Eigen::Quaternion<T> q = Eigen::Map<const Eigen::Quaternion<T> >(camera_rotation);

        // Rotate the point using Eigen rotations
        Eigen::Matrix<T,3,1> p = q * point;

        // Map T* to Eigen Vector3 with correct Scalar type
        Eigen::Matrix<T,3,1> t = Eigen::Map<const Eigen::Matrix<T,3,1> >(camera_translation);
        p += t;

        // The error is the difference between the predicted and observed position.
        residuals[0] = p[0] - T(p_dst[0]);
        residuals[1] = p[1] - T(p_dst[1]);
        residuals[2] = p[2] - T(p_dst[2]);

        return true;
    }
};




struct PointToPointError_CeresAngleAxis{

    const Eigen::Vector3d& p_dst;
    const Eigen::Vector3d& p_src;

    PointToPointError_CeresAngleAxis(const Eigen::Vector3d &dst, const Eigen::Vector3d &src) :
        p_dst(dst), p_src(src)
    {
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d &observed, const Eigen::Vector3d &worldPoint) {
        return (new ceres::AutoDiffCostFunction<PointToPointError_CeresAngleAxis, 3, 6>(new PointToPointError_CeresAngleAxis(observed, worldPoint)));
    }

    template <typename T>
    bool operator()(const T* const camera, T* residuals) const {

//            Eigen::Matrix<T,3,1> point;
//            point << T(p_src[0]), T(p_src[1]), T(p_src[2]);

        T p[3] = {T(p_src[0]), T(p_src[1]), T(p_src[2])};
        ceres::AngleAxisRotatePoint(camera,p,p);

        // camera[3,4,5] are the translation.
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        // The error is the difference between the predicted and observed position.
        residuals[0] = p[0] - T(p_dst[0]);
        residuals[1] = p[1] - T(p_dst[1]);
        residuals[2] = p[2] - T(p_dst[2]);

        return true;
    }
};

struct PointToPlaneError{
    const Eigen::Vector3d& p_dst;
    const Eigen::Vector3d& p_src;
    const Eigen::Vector3d& p_nor;


    PointToPlaneError(const Eigen::Vector3d& dst, const Eigen::Vector3d& src, const Eigen::Vector3d& nor) :
    p_dst(dst), p_src(src), p_nor(nor)
    {
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d& observed, const Eigen::Vector3d& worldPoint, const Eigen::Vector3d& normal) {
        return (new ceres::AutoDiffCostFunction<PointToPlaneError, 1, 6>(new PointToPlaneError(observed, worldPoint,normal)));
    }

    template <typename T>
    bool operator()(const T* const camera, T* residuals) const {

        T p[3] = {T(p_src[0]), T(p_src[1]), T(p_src[2])};
        ceres::AngleAxisRotatePoint(camera,p,p);

        // camera[3,4,5] are the translation.
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        // The error is the difference between the predicted and observed position.
        residuals[0] = (p[0] - T(p_dst[0])) * T(p_nor[0]) + \
                       (p[1] - T(p_dst[1])) * T(p_nor[1]) + \
                       (p[2] - T(p_dst[2])) * T(p_nor[2]);

        return true;
    }
};

struct PointToPlaneError_EigenQuaternion{
    const Eigen::Vector3d& p_dst;
    const Eigen::Vector3d& p_src;
    const Eigen::Vector3d& p_nor;


    PointToPlaneError_EigenQuaternion(const Eigen::Vector3d& dst, const Eigen::Vector3d& src, const Eigen::Vector3d& nor) :
    p_dst(dst), p_src(src), p_nor(nor)
    {
    }


    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d& observed, const Eigen::Vector3d& worldPoint, const Eigen::Vector3d& normal) {
        return (new ceres::AutoDiffCostFunction<PointToPlaneError_EigenQuaternion, 1, 4, 3>(new PointToPlaneError_EigenQuaternion(observed, worldPoint,normal)));
    }

    template <typename T>
    bool operator()(const T* const camera_rotation, const T* const camera_translation, T* residuals) const {

        // Make sure the Eigen::Vector world point is using the ceres::Jet type as it's Scalar type
        Eigen::Matrix<T,3,1> point; point << T((double) p_src[0]), T((double) p_src[1]), T((double) p_src[2]);

        // Map the T* array to an Eigen Quaternion object (with appropriate Scalar type)
        Eigen::Quaternion<T> q = Eigen::Map<const Eigen::Quaternion<T> >(camera_rotation);

        // Rotate the point using Eigen rotations
        Eigen::Matrix<T,3,1> p = q * point;

        // Map T* to Eigen Vector3 with correct Scalar type
        Eigen::Matrix<T,3,1> t = Eigen::Map<const Eigen::Matrix<T,3,1> >(camera_translation);
        p += t;

        Eigen::Matrix<T,3,1> point2; point2 << T((double) p_dst[0]), T((double) p_dst[1]), T((double) p_dst[2]);
        Eigen::Matrix<T,3,1> normal; normal << T((double) p_nor[0]), T((double) p_nor[1]), T((double) p_nor[2]);

        // The error is the difference between the predicted and observed position projected onto normal
        residuals[0] = (p - point2).dot(normal);

        return true;
    }
};

}
#endif // ICPCERES

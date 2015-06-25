//
//  ceres_extensions.h
//  Bundle_Adjust_Test
//
//  Created by Lloyd Hughes on 2014/04/11.
//  Copyright (c) 2014 Lloyd Hughes. All rights reserved.
//  hughes.lloyd@gmail.com
//

#ifndef Bundle_Adjust_Test_ceres_extensions_h
#define Bundle_Adjust_Test_ceres_extensions_h

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace eigen_quaternion {

////from autodiff_local_parameterization.h
//   struct EigenQuaternionPlus {
//     template<typename T>
//     bool operator()(const T* x, const T* delta, T* x_plus_delta) const {
//       const T squared_norm_delta =
//           delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2];

//       T q_delta[4];
//       if (squared_norm_delta > T(0.0)) {
//         T norm_delta = sqrt(squared_norm_delta);
//         const T sin_delta_by_delta = sin(norm_delta) / norm_delta;
//         q_delta[0] = cos(norm_delta);
//         q_delta[1] = sin_delta_by_delta * delta[0];
//         q_delta[2] = sin_delta_by_delta * delta[1];
//         q_delta[3] = sin_delta_by_delta * delta[2];
//       } else {
//         // We do not just use q_delta = [1,0,0,0] here because that is a
//         // constant and when used for automatic differentiation will
//         // lead to a zero derivative. Instead we take a first order
//         // approximation and evaluate it at zero.
//         q_delta[0] = T(1.0);
//         q_delta[1] = delta[0];
//         q_delta[2] = delta[1];
//         q_delta[3] = delta[2];
//       }

//       QuaternionProduct(q_delta, x, x_plus_delta);
//       return true;
//     }
//   };
//   //LocalParameterization* local_parameterization = new AutoDiffLocalParameterization<QuaternionPlus, 4, 3>;

    // Plus(x, delta) = [cos(|delta|), sin(|delta|) delta / |delta|] * x
    // with * being the quaternion multiplication operator. Here we assume
    // that the first element of the quaternion vector is the real (cos
    // theta) part.
    class EigenQuaternionParameterization : public ceres::LocalParameterization {
    public:
        virtual ~EigenQuaternionParameterization() {}
//from local_parameterization.cc

//        bool QuaternionParameterization::Plus(const double* x,
//                                              const double* delta,
//                                              double* x_plus_delta) const {
//          const double norm_delta =
//              sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
//          if (norm_delta > 0.0) {
//            const double sin_delta_by_delta = (sin(norm_delta) / norm_delta);
//            double q_delta[4];
//            q_delta[0] = cos(norm_delta);
//            q_delta[1] = sin_delta_by_delta * delta[0];
//            q_delta[2] = sin_delta_by_delta * delta[1];
//            q_delta[3] = sin_delta_by_delta * delta[2];
//            QuaternionProduct(q_delta, x, x_plus_delta);
//          } else {
//            for (int i = 0; i < 4; ++i) {
//              x_plus_delta[i] = x[i];
//            }
//          }
//          return true;
//        }

//        bool QuaternionParameterization::ComputeJacobian(const double* x,
//                                                         double* jacobian) const {
//          jacobian[0] = -x[1]; jacobian[1]  = -x[2]; jacobian[2]  = -x[3];  // NOLINT
//          jacobian[3] =  x[0]; jacobian[4]  =  x[3]; jacobian[5]  = -x[2];  // NOLINT
//          jacobian[6] = -x[3]; jacobian[7]  =  x[0]; jacobian[8]  =  x[1];  // NOLINT
//          jacobian[9] =  x[2]; jacobian[10] = -x[1]; jacobian[11] =  x[0];  // NOLINT
//          return true;
//        }
        
        virtual bool Plus(const double* x_raw, const double* delta_raw, double* x_plus_delta_raw) const {
            const Eigen::Map<const Eigen::Quaterniond> x(x_raw);
            const Eigen::Map<const Eigen::Vector3d > delta(delta_raw);
            
            Eigen::Map<Eigen::Quaterniond> x_plus_delta(x_plus_delta_raw);
            
            const double delta_norm = delta.norm();
            if ( delta_norm > 0.0 ){
                const double sin_delta_by_delta = sin(delta_norm) / delta_norm;
                Eigen::Quaterniond tmp( cos(delta_norm), sin_delta_by_delta*delta[0], sin_delta_by_delta*delta[1], sin_delta_by_delta*delta[2] );
                
                x_plus_delta = tmp*x;
            }
            else {
                x_plus_delta = x;
            }
            return true;
        }
        
        virtual bool ComputeJacobian(const double* x, double* jacobian) const {
            jacobian[0] =  x[3]; jacobian[1]  =  x[2]; jacobian[2]   = -x[1];  // NOLINT x
            jacobian[3] = -x[2]; jacobian[4]  =  x[3]; jacobian[5]   =  x[0];  // NOLINT y
            jacobian[6] =  x[1]; jacobian[7]  = -x[0]; jacobian[8]   =  x[3];  // NOLINT z
            jacobian[9] = -x[0]; jacobian[10] = -x[1]; jacobian[11]  = -x[2];  // NOLINT w
                return true;
        }
        
        virtual int GlobalSize() const { return 4; }
        virtual int LocalSize() const { return 3; }
        
    };
    
    template <typename T> inline
    void EigenQuaternionToScaledRotation(const T q[4], T R[3 * 3]) {
        EigenQuaternionToScaledRotation(q, RowMajorAdapter3x3(R));
    }
    
    template <typename T, int row_stride, int col_stride> inline
    void EigenQuaternionToScaledRotation(const T q[4],
                                    const ceres::MatrixAdapter<T, row_stride, col_stride>& R) {
        // Make convenient names for elements of q.
        T a = q[3];
        T b = q[0];
        T c = q[1];
        T d = q[2];
        // This is not to eliminate common sub-expression, but to
        // make the lines shorter so that they fit in 80 columns!
        T aa = a * a;
        T ab = a * b;
        T ac = a * c;
        T ad = a * d;
        T bb = b * b;
        T bc = b * c;
        T bd = b * d;
        T cc = c * c;
        T cd = c * d;
        T dd = d * d;
        
        R(0, 0) = aa + bb - cc - dd; R(0, 1) = T(2) * (bc - ad);  R(0, 2) = T(2) * (ac + bd);  // NOLINT
        R(1, 0) = T(2) * (ad + bc);  R(1, 1) = aa - bb + cc - dd; R(1, 2) = T(2) * (cd - ab);  // NOLINT
        R(2, 0) = T(2) * (bd - ac);  R(2, 1) = T(2) * (ab + cd);  R(2, 2) = aa - bb - cc + dd; // NOLINT
    }
    
    template <typename T> inline
    void EigenQuaternionToRotation(const T q[4], T R[3 * 3]) {
        EigenQuaternionToRotation(q, RowMajorAdapter3x3(R));
    }
    
    template <typename T, int row_stride, int col_stride> inline
    void EigenQuaternionToRotation(const T q[4],
                              const ceres::MatrixAdapter<T, row_stride, col_stride>& R) {
        EigenQuaternionToScaledRotation(q, R);
        
        T normalizer = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
        CHECK_NE(normalizer, T(0));
        normalizer = T(1) / normalizer;
        
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R(i, j) *= normalizer;
            }
        }
    }
    
    template <typename T> inline
    void EigenUnitQuaternionRotatePoint(const T q[4], const T pt[3], T result[3]) {
        const T t2 =  q[3] * q[0];
        const T t3 =  q[3] * q[1];
        const T t4 =  q[3] * q[2];
        const T t5 = -q[0] * q[0];
        const T t6 =  q[0] * q[1];
        const T t7 =  q[0] * q[2];
        const T t8 = -q[1] * q[1];
        const T t9 =  q[1] * q[2];
        const T t1 = -q[2] * q[2];
        result[0] = T(2) * ((t8 + t1) * pt[0] + (t6 - t4) * pt[1] + (t3 + t7) * pt[2]) + pt[0];  // NOLINT
        result[1] = T(2) * ((t4 + t6) * pt[0] + (t5 + t1) * pt[1] + (t9 - t2) * pt[2]) + pt[1];  // NOLINT
        result[2] = T(2) * ((t7 - t3) * pt[0] + (t2 + t9) * pt[1] + (t5 + t8) * pt[2]) + pt[2];  // NOLINT
    }
    
    template <typename T> inline
    void EigenQuaternionRotatePoint(const T q[4], const T pt[3], T result[3]) {
        // 'scale' is 1 / norm(q).
        const T scale = T(1) / sqrt(q[0] * q[0] +
                                    q[1] * q[1] +
                                    q[2] * q[2] +
                                    q[3] * q[3]);
        
        // Make unit-norm version of q.
        const T unit[4] = {
            scale * q[0],
            scale * q[1],
            scale * q[2],
            scale * q[3],
        };
        
        EigenUnitQuaternionRotatePoint(unit, pt, result);
    }
    
    template<typename T> inline
    void EigenQuaternionProduct(const T z[4], const T w[4], T zw[4]) {
        zw[0] =   z[0] * w[3] + z[1] * w[2] - z[2] * w[1] + z[3] * w[0];
        zw[1] = - z[0] * w[2] + z[1] * w[3] + z[2] * w[0] + z[3] * w[1];
        zw[2] =   z[0] * w[1] - z[1] * w[0] + z[2] * w[3] + z[3] * w[2];
        zw[3] = - z[0] * w[0] - z[1] * w[1] - z[2] * w[2] + z[3] * w[3];
    }
}

#endif

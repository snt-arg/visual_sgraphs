#ifndef G2O_ISOMETRY3D_MAPPINGS_H_
#define G2O_ISOMETRY3D_MAPPINGS_H_

#include <Eigen/Core>

#include "se3quat.h"
#include "../core/eigen_types.h"
#include "g2o_types_slam3d_api.h"

namespace g2o
{
    namespace internal
    {
        template <typename Derived>
        bool writeVector(std::ostream &os, const Eigen::DenseBase<Derived> &b)
        {
            for (int i = 0; i < b.size(); i++)
                os << b(i) << " ";
            return os.good();
        }

        template <typename Derived>
        bool readVector(std::istream &is, Eigen::DenseBase<Derived> &b)
        {
            for (int i = 0; i < b.size() && is.good(); i++)
                is >> b(i);
            return is.good() || is.eof();
        }

        /**
         * Extract the rotation matrix from an Isometry3 matrix
         */
        inline Isometry3D::ConstLinearPart extractRotation(const Isometry3D &A)
        {
            return A.matrix().topLeftCorner<3, 3>();
        }

        /**
         * Normalize the quaternion, such that ||q|| == 1 and q.w() > 0
         */
        G2O_TYPES_SLAM3D_API Quaternion normalized(const Quaternion &q);

        /**
         * Normalize the quaternion, such that ||q|| == 1 and q.w() > 0 (in-place)
         */
        G2O_TYPES_SLAM3D_API Quaternion &normalize(Quaternion &q);

        /**
         * Converts a Rotation Matrix to (qx qy, qz)
         */
        G2O_TYPES_SLAM3D_API Vector3D toCompactQuaternion(const Matrix3D &R);

        /**
         * Converts an Isometry3D to (x, y, z, qx, qy, qz)
         */
        G2O_TYPES_SLAM3D_API Vector6D toVectorMQT(const Isometry3D &t);

        /**
         * Converts an Isometry3D to (x, y, z, qx, qy, qz, qw)
         */
        G2O_TYPES_SLAM3D_API Vector7D toVectorQT(const Isometry3D &t);

        /**
         * Converts a (x, y, z, qx, qy, qz, qw) to an Isometry3D
         */
        G2O_TYPES_SLAM3D_API Isometry3D fromVectorQT(const Vector7D &v);

    } // end namespace internal
} // end namespace g2o

#endif

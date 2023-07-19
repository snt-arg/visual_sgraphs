#include "../stuff/misc.h"
#include "isometry3d_mappings.h"

namespace g2o
{
    namespace internal
    {
        Quaternion normalized(const Quaternion &q)
        {
            Quaternion q2(q);
            normalize(q2);
            return q2;
        }

        Quaternion &normalize(Quaternion &q)
        {
            q.normalize();
            if (q.w() < 0)
            {
                q.coeffs() *= -1;
            }
            return q;
        }

        Vector3D toCompactQuaternion(const Matrix3D &R)
        {
            Quaternion q(R);
            normalize(q);
            return q.coeffs().head<3>();
        }

        Vector6D toVectorMQT(const Isometry3D &t)
        {
            Vector6D vec;
            vec.block<3, 1>(3, 0) = toCompactQuaternion(extractRotation(t));
            vec.block<3, 1>(0, 0) = t.translation();
            return vec;
        }

        Vector7D toVectorQT(const Isometry3D &t)
        {
            Quaternion q(extractRotation(t));
            q.normalize();
            Vector7D v;
            v[3] = q.x();
            v[4] = q.y();
            v[5] = q.z();
            v[6] = q.w();
            v.block<3, 1>(0, 0) = t.translation();
            return v;
        }

        Isometry3D fromVectorQT(const Vector7D &v)
        {
            Isometry3D t;
            t = Quaternion(v[6], v[3], v[4], v[5]).toRotationMatrix();
            t.translation() = v.head<3>();
            return t;
        }

    } // end namespace internal
} // end namespace g2o

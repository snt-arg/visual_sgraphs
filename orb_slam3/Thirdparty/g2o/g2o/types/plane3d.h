#ifndef G2O_PLANE3D_H_
#define G2O_PLANE3D_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "../stuff/misc.h"
#include "../core/eigen_types.h"
#include "g2o_types_slam3d_addons_api.h"

namespace g2o
{
    class G2O_TYPES_SLAM3D_ADDONS_API Plane3D
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        Plane3D() { fromVector(Vector4D(1., 0., 0., -1.)); }

        Plane3D(const Vector4D &v) { fromVector(v); }

        inline Vector4D toVector() const { return _coeffs; }

        inline void fromVector(const Vector4D &coeffs_)
        {
            _coeffs = coeffs_;
            normalize(_coeffs);
        }

    protected:
        Vector4D _coeffs;

        static inline void normalize(Vector4D &coeffs)
        {
            double n = coeffs.head<3>().norm();
            coeffs = coeffs * (1. / n);
        }
    };

} // namespace g2o

#endif

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

        // friend Plane3D operator*(const Isometry3D &t, const Plane3D &plane);

        Plane3D() { fromVector(Vector4D(1., 0., 0., -1.)); }

        Plane3D(const Vector4D &v) { fromVector(v); }

        inline Vector4D toVector() const { return _coeffs; }

        // inline const Vector4D &coeffs() const { return _coeffs; }

        inline void fromVector(const Vector4D &coeffs_)
        {
            _coeffs = coeffs_;
            normalize(_coeffs);
        }

        // static double azimuth(const Vector3D &v) { return std::atan2(v(1), v(0)); }

        // static double elevation(const Vector3D &v)
        // {
        //     return std::atan2(v(2), v.head<2>().norm());
        // }

        // double distance() const { return -_coeffs(3); }

        // Vector3D normal() const { return _coeffs.head<3>(); }

        // static Matrix3D rotation(const Vector3D &v)
        // {
        //     double _azimuth = azimuth(v);
        //     double _elevation = elevation(v);
        //     return (AngleAxis(_azimuth, Vector3D::UnitZ()) *
        //             AngleAxis(-_elevation, Vector3D::UnitY()))
        //         .toRotationMatrix();
        // }

        // inline void oplus(const Vector3D &v)
        // {
        //     // construct a normal from azimuth and elevation;
        //     double _azimuth = v[0];
        //     double _elevation = v[1];
        //     double s = std::sin(_elevation), c = std::cos(_elevation);
        //     Vector3D n(c * std::cos(_azimuth), c * std::sin(_azimuth), s);

        //     // rotate the normal
        //     Matrix3D R = rotation(normal());
        //     double d = distance() + v[2];
        //     _coeffs.head<3>() = R * n;
        //     _coeffs(3) = -d;
        //     normalize(_coeffs);
        // }

        // inline Vector3D ominus(const Plane3D &plane)
        // {
        //     // construct the rotation that would bring the plane normal in (1 0 0)
        //     Matrix3D R = rotation(normal()).transpose();
        //     Vector3D n = R * plane.normal();
        //     double d = distance() - plane.distance();
        //     return Vector3D(azimuth(n), elevation(n), d);
        // }

    protected:
        Vector4D _coeffs;

        static inline void normalize(Vector4D &coeffs)
        {
            double n = coeffs.head<3>().norm();
            coeffs = coeffs * (1. / n);
        }
    };

    // inline Plane3D operator*(const Isometry3D &t, const Plane3D &plane)
    // {
    //     Vector4D v = plane._coeffs;
    //     Vector4D v2;
    //     Matrix3D R = t.rotation();
    //     v2.head<3>() = R * v.head<3>();
    //     v2(3) = v(3) - t.translation().dot(v2.head<3>());
    //     return Plane3D(v2);
    // };

} // namespace g2o

#endif

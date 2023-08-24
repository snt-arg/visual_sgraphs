/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ORB_SLAM3_OPTIMIZABLETYPES_H
#define ORB_SLAM3_OPTIMIZABLETYPES_H

#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/types/isometry3d_mappings.h"

#include <Eigen/Geometry>
#include <Thirdparty/g2o/g2o/types/sim3.h>
#include <include/CameraModels/GeometricCamera.h>
#include <Thirdparty/g2o/g2o/types/vertex_plane.h>
#include <Thirdparty/g2o/g2o/types/types_six_dof_expmap.h>

namespace ORB_SLAM3
{
    class EdgeSE3ProjectXYZOnlyPose : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeSE3ProjectXYZOnlyPose() {}

        bool read(std::istream &is);

        bool write(std::ostream &os) const;

        void computeError()
        {
            const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
            Eigen::Vector2d obs(_measurement);
            _error = obs - pCamera->project(v1->estimate().map(Xw));
        }

        bool isDepthPositive()
        {
            const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
            return (v1->estimate().map(Xw))(2) > 0.0;
        }

        virtual void linearizeOplus();

        Eigen::Vector3d Xw;
        GeometricCamera *pCamera;
    };

    class EdgeSE3ProjectXYZOnlyPoseToBody : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeSE3ProjectXYZOnlyPoseToBody() {}

        bool read(std::istream &is);

        bool write(std::ostream &os) const;

        void computeError()
        {
            const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
            Eigen::Vector2d obs(_measurement);
            _error = obs - pCamera->project((mTrl * v1->estimate()).map(Xw));
        }

        bool isDepthPositive()
        {
            const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
            return ((mTrl * v1->estimate()).map(Xw))(2) > 0.0;
        }

        virtual void linearizeOplus();

        Eigen::Vector3d Xw;
        GeometricCamera *pCamera;

        g2o::SE3Quat mTrl;
    };

    class EdgeSE3ProjectXYZ : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeSE3ProjectXYZ();

        bool read(std::istream &is);

        bool write(std::ostream &os) const;

        void computeError()
        {
            const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
            const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
            Eigen::Vector2d obs(_measurement);
            _error = obs - pCamera->project(v1->estimate().map(v2->estimate()));
        }

        bool isDepthPositive()
        {
            const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
            const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
            return ((v1->estimate().map(v2->estimate()))(2) > 0.0);
        }

        virtual void linearizeOplus();

        GeometricCamera *pCamera;
    };

    class EdgeSE3ProjectXYZToBody : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeSE3ProjectXYZToBody();

        bool read(std::istream &is);

        bool write(std::ostream &os) const;

        void computeError()
        {
            const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
            const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
            Eigen::Vector2d obs(_measurement);
            _error = obs - pCamera->project((mTrl * v1->estimate()).map(v2->estimate()));
        }

        bool isDepthPositive()
        {
            const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
            const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
            return ((mTrl * v1->estimate()).map(v2->estimate()))(2) > 0.0;
        }

        virtual void linearizeOplus();

        GeometricCamera *pCamera;
        g2o::SE3Quat mTrl;
    };

    class VertexSim3Expmap : public g2o::BaseVertex<7, g2o::Sim3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        VertexSim3Expmap();
        virtual bool read(std::istream &is);
        virtual bool write(std::ostream &os) const;

        virtual void setToOriginImpl()
        {
            _estimate = g2o::Sim3();
        }

        virtual void oplusImpl(const double *update_)
        {
            Eigen::Map<g2o::Vector7d> update(const_cast<double *>(update_));

            if (_fix_scale)
                update[6] = 0;

            g2o::Sim3 s(update);
            setEstimate(s * estimate());
        }

        GeometricCamera *pCamera1, *pCamera2;

        bool _fix_scale;
    };

    class EdgeSim3ProjectXYZ : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, ORB_SLAM3::VertexSim3Expmap>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EdgeSim3ProjectXYZ();
        virtual bool read(std::istream &is);
        virtual bool write(std::ostream &os) const;

        void computeError()
        {
            const ORB_SLAM3::VertexSim3Expmap *v1 = static_cast<const ORB_SLAM3::VertexSim3Expmap *>(_vertices[1]);
            const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);

            Eigen::Vector2d obs(_measurement);
            _error = obs - v1->pCamera1->project(v1->estimate().map(v2->estimate()));
        }

        // virtual void linearizeOplus();
    };

    class EdgeInverseSim3ProjectXYZ : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, VertexSim3Expmap>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EdgeInverseSim3ProjectXYZ();
        virtual bool read(std::istream &is);
        virtual bool write(std::ostream &os) const;

        void computeError()
        {
            const ORB_SLAM3::VertexSim3Expmap *v1 = static_cast<const ORB_SLAM3::VertexSim3Expmap *>(_vertices[1]);
            const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);

            Eigen::Vector2d obs(_measurement);
            _error = obs - v1->pCamera2->project((v1->estimate().inverse().map(v2->estimate())));
        }

        // virtual void linearizeOplus();
    };

    /**
     * 🚀 Below classes have been added to ORB-SLAM 3.0 to augment fiducial markers and semantic data.
     */

    /**
     * The edge used to connect a Marker vertex (SE3) to a KeyFrame vertex (SE3)
     * [Note]: it creates constraint for six measurements, i.e., (x, y, z, roll, pitch, yaw)
     */
    class EdgeSE3ProjectSE3 : public g2o::BaseBinaryEdge<6, g2o::Isometry3D, g2o::VertexSE3Expmap, g2o::VertexSE3Expmap>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EdgeSE3ProjectSE3();
        virtual bool read(std::istream &is);
        virtual bool write(std::ostream &os) const;
        virtual void setMeasurement(const g2o::Isometry3D &m) override { _measurement = m; }

        void computeError()
        {
            // Marker's global pose
            const g2o::VertexSE3Expmap *vMarkerGP = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
            // KeyFrame's global pose
            const g2o::VertexSE3Expmap *vKeyFrameGP = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);

            // Calculate the local pose of the marker w.r.t. the keyframe
            g2o::SE3Quat markerLP = vKeyFrameGP->estimate() * vMarkerGP->estimate();

            g2o::Isometry3D markerLPIso = g2o::Isometry3D::Identity();
            markerLPIso.matrix() = markerLP.to_homogeneous_matrix();
            // Calculating the transformation between the measuremenent and the marker's local pose
            g2o::Isometry3D delta = _measurement.inverse() * markerLPIso;

            // Calculating the final error
            _error = g2o::internal::toVectorMQT(delta);
        }
    };

    /**
     * The edge used to connect a Room vertex (SE3) to a Door vertex (SE3)
     * [Note]: it creates constraint for six measurements, i.e., (x, y, z, roll, pitch, yaw)
     */
    class EdgeSE3DoorProjectSE3Room : public EdgeSE3ProjectSE3
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EdgeSE3DoorProjectSE3Room();

        void computeError()
        {
            // Room's global pose
            const g2o::VertexSE3Expmap *vRoomGP = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
            // Door's global pose
            const g2o::VertexSE3Expmap *vDoorGP = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);

            // Calculate the local pose of the door w.r.t. the keyframe
            g2o::SE3Quat doorLP = vRoomGP->estimate().inverse() * vDoorGP->estimate();

            g2o::Isometry3D doorLPIso = g2o::Isometry3D::Identity();
            doorLPIso.matrix() = doorLP.to_homogeneous_matrix();
            // Calculating the transformation between the measuremenent and the door's local pose
            g2o::Isometry3D delta = _measurement.inverse() * doorLPIso;

            // Calculating the final error
            _error = g2o::internal::toVectorMQT(delta);
        }
    };

    /**
     * The edge used to connect a Wall vertex (VertexPlane) to a Marker vertex (SE3)
     * [Note]: it creates constraint for four measurements, i.e., (x, y, z, d)
     */
    class EdgeVertexPlaneProjectSE3 : public g2o::BaseBinaryEdge<4, Eigen::Vector4d, g2o::VertexSE3Expmap, g2o::VertexPlane>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeVertexPlaneProjectSE3();
        virtual bool read(std::istream &is);
        virtual bool write(std::ostream &os) const;

        void computeError()
        {
            // Marker's global pose
            const g2o::VertexSE3Expmap *vMarkerGP = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
            // Wall's global pose
            const g2o::VertexPlane *vWallGP = static_cast<const g2o::VertexPlane *>(_vertices[1]);

            // Calculating marker's pose
            g2o::Isometry3D markerPose = vMarkerGP->estimate();

            // Calculate the wall's coefficients (in global frame)
            g2o::Vector4D wallCoeffs = vWallGP->estimate().coeffs();

            // Normalize the wall vector if necessary
            if (wallCoeffs(3) < 0)
            {
                wallCoeffs *= -1;
            }

            // Create the wall plane in global frame
            g2o::Plane3D wall_g(wallCoeffs);

            // Calculate the wall in marker's frame
            g2o::Plane3D wall_m = markerPose.inverse() * wall_g;

            // Calculate the normal of the marker in marker's frame
            g2o::Plane3D markerNormal_m = markerPose.inverse() * markerPose.matrix().col(2);

            // Calculate the difference of the planes
            Eigen::Vector3d planeDiff = wall_m.coeffs().head(3) - markerNormal_m.coeffs().head(3);

            _error[0] = planeDiff(0);
            _error[1] = planeDiff(1);
            _error[2] = planeDiff(2);
            _error[3] = wall_m.coeffs()(3); // Distance (d) of the cacmera
        }
    };
}

#endif // ORB_SLAM3_OPTIMIZABLETYPES_H

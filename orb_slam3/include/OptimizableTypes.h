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
#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
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
    };

    /**
     * 🚀 [vS-Graphs] Edges for Adding Geometric and Semantic Constraints
     */

    /**
     * The edge used to connect a Marker vertex (SE3) to a KeyFrame vertex (SE3)
     * 🚧 [vS-Graphs v.2.0] This edge is not used anymore, in contrast to the previous version.
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
     * The edge used to connect a Plane vertex (VertexPlane) to a KeyFrame vertex (SE3)
     * [Note]: it creates constraint for three measurements, i.e., (x, y, z)
     */
    class EdgeVertexPlaneProjectSE3KF : public g2o::BaseBinaryEdge<3, g2o::Plane3D, g2o::VertexSE3Expmap, g2o::VertexPlane>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeVertexPlaneProjectSE3KF();
        virtual bool read(std::istream &is);
        virtual bool write(std::ostream &os) const;

        void setMeasurement(const g2o::Plane3D &m) override { _measurement = m; }

        void computeError()
        {
            // KeyFrame's global pose
            const g2o::VertexSE3Expmap *vKeyFrameGP = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
            // Plane's global pose
            const g2o::VertexPlane *vPlaneGP = static_cast<const g2o::VertexPlane *>(_vertices[1]);

            // Calculating poses (in global frame)
            Eigen::Isometry3d kfPose = vKeyFrameGP->estimate();
            g2o::Plane3D localPlane = kfPose * vPlaneGP->estimate();

            // Calculating the error
            _error = localPlane.ominus(_measurement);
        }
    };

    /**
     * The edge used to connect a Plane vertex (VertexPlane) to a Marker vertex (SE3)
     * [Note]: it creates constraint for four measurements, i.e., (x, y, z, d)
     */
    class EdgeVertexPlaneProjectSE3M : public g2o::BaseBinaryEdge<4, Eigen::Vector4d, g2o::VertexSE3Expmap, g2o::VertexPlane>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeVertexPlaneProjectSE3M();
        virtual bool read(std::istream &is);
        virtual bool write(std::ostream &os) const;

        void computeError()
        {
            // Marker's global pose
            const g2o::VertexSE3Expmap *vMarkerGP = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
            // Plane's global pose
            const g2o::VertexPlane *vPlaneGP = static_cast<const g2o::VertexPlane *>(_vertices[1]);

            // Calculating poses (in global frame)
            g2o::Isometry3D markerPose = vMarkerGP->estimate();
            g2o::Vector4D planeCoeffs = vPlaneGP->estimate().coeffs();

            // Normalize the plane vector if necessary
            if (planeCoeffs(3) < 0)
                planeCoeffs *= -1;

            // Create the plane plane in global frame
            g2o::Plane3D plane_g(planeCoeffs);

            // Calculate the plane in marker's frame
            g2o::Plane3D plane_m = markerPose.inverse() * plane_g;

            // Calculate the normal of the marker in marker's frame
            g2o::Plane3D markerNormal_m = markerPose.inverse() * markerPose.matrix().col(2);

            // Calculate the difference of the planes
            Eigen::Vector3d planeDiff = plane_m.coeffs().head(3) - markerNormal_m.coeffs().head(3);

            _error[0] = planeDiff(0);
            _error[1] = planeDiff(1);
            _error[2] = planeDiff(2);
            _error[3] = plane_m.coeffs()(3); // Distance (d) of the cacmera
        }
    };

    /**
     * The edge used to connect a Two-wall Room's center (SE3) to Wall vertices (VertexPlane)
     * [Note]: it creates constraint for three measurements, i.e., (x, y, z)
     */
    class EdgeVertex2PlaneProjectSE3Room : public g2o::BaseMultiEdge<3, Eigen::Vector3d>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3d markerPosition;

        EdgeVertex2PlaneProjectSE3Room();
        EdgeVertex2PlaneProjectSE3Room(Eigen::Vector3d position);

        virtual bool read(std::istream &is);
        virtual bool write(std::ostream &os) const;

        void computeError() override
        {
            const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
            const g2o::VertexPlane *v2 = static_cast<const g2o::VertexPlane *>(_vertices[1]);
            const g2o::VertexPlane *v3 = static_cast<const g2o::VertexPlane *>(_vertices[2]);

            Eigen::Vector3d roomPose = v1->estimate().translation();
            Eigen::Vector4d wall1 = v2->estimate().coeffs();
            Eigen::Vector4d wall2 = v3->estimate().coeffs();

            correctPlaneDirection(wall1);
            correctPlaneDirection(wall2);

            Eigen::Vector3d vec;
            if (fabs(wall1(3)) > fabs(wall2(3)))
            {
                vec = (0.5 * (fabs(wall1(3)) * wall1.head(3) - fabs(wall2(3)) * wall2.head(3))) +
                      fabs(wall2(3)) * wall2.head(3);
            }
            else
            {
                vec = (0.5 * (fabs(wall2(3)) * wall2.head(3) - fabs(wall1(3)) * wall1.head(3))) +
                      fabs(wall1(3)) * wall1.head(3);
            }

            Eigen::Vector3d normal = vec / vec.norm();
            Eigen::Vector3d finalPose = vec + (markerPosition - (markerPosition.dot(normal)) * normal);

            _error = roomPose - finalPose;
        }

    protected:
        virtual void correctPlaneDirection(Eigen::Vector4d &plane)
        {
            if (plane(3) > 0)
                plane *= -1;
        }
    };

    /**
     * The edge used to connect a Four-wall Room's center (SE3) to Wall vertices (VertexPlane)
     * [Note]: it creates constraint for three measurements, i.e., (x, y, z)
     */
    class EdgeVertex4PlaneProjectSE3Room : public g2o::BaseMultiEdge<3, Eigen::Vector3d>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeVertex4PlaneProjectSE3Room();
        virtual bool read(std::istream &is);
        virtual bool write(std::ostream &os) const;

        void computeError() override
        {
            const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
            const g2o::VertexPlane *v2 = static_cast<const g2o::VertexPlane *>(_vertices[1]);
            const g2o::VertexPlane *v3 = static_cast<const g2o::VertexPlane *>(_vertices[2]);
            const g2o::VertexPlane *v4 = static_cast<const g2o::VertexPlane *>(_vertices[3]);
            const g2o::VertexPlane *v5 = static_cast<const g2o::VertexPlane *>(_vertices[4]);

            Eigen::Vector3d roomPose = v1->estimate().translation();
            Eigen::Vector4d xPlane1 = v2->estimate().coeffs();
            Eigen::Vector4d xPlane2 = v3->estimate().coeffs();
            Eigen::Vector4d yPlane1 = v4->estimate().coeffs();
            Eigen::Vector4d yPlane2 = v5->estimate().coeffs();

            correctPlaneDirection(xPlane1);
            correctPlaneDirection(xPlane2);
            correctPlaneDirection(yPlane1);
            correctPlaneDirection(yPlane2);

            Eigen::Vector3d vecX, vecY;
            if (fabs(xPlane1(3)) > fabs(xPlane2(3)))
                vecX = (0.5 * (fabs(xPlane1(3)) * xPlane1.head(3) - fabs(xPlane2(3)) * xPlane2.head(3))) + fabs(xPlane2(3)) * xPlane2.head(3);
            else
                vecX = (0.5 * (fabs(xPlane2(3)) * xPlane2.head(3) - fabs(xPlane1(3)) * xPlane1.head(3))) + fabs(xPlane1(3)) * xPlane1.head(3);

            if (fabs(yPlane1(3)) > fabs(yPlane2(3)))
                vecY = (0.5 * (fabs(yPlane1(3)) * yPlane1.head(3) - fabs(yPlane2(3)) * yPlane2.head(3))) + fabs(yPlane2(3)) * yPlane2.head(3);
            else
                vecY = (0.5 * (fabs(yPlane2(3)) * yPlane2.head(3) - fabs(yPlane1(3)) * yPlane1.head(3))) + fabs(yPlane1(3)) * yPlane1.head(3);

            Eigen::Vector3d finalPose = vecX + vecY;
            _error = roomPose - finalPose;
        }

    protected:
        virtual void correctPlaneDirection(Eigen::Vector4d &plane)
        {
            if (plane(3) > 0)
                plane *= -1;
        }
    };

    /**
     * The edge used to connect a Room's center (SE3) to a Marker vertex (SE3)
     * [Note]: it creates constraint for four measurements, i.e., (x, y, z, d)
     */
    class EdgeVertexSE3RoomProjectSE3Marker : public g2o::BaseBinaryEdge<4, Eigen::Vector4d, g2o::VertexSE3Expmap, g2o::VertexSE3Expmap>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeVertexSE3RoomProjectSE3Marker();
        virtual bool read(std::istream &is);
        virtual bool write(std::ostream &os) const;

        void computeError()
        {
            // Marker's global pose
            const g2o::VertexSE3Expmap *vMarkerGP = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
            // Room's center point in global pose
            const g2o::VertexSE3Expmap *vRoomCenterGP = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);

            // Calculating poses (in global frame)
            g2o::Isometry3D markerPose = vMarkerGP->estimate();
            Eigen::Vector3d roomPose = vRoomCenterGP->estimate().translation();

            // Calculating the error
            _error[0] = markerPose.translation()(0) - roomPose(0);
            _error[1] = markerPose.translation()(1) - roomPose(1);
            _error[2] = markerPose.translation()(2) - roomPose(2);
            _error[3] = markerPose.translation().norm();
        }
    };
}

#endif

/**
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <https://www.gnu.org/licenses/>.
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
    /**
     * The edge used to connect a MapPoint vertex (SBAPointXYZ) to a Camera vertex (SE3)
     * [Note]: it creates constraint for two measurements, i.e., (u, v)
     */
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

    /**
     * The edge used to connect a MapPoint vertex (SBAPointXYZ) to a Camera vertex (SE3)
     * [Note]: it creates constraint for two measurements, i.e., (u, v)
     */
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

    /**
     * The edge used to connect a MapPoint vertex (XYZ) to a KeyFrame vertex (SE3)
     * [Note]: it creates constraint for two measurements, i.e., (u, v)
     */
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

    /**
     * The edge used to connect a MapPoint vertex (XYZ) to a KeyFrame vertex (SE3)
     * [Note]: it creates constraint for two measurements, i.e., (u, v)
     */
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

    /**
     * The edge used to connect a MapPoint vertex (SBAPointXYZ) to a Camera vertex (Sim3)
     * [Note]: it creates constraint for seven measurements, i.e., (u, v, z, roll, pitch, yaw, scale)
     */
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
     * The edge used to connect a Marker vertex (SE3) to a KeyFrame vertex (SE3)
     * [Note]: it creates constraint for six measurements, i.e., (x, y, z, roll, pitch, yaw)
     * 🚧 [vS-Graphs v1.5] Deprecated, but extended for other optimization constraints.
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
     * The edge used to connect a Room vertex (SE3) to a Passage vertex (SE3)
     * [Note]: it creates constraint for six measurements, i.e., (x, y, z, roll, pitch, yaw)
     */
    class EdgeSE3DoorwayProjectSE3Room : public EdgeSE3ProjectSE3
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EdgeSE3DoorwayProjectSE3Room();

        void computeError()
        {
            // Room's global pose
            const g2o::VertexSE3Expmap *vRoomGP = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
            // Passage's global pose
            const g2o::VertexSE3Expmap *vDoorwayGP = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);

            // Calculate the local pose of the doorway w.r.t. the keyframe
            g2o::SE3Quat doorwayLP = vRoomGP->estimate().inverse() * vDoorwayGP->estimate();

            g2o::Isometry3D doorwayLPIso = g2o::Isometry3D::Identity();
            doorwayLPIso.matrix() = doorwayLP.to_homogeneous_matrix();
            // Calculating the transformation between the measuremenent and the doorway's local pose
            g2o::Isometry3D delta = _measurement.inverse() * doorwayLPIso;

            // Calculating the final error
            _error = g2o::internal::toVectorMQT(delta);
        }
    };

    /**
     * The edge used to connect a Plane vertex (VertexPlane) to a KeyFrame point (SE3)
     * [Note]: it creates constraint connecting the points in a plane observation to the plane
     */
    class EdgeSE3KFPointToPlane : public g2o::BaseBinaryEdge<1, Eigen::Matrix4d, g2o::VertexSE3Expmap, g2o::VertexPlane>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeSE3KFPointToPlane();
        virtual bool read(std::istream &is);
        virtual bool write(std::ostream &os) const;

        void setMeasurement(const Eigen::Matrix4d &m) override { _measurement = m; }

        void computeError()
        {
            const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
            const g2o::VertexPlane *v2 = static_cast<const g2o::VertexPlane *>(_vertices[1]);

            Eigen::Matrix4d Ti = v1->estimate().inverse().to_homogeneous_matrix();
            Eigen::Vector4d Pj = v2->estimate().coeffs();
            Eigen::Matrix4d Gij = _measurement;
            _error = Pj.transpose() * Ti * Gij * Ti.transpose() * Pj;
        }

        // Checks if the plane distance d is in the correct direction
        bool isDistanceCorrect()
        {
            const g2o::VertexSE3Expmap *vKeyFrameGP = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
            const g2o::VertexPlane *vPlaneGP = static_cast<const g2o::VertexPlane *>(_vertices[1]);

            // local plane equation
            Eigen::Isometry3d kfPose = vKeyFrameGP->estimate();
            g2o::Plane3D localPlane = kfPose * vPlaneGP->estimate();

            return (localPlane.coeffs()(3) > 0);
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

        // Checks if the plane distance d is in the correct direction
        bool isDistanceCorrect()
        {
            const g2o::VertexSE3Expmap *vKeyFrameGP = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
            const g2o::VertexPlane *vPlaneGP = static_cast<const g2o::VertexPlane *>(_vertices[1]);

            // local plane equation
            Eigen::Isometry3d kfPose = vKeyFrameGP->estimate();
            g2o::Plane3D localPlane = kfPose * vPlaneGP->estimate();

            return (localPlane.coeffs()(3) > 0);
        }
    };

    /**
     * The edge used to connect a MapPoint vertex (SBAPointXYZ) to a Plane vertex (VertexPlane)
     */
    class EdgeVertexPlaneProjectPointXYZ : public g2o::BaseBinaryEdge<1, double, g2o::VertexSBAPointXYZ, g2o::VertexPlane>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeVertexPlaneProjectPointXYZ();
        virtual bool read(std::istream &is);
        virtual bool write(std::ostream &os) const;

        void computeError()
        {
            // Map Point's position
            const g2o::VertexSBAPointXYZ *vPoint = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
            // Plane's global pose
            const g2o::VertexPlane *vPlaneGP = static_cast<const g2o::VertexPlane *>(_vertices[1]);

            // Calculating the error
            // plane equation is already normalized -> D = n.x + d
            _error[0] = vPlaneGP->estimate().coeffs().head(3).dot(vPoint->estimate().head(3)) + vPlaneGP->estimate().coeffs()(3);
        }
    };

    /**
     * The edge used to connect a Plane vertex (VertexPlane) to a Marker vertex (SE3)
     * [Note]: it creates constraint for four measurements, i.e., (x, y, z, d)
     * 🚧 [vS-Graphs v1.5] Unused.
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
     * The edge used to enforce parallelism between two Plane vertices (VertexPlane)
     * [Note]: it creates constraint for one measurement, i.e., (angle difference)
     */
    class EdgeVertexPlaneParallelism : public g2o::BaseBinaryEdge<1, double, g2o::VertexPlane, g2o::VertexPlane>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeVertexPlaneParallelism();
        virtual bool read(std::istream &is);
        virtual bool write(std::ostream &os) const;

        void computeError() override
        {
            // Planes
            const g2o::VertexPlane *vPlane1 = static_cast<const g2o::VertexPlane *>(_vertices[0]);
            const g2o::VertexPlane *vPlane2 = static_cast<const g2o::VertexPlane *>(_vertices[1]);

            Eigen::Vector3d n1 = vPlane1->estimate().normal().normalized();
            Eigen::Vector3d n2 = vPlane2->estimate().normal().normalized();

            // Compute deviation from parallelism
            double err = std::fabs(n1.dot(n2));
            _error[0] = 1 - err;
        }
    };

    /**
     * The edge used to enforce perpendicularity between two Plane vertices (VertexPlane)
     * [Note]: it creates constraint for one measurement, i.e., (angle difference)
     */
    class EdgeVertexPlanePerpendicularity : public g2o::BaseBinaryEdge<1, double, g2o::VertexPlane, g2o::VertexPlane>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeVertexPlanePerpendicularity();
        virtual bool read(std::istream &is);
        virtual bool write(std::ostream &os) const;

        void computeError() override
        {
            // Planes
            const g2o::VertexPlane *vPlane1 = static_cast<const g2o::VertexPlane *>(_vertices[0]);
            const g2o::VertexPlane *vPlane2 = static_cast<const g2o::VertexPlane *>(_vertices[1]);

            Eigen::Vector3d n1 = vPlane1->estimate().normal().normalized();
            Eigen::Vector3d n2 = vPlane2->estimate().normal().normalized();

            // Compute deviation from perpendicularity
            double err = std::fabs(n1.dot(n2));
            _error[0] = err;
        }
    };

    /**
     * The edge used to connect a Two-wall Room's center (SE3) to Wall vertices (VertexPlane)
     * [Note]: it creates constraint for three measurements, i.e., (x, y, z)
     * 🚧 [vS-Graphs v1.5] Deprecated with the introduction of n-wall rooms.
     */
    class EdgeVertex2PlaneProjectSE3Room : public g2o::BaseMultiEdge<3, Eigen::Vector3d>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
            // Eigen::Vector3d finalPose = vec + (markerPosition - (markerPosition.dot(normal)) * normal);

            _error = roomPose - vec;
        }

    protected:
        virtual void correctPlaneDirection(Eigen::Vector4d &plane)
        {
            if (plane(3) > 0)
                plane *= -1;
        }
    };

    /**
     * The edge used to connect a Four-wall Room's centroid (SE3) to Wall vertices (VertexPlane)
     * [Note]: it creates constraint for three measurements, i.e., (x, y, z)
     * 🚧 [vS-Graphs v1.5] Deprecated with the introduction of n-wall rooms.
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
     * The edge used to connect an N-wall Room's centroid (SE3) to its Wall vertices (VertexPlane)
     * [Note]: it creates constraint for three measurements, i.e., (x, y, z)
     */
    class EdgeVertexNPlaneProjectSE3Room : public g2o::BaseMultiEdge<3, Eigen::Vector3d>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        virtual bool read(std::istream &is);
        virtual bool write(std::ostream &os) const;
        EdgeVertexNPlaneProjectSE3Room()
        {
            // Dynamically sized edge: at least one SE3 (room center) + N planes
            resize(1);
        }

        void computeError() override
        {
            // First vertex is always the room pose (SE3)
            const g2o::VertexSE3Expmap *vRoom = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
            Eigen::Vector3d roomPose = vRoom->estimate().translation();

            // Remaining vertices are walls
            std::vector<Eigen::Vector4d> walls;
            for (size_t i = 1; i < _vertices.size(); ++i)
            {
                const g2o::VertexPlane *vWall = static_cast<const g2o::VertexPlane *>(_vertices[i]);
                Eigen::Vector4d plane = vWall->estimate().coeffs();
                correctPlaneDirection(plane);
                walls.push_back(plane);
            }

            // Compute representative position from all walls enclosing the cluster
            Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
            for (const auto &wall : walls)
            {
                // Each wall equation ax+by+cz+d=0 → normal = (a,b,c), offset = d
                Eigen::Vector3d normal = wall.head<3>();
                double d = wall(3);

                // Approximate contribution: project along the normal
                Eigen::Vector3d contrib = -d * normal;
                centroid += contrib;
            }

            if (!walls.empty())
                centroid /= static_cast<double>(walls.size());

            // Final error
            _error = roomPose - centroid;
        }

    protected:
        void correctPlaneDirection(Eigen::Vector4d &plane)
        {
            if (plane(3) > 0)
                plane *= -1;
        }
    };

    /**
     * The edge used to connect a Floor centroid (SE3) to its Room vertices (SE3)
     * [Note]: it creates constraint for three measurements, i.e., (x, y, z)
     */
    class EdgeVertexNSE3RoomProjectSE3Floor : public g2o::BaseMultiEdge<3, Eigen::Vector3d>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        virtual bool read(std::istream &is);
        virtual bool write(std::ostream &os) const;
        EdgeVertexNSE3RoomProjectSE3Floor()
        {
            // Dynamically sized edge: at least one SE3 (floor center) + N rooms
            resize(1);
        }

        void computeError() override
        {
            // First vertex is always the floor pose (SE3)
            const g2o::VertexSE3Expmap *vFloor = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
            Eigen::Vector3d floorPose = vFloor->estimate().translation();

            // Remaining vertices are rooms
            std::vector<Eigen::Vector3d> rooms;
            for (size_t i = 1; i < _vertices.size(); ++i)
            {
                const g2o::VertexSE3Expmap *vRoom = static_cast<const g2o::VertexSE3Expmap *>(_vertices[i]);
                Eigen::Vector3d room = vRoom->estimate().translation();
                rooms.push_back(room);
            }

            // Compute representative position from all rooms in the floor
            Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
            for (const auto &room : rooms)
                centroid += room;

            if (!rooms.empty())
                centroid /= static_cast<double>(rooms.size());

            // Final error
            _error = floorPose - centroid;
        }
    };

    /**
     * The edge used to connect a Room's centroid (SE3) to a Marker vertex (SE3)
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

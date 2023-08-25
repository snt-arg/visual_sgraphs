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
     * The vertex used for connecting the center of a Room (SE3)
     */
    class VertexVec3Expmap : public g2o::BaseVertex<3, g2o::Vector3D>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        VertexVec3Expmap();
        virtual bool read(std::istream &is){};
        virtual bool write(std::ostream &os) const {};

        virtual void setToOriginImpl() { _estimate.fill(0.); }

        virtual bool getEstimateData(double *est) const;
        virtual bool setEstimateDataImpl(const double *est);
        virtual bool getMinimalEstimateData(double *est) const;

        virtual int estimateDimension() const { return 3; }
        virtual int minimalEstimateDimension() const { return 3; }

        virtual bool setMinimalEstimateDataImpl(const double *est)
        {
            _estimate = Eigen::Map<const g2o::Vector3D>(est);
            return true;
        }

        virtual void oplusImpl(const double *update)
        {
            for (int i = 0; i < 3; ++i)
                _estimate[i] += update[i];
        }
    };

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

    /**
     * The edge used to connect a two-wall Room vertex (VertexPlane) to a Marker vertex (SE3)
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
            const VertexVec3Expmap *v1 = static_cast<const VertexVec3Expmap *>(_vertices[0]);
            const g2o::VertexPlane *v2 = static_cast<const g2o::VertexPlane *>(_vertices[1]);
            const g2o::VertexPlane *v3 = static_cast<const g2o::VertexPlane *>(_vertices[2]);

            Eigen::Vector3d roomPose = v1->estimate();
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
     * The edge used to connect a four-wall Room vertex (VertexPlane) to a Marker vertex (SE3)
     * [Note]: it creates constraint for three measurements, i.e., (x, y, z)
     */
    class EdgeVertex4PlaneProjectSE3Room : public EdgeVertex2PlaneProjectSE3Room
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeVertex4PlaneProjectSE3Room();

        void computeError() override
        {
            const VertexVec3Expmap *v1 = static_cast<const VertexVec3Expmap *>(_vertices[0]);
            const g2o::VertexPlane *v2 = static_cast<const g2o::VertexPlane *>(_vertices[1]);
            const g2o::VertexPlane *v3 = static_cast<const g2o::VertexPlane *>(_vertices[2]);
            const g2o::VertexPlane *v4 = static_cast<const g2o::VertexPlane *>(_vertices[3]);
            const g2o::VertexPlane *v5 = static_cast<const g2o::VertexPlane *>(_vertices[4]);

            Eigen::Vector3d roomPose = v1->estimate();
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
    };
}

#endif // ORB_SLAM3_OPTIMIZABLETYPES_H

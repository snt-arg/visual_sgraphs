// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.

#ifndef G2O_EIGEN_TYPES_H
#define G2O_EIGEN_TYPES_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace g2o
{
  // Vector (int)
  typedef Eigen::Matrix<int, 2, 1, Eigen::ColMajor> Vector2I;
  typedef Eigen::Matrix<int, 3, 1, Eigen::ColMajor> Vector3I;
  typedef Eigen::Matrix<int, 4, 1, Eigen::ColMajor> Vector4I;
  typedef Eigen::Matrix<int, Eigen::Dynamic, 1, Eigen::ColMajor> VectorXI;

  // Vector (float)
  typedef Eigen::Matrix<float, 2, 1, Eigen::ColMajor> Vector2F;
  typedef Eigen::Matrix<float, 3, 1, Eigen::ColMajor> Vector3F;
  typedef Eigen::Matrix<float, 4, 1, Eigen::ColMajor> Vector4F;
  typedef Eigen::Matrix<float, Eigen::Dynamic, 1, Eigen::ColMajor> VectorXF;

  // Vector (double)
  typedef Eigen::Matrix<double, 2, 1, Eigen::ColMajor> Vector2D;
  typedef Eigen::Matrix<double, 3, 1, Eigen::ColMajor> Vector3D;
  typedef Eigen::Matrix<double, 4, 1, Eigen::ColMajor> Vector4D;
  typedef Eigen::Matrix<double, 6, 1, Eigen::ColMajor> Vector6D;
  typedef Eigen::Matrix<double, 7, 1, Eigen::ColMajor> Vector7D;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor> VectorXD;

  // Matrix (int)
  typedef Eigen::Matrix<int, 2, 2, Eigen::ColMajor> Matrix2I;
  typedef Eigen::Matrix<int, 3, 3, Eigen::ColMajor> Matrix3I;
  typedef Eigen::Matrix<int, 4, 4, Eigen::ColMajor> Matrix4I;
  typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> MatrixXI;

  // Matrix (float)
  typedef Eigen::Matrix<float, 2, 2, Eigen::ColMajor> Matrix2F;
  typedef Eigen::Matrix<float, 3, 3, Eigen::ColMajor> Matrix3F;
  typedef Eigen::Matrix<float, 4, 4, Eigen::ColMajor> Matrix4F;
  typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> MatrixXF;

  // Matrix (double)
  typedef Eigen::Matrix<double, 2, 2, Eigen::ColMajor> Matrix2D;
  typedef Eigen::Matrix<double, 3, 3, Eigen::ColMajor> Matrix3D;
  typedef Eigen::Matrix<double, 4, 4, Eigen::ColMajor> Matrix4D;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> MatrixXD;

  // Isometry (double)
  typedef Eigen::Transform<double, 2, Eigen::Isometry, Eigen::ColMajor> Isometry2D;
  typedef Eigen::Transform<double, 3, Eigen::Isometry, Eigen::ColMajor> Isometry3D;

  // Affine (double)
  typedef Eigen::Transform<double, 2, Eigen::Affine, Eigen::ColMajor> Affine2D;
  typedef Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> Affine3D;

  typedef Eigen::AngleAxis<double> AngleAxis;
  typedef Eigen::Quaternion<double> Quaternion;
  typedef Eigen::Rotation2D<double> Rotation2D;

} // end namespace g2o

#endif

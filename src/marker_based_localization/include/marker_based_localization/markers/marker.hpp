#ifndef MARKER_HPP
#define MARKER_HPP

#include <Eigen/Geometry>
#include "Eigen/src/Geometry/RotationBase.h"
#include <ros/ros.h>

using Eigen::Isometry3d;
using Eigen::Quaterniond;
using Eigen::Translation3d;
using Eigen::Vector3d;

// Detected fiducial marker
class Marker
{
public:
  unsigned int id;
  Isometry3d transform;
  Vector3d error;  // Error, preferably variance in meters
  Marker() = default;
  Marker(unsigned int id, Translation3d position) : id(id), transform(position)
  {
  }
  Marker(unsigned int id, Translation3d position, Vector3d error) : id(id), transform(position), error(error)
  {
  }
  Marker(unsigned int id, Translation3d position, Quaterniond orientation, Translation3d offset)
    : id(id), transform(position * orientation * offset)
  {
    this->transform.linear() = Eigen::Matrix3d::Identity();
  }
  Marker(unsigned int id, Translation3d position, Quaterniond orientation, Translation3d offset, Vector3d error)
    : Marker(id, position, orientation, offset)
  {
    this->error = error;
  }
  Marker(unsigned int id, Isometry3d transform) : id(id)
  {
    this->transform = Eigen::Translation3d(transform.translation()) * Eigen::Quaterniond::Identity();
  }
  Marker(unsigned int id, Isometry3d transform, Vector3d error) : Marker(id, transform)
  {
    this->error = error;
  }
  Marker(unsigned int id, Isometry3d transform, Translation3d offset)
    : Marker(id, Eigen::Translation3d(transform.translation()), Eigen::Quaterniond(transform.rotation()), offset)
  {
  }
  Marker(unsigned int id, Isometry3d transform, Translation3d offset, Vector3d error) : Marker(id, transform, offset)
  {
    this->error = error;
  }
  double maxError() const
  {
    return error.maxCoeff();
  }
  double distance() const
  {
    return transform.translation().norm();
  }
  Marker& average(const Marker& other)
  {
    transform.translation() = (transform.translation() + other.transform.translation()) / 2;
    return *this;
  }
  Marker& changeReferenceFrame(const Isometry3d& transform)
  {
    this->transform.translation() = transform * this->transform.translation();
    return *this;
  }
  operator int() const
  {
    return id;
  }
  bool operator<(const Marker& other) const
  {
    return id < other.id;
  }
};

#endif  // MARKER_HPP
#ifndef ALIGNED_MARKER_HPP
#define ALIGNED_MARKER_HPP

#include <marker_based_localization/markers/marker.hpp>

using Eigen::Isometry3d;
using Eigen::Quaterniond;
using Eigen::Translation3d;
using Eigen::Vector3d;

// Marker with restricted orientation
class AlignedMarker : public Marker
{
public:
  AlignedMarker(unsigned int id, Isometry3d transform) : Marker()
  {
    this->id = id;
    this->transform = transform;
  }
  AlignedMarker(unsigned int id, Isometry3d transform, Vector3d error) : AlignedMarker(id, transform)
  {
    this->error = error;
  }
  AlignedMarker(unsigned int id, Isometry3d transform, Translation3d offset)
  {
    this->id = id;
    this->transform = Translation3d(transform.translation()) * Eigen::Quaterniond(transform.rotation()) * offset;
    this->transform.linear() = transform.linear();
  }
  AlignedMarker(unsigned int id, Isometry3d transform, Translation3d offset, Vector3d error)
    : AlignedMarker(id, transform, offset)
  {
    this->error = error;
  }
  AlignedMarker(unsigned int id, Translation3d position, Vector3d error)
    : AlignedMarker(id, Isometry3d(position), error)
  {
  }
  AlignedMarker(unsigned int id, Translation3d position, Quaterniond orientation, Translation3d offset)
    : AlignedMarker(id, Isometry3d(position * orientation), offset)
  {
  }
  AlignedMarker(unsigned int id, Translation3d position, Quaterniond orientation, Translation3d offset, Vector3d error)
    : AlignedMarker(id, Isometry3d(position * orientation), offset, error)
  {
  }

  void changeReferenceFrame(const Isometry3d& transform)
  {
    this->transform = transform * this->transform;
  }
  Marker toMarker() const
  {
    return Marker(id, transform, error);
  }
  Marker toMarker(Translation3d offset) const
  {
    return Marker(id, transform, offset, error);
  }
};

#endif  // ALIGNED_MARKER_HPP
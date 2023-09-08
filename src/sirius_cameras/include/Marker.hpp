#ifndef MARKER_HPP
#define MARKER_HPP

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "Eigen/src/Geometry/Transform.h"

using Eigen::Isometry3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

// Detected fiducial marker
class Marker
{
public:
  Marker(unsigned int id, Vector3d position, Vector3d error = Vector3d::Zero())
    : id(id), position(position), error(error)
  {
  }
  unsigned int id;
  Vector3d position;
  // Error, preferably variance in meters
  Vector3d error;

  double maxError() const
  {
    return error.maxCoeff();
  }

  double distance() const
  {
    return position.norm();
  }

  void average(const Marker& other)
  {
    position = (position + other.position) / 2;
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

// Marker with restricted orientation
class AlignedMarker : public Marker
{
public:
  AlignedMarker(unsigned int id, Vector3d position, Quaterniond orientation, Vector3d error = Vector3d::Zero())
    : Marker(id, position, error), orientation(orientation)
  {
    transform = Isometry3d::Identity();
    transform.translate(position);
    transform.rotate(orientation);
  }
  Quaterniond orientation;
  Isometry3d transform;
};
// Marker with offset
class OffsetMarker : public Marker
{
public:
  OffsetMarker(unsigned int id, Vector3d position, Quaterniond orientation, Vector3d offset,
               Vector3d error = Vector3d::Zero())
    : Marker(id, position, error)
  {
    this->position += orientation * offset;
  }
};

template <typename T>  // T is as Marker
class Markers
{
private:
  std::vector<T> markers;

public:
  void add(T marker, bool average = false)
  {
    auto it =
        std::lower_bound(markers.begin(), markers.end(), marker.id, [](const T& a, const int& b) { return a.id < b; });

    if (it == markers.end() || it->id != marker.id)
    {
      markers.insert(it, marker);
    }
    else if (average)
    {
      it->average(marker);
    }
    else
    {
      *it = marker;
    }
  }
  int size() const
  {
    return markers.size();
  }
  void clear()
  {
    markers.clear();
  }
  bool empty() const
  {
    return markers.empty();
  }
  T& operator[](const int index)
  {
    return markers[index];
  }
  const T& operator[](const int index) const
  {
    return markers[index];
  }
  // constant index operator

  typename std::vector<T>::iterator begin()
  {
    return markers.begin();
  }
  typename std::vector<T>::iterator end()
  {
    return markers.end();
  }
  typename std::vector<T>::const_iterator begin() const
  {
    return markers.begin();
  }
  typename std::vector<T>::const_iterator end() const
  {
    return markers.end();
  }
};

#endif  // MARKER_HPP
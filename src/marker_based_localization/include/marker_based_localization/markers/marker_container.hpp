#ifndef MARKER_CONTAINER_HPP
#define MARKER_CONTAINER_HPP

#include <vector>
#include <boost/optional.hpp>
#include <marker_based_localization/markers/marker.hpp>

using Eigen::Isometry3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

template <typename T>  // T is as Marker
class MarkerContainer
{
private:
  std::vector<T> marker_container;
  // Returns iterator to marker with given id using lower_bound function. If no marker with given id is found, returns
  // end
  typename std::vector<T>::iterator getIteratorById(const unsigned int id)
  {
    auto it = std::lower_bound(marker_container.begin(), marker_container.end(), id,
                               [](const auto& a, const unsigned int b) { return a.id < b; });

    if (it != marker_container.end() && it->id == id)
    {
      return it;
    }
    return marker_container.end();
  }
  typename std::vector<T>::const_iterator getIteratorById(const unsigned int id) const
  {
    auto it = std::lower_bound(marker_container.begin(), marker_container.end(), id,
                               [](const auto& a, const unsigned int b) { return a.id < b; });
    if (it != marker_container.end() && it->id == id)
    {
      return it;
    }
    return marker_container.end();
  }

public:
  MarkerContainer<T>& add(T marker, bool average = false)
  {
    auto it = getIteratorById(marker.id);

    if (it == marker_container.end())
    {
      marker_container.insert(it, marker);
    }
    else if (average)
    {
      it->average(marker);
    }
    else
    {
      *it = marker;
    }
    return *this;
  }
  boost::optional<T> getById(const unsigned int id) const
  {
    auto it = getIteratorById(id);
    if (it != marker_container.end())
    {
      return *it;
    }
    return boost::none;
  }

  MarkerContainer<T>& removeById(const unsigned int id)
  {
    auto it = getIteratorById(id);
    if (it != marker_container.end())
    {
      marker_container.erase(it);
    }
    return *this;
  }

  int size() const
  {
    return marker_container.size();
  }
  MarkerContainer<T>& clear()
  {
    marker_container.clear();
    return *this;
  }
  bool empty() const
  {
    return marker_container.empty();
  }
  MarkerContainer<T>& changeReferenceFrame(const Isometry3d& transform)
  {
    for (auto& marker : marker_container)
    {
      marker.changeReferenceFrame(transform);
    }
    return *this;
  }
  T& operator[](const int index)
  {
    return marker_container[index];
  }
  const T& operator[](const int index) const
  {
    return marker_container[index];
  }
  // constant index operator

  typename std::vector<T>::iterator begin()
  {
    return marker_container.begin();
  }
  typename std::vector<T>::iterator end()
  {
    return marker_container.end();
  }
  typename std::vector<T>::const_iterator begin() const
  {
    return marker_container.begin();
  }
  typename std::vector<T>::const_iterator end() const
  {
    return marker_container.end();
  }
};

#endif  // MARKER_CONTAINER_HPP
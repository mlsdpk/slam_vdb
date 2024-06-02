#ifndef SLAM_VDB_LIB_MEASUREMENT_POINTCLOUD_POINTCLOUD_BASE_HPP_
#define SLAM_VDB_LIB_MEASUREMENT_POINTCLOUD_POINTCLOUD_BASE_HPP_

#include <cstddef>

namespace slam_vdb {
namespace measurement::pointcloud {

/**
 * @brief A base class template for representing point clouds in a multi-dimensional space.
 *
 * This class uses the Curiously Recurring Template Pattern (CRTP) to allow derived classes
 * to specify the actual type of the point cloud and the number of dimensions.
 *
 * @tparam Derived The derived class type.
 * @tparam PointT The type of the points in the point cloud.
 */
template <typename Derived, typename PointT>
class PointCloudBase
{
  /**
   * @brief Gets a reference to the derived class.
   *
   * @return Derived& A reference to the derived class.
   */
  Derived& derived() { return *static_cast<Derived*>(this); }

  /**
   * @brief Gets a const reference to the derived class.
   *
   * @return const Derived& A const reference to the derived class.
   */
  const Derived& derived() const { return *static_cast<const Derived*>(this); }

public:
  /// Type alias for the point type.
  using point_t = PointT;

  /**
   * @brief Inserts a point into the point cloud.
   *
   * This method delegates the insertion to the derived class.
   *
   * @param point The point to insert.
   */
  void insert(const point_t& point) { derived().insert(point); }

  /**
   * @brief Clears all points from the point cloud.
   *
   * This method delegates the clearing operation to the derived class.
   */
  void clear() { derived().clear(); }

  /**
   * @brief Return the number of points in the point cloud.
   *
   * @return The number of points in the point cloud.
   */
  std::size_t size() const { return derived().size(); }

  /**
   * @brief Accesses the point at the specified index.
   *
   * @param index The index of the point.
   * @return point_t& A reference to the point at the specified index.
   */
  point_t& operator[](std::size_t index) { return derived().operator[](index); }

  /**
   * @brief Accesses the point at the specified index (const version).
   *
   * @param index The index of the point.
   * @return const point_t& A const reference to the point at the specified index.
   */
  const point_t& operator[](std::size_t index) const { return derived().operator[](index); }
};

}  // namespace measurement::pointcloud
}  // namespace slam_vdb

#endif  // SLAM_VDB_LIB_MEASUREMENT_POINTCLOUD_POINTCLOUD_BASE_HPP_
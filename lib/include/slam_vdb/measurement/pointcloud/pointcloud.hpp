#ifndef SLAM_VDB_LIB_MEASUREMENT_POINTCLOUD_POINTCLOUD_HPP_
#define SLAM_VDB_LIB_MEASUREMENT_POINTCLOUD_POINTCLOUD_HPP_

#include <cstddef>
#include <vector>

#include "slam_vdb/measurement/pointcloud/pointcloud_base.hpp"

namespace slam_vdb {
namespace measurement::pointcloud {

/**
 * @brief A class representing a point cloud, derived from PointCloudBase.
 *
 * This class holds a collection of points and provides methods to manipulate
 * the point cloud.
 *
 * @tparam PointT The type of the points in the point cloud.
 */
template <typename PointT>
class PointCloud : public PointCloudBase<PointCloud<PointT>, PointT>
{
public:
  /// Type alias for the point type.
  using point_t = PointT;

  /// Default constructor
  PointCloud() = default;

  /// Destructor
  ~PointCloud() = default;

  /// Copy constructor
  PointCloud(const PointCloud& other) = default;

  /// Copy assignment operator
  PointCloud& operator=(const PointCloud& other) = default;

  /// Move constructor
  PointCloud(PointCloud&& other) noexcept = default;

  /// Move assignment operator
  PointCloud& operator=(PointCloud&& other) noexcept = default;

  /**
   * @brief Inserts a point into the point cloud.
   *
   * @param point The point to insert.
   */
  void insert(const point_t& point) { m_points.push_back(point); }

  /**
   * @brief Clears all points from the point cloud.
   */
  void clear() { m_points.clear(); }

  /**
   * @brief Return the number of points in the point cloud.
   *
   * @return The number of points in the point cloud.
   */
  std::size_t size() const { return m_points.size(); }

  /**
   * @brief Accesses the point at the specified index.
   *
   * @param index The index of the point.
   * @return point_t& A reference to the point at the specified index.
   */
  point_t& operator[](std::size_t index) { return m_points[index]; }

  /**
   * @brief Accesses the point at the specified index (const version).
   *
   * @param index The index of the point.
   * @return const point_t& A const reference to the point at the specified index.
   */
  const point_t& operator[](std::size_t index) const { return m_points[index]; }

private:
  /// Container for storing the points in the point cloud.
  std::vector<point_t> m_points;
};

}  // namespace measurement::pointcloud
}  // namespace slam_vdb

#endif  // SLAM_VDB_LIB_MEASUREMENT_POINTCLOUD_POINTCLOUD_XYZ_HPP_
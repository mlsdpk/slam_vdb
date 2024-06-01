#ifndef SLAM_VDB_LIB_MEASUREMENT_POINT_POINT_XYZ_HPP_
#define SLAM_VDB_LIB_MEASUREMENT_POINT_POINT_XYZ_HPP_

#include <Eigen/Dense>
#include <cstddef>

#include "slam_vdb/measurement/point/point_base.hpp"

namespace slam_vdb {
namespace measurement::point {

// TODO: not really a good place to stay
namespace index_t {
static constexpr size_t x = 0;
static constexpr size_t y = 1;
static constexpr size_t z = 2;
}  // namespace index_t

/**
 * @brief A class representing a 3-dimensional point (XYZ) in space.
 *
 * This class inherits from PointBase to represent a point in a 3-dimensional space.
 * It uses the Eigen library for efficient vector operations.
 *
 * @tparam T The type of the coordinates (e.g., float, double).
 */
template <typename T>
class PointXYZ : public PointBase<PointXYZ<T>, T, 3u>
{
public:
  /// Type alias for the coordinate type.
  using value_t = T;

  /// Type alias for the container type used to store the coordinates.
  using container_t = Eigen::Matrix<value_t, 3, 1>;

  /// Default constructor
  PointXYZ() = default;

  /// Destructor
  ~PointXYZ() = default;

  /// Copy constructor
  PointXYZ(const PointXYZ& other) = default;

  /// Copy assignment operator
  PointXYZ& operator=(const PointXYZ& other) = default;

  /// Move constructor
  PointXYZ(PointXYZ&& other) noexcept = default;

  /// Move assignment operator
  PointXYZ& operator=(PointXYZ&& other) noexcept = default;

  /**
   * @brief Accesses the coordinate at the specified index.
   *
   * @param index The index of the coordinate.
   * @return value_t& A reference to the coordinate at the specified index.
   */
  value_t& operator[](std::size_t index) { return m_data[index]; }

  /**
   * @brief Accesses the coordinate at the specified index (const version).
   *
   * @param index The index of the coordinate.
   * @return const value_t& A const reference to the coordinate at the specified index.
   */
  const value_t& operator[](std::size_t index) const { return m_data[index]; }

private:
  /// Container for storing the coordinates, initialized to zero.
  container_t m_data{container_t::Zero(3u)};
};

}  // namespace measurement::point
}  // namespace slam_vdb

#endif  // SLAM_VDB_LIB_MEASUREMENT_POINT_POINT_XYZ_HPP_
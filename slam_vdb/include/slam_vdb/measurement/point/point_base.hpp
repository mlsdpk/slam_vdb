#ifndef SLAM_VDB_LIB_MEASUREMENT_POINT_POINT_BASE_HPP_
#define SLAM_VDB_LIB_MEASUREMENT_POINT_POINT_BASE_HPP_

#include <cstddef>

namespace slam_vdb {
namespace measurement::point {

/**
 * @brief A base class template for representing points in a multi-dimensional space.
 *
 * This class uses the Curiously Recurring Template Pattern (CRTP) to allow derived classes
 * to specify the actual type of the point, the type of the point's coordinates, and the
 * number of dimensions.
 *
 * @tparam Derived The derived class type.
 * @tparam T The type of the coordinates (e.g., float, double).
 * @tparam Dimension The number of dimensions.
 */
template <typename Derived, typename T, std::size_t Dimension>
class PointBase
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
  /// Type alias for the coordinate type
  using value_t = T;

  /// The number of dimensions of the point
  static constexpr size_t DIMENSION = Dimension;

  /**
   * @brief Accesses the coordinate at the specified index.
   *
   * This operator is provided to allow derived classes to define their own
   * coordinate access method.
   *
   * @param index The index of the coordinate.
   * @return value_t& A reference to the coordinate at the specified index.
   */
  value_t& operator[](std::size_t index) { return derived().operator[](index); }

  /**
   * @brief Accesses the coordinate at the specified index (const version).
   *
   * This operator is provided to allow derived classes to define their own
   * coordinate access method.
   *
   * @param index The index of the coordinate.
   * @return const value_t& A const reference to the coordinate at the specified index.
   */
  const value_t& operator[](std::size_t index) const { return derived().operator[](index); }

  /**
   * @brief Outputs the point to the specified output stream.
   *
   * This friend function allows the point to be printed in a human-readable format.
   *
   * @param os The output stream.
   * @param point The point to output.
   * @return std::ostream& A reference to the output stream.
   */
  friend std::ostream& operator<<(std::ostream& os, const PointBase& point)
  {
    os << "(";
    for (std::size_t i = 0; i < DIMENSION; ++i)
    {
      os << point[i];
      if (i < DIMENSION - 1)
      {
        os << ", ";
      }
    }
    os << ")";
    return os;
  }
};

}  // namespace measurement::point
}  // namespace slam_vdb

#endif  // SLAM_VDB_LIB_MEASUREMENT_POINT_POINT_BASE_HPP_
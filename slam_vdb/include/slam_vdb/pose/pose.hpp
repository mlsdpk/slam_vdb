#ifndef SLAM_VDB_POSE_POSE_HPP_
#define SLAM_VDB_POSE_POSE_HPP_

#include <Eigen/Dense>
#include <ostream>

#include "slam_vdb/pose/pose_base.hpp"

namespace slam_vdb {
namespace pose {

/**
 * @brief A class representing a pose in 2D or 3D space.
 *
 * This class template provides a representation of a pose, which includes
 * a translation and a rotation component. It supports both 2D and 3D poses.
 *
 * @tparam T The type of the values (e.g., float, double).
 * @tparam Dimension The dimension of the pose (2 for 2D, 3 for 3D). Default is 3.
 */
template <typename T, std::size_t Dimension = 3>
class Pose : public PoseBase<Pose<T, Dimension>, T>
{
public:
  /// @brief The type of the values.
  using value_t = T;

  /// @brief The dimension of the pose.
  static constexpr size_t DIMENSION = Dimension;

  /// @brief The type of the translation vector.
  using translation_t = Eigen::Matrix<value_t, DIMENSION, 1>;

  /// @brief The type of the rotation component, depends on the dimension.
  using rotation_t = typename std::conditional_t<(DIMENSION == 2u), T, Eigen::Quaternion<value_t>>;

  /**
   * @brief Default constructor. Initializes the translation to zero and rotation to identity.
   */
  Pose()
  {
    if constexpr (DIMENSION == 2u)
    {
      m_translation.setZero();
      m_rotation = 0.0;
    }
    else
    {
      m_translation.setZero();
      m_rotation = Eigen::Quaternion<value_t>::Identity();
    }
  }

  /**
   * @brief Parameterized constructor.
   *
   * @param translation The translation vector.
   * @param rotation The rotation component.
   */
  Pose(const translation_t& translation, const rotation_t& rotation)
    : m_translation(translation), m_rotation(rotation)
  {
  }

  /**
   * @brief Parameterized constructor for 2D pose.
   *
   * @param x The x-coordinate of the translation.
   * @param y The y-coordinate of the translation.
   * @param theta The rotation angle.
   */
  Pose(value_t x, value_t y, value_t theta)
  {
    static_assert(DIMENSION == 2u, "This constructor is only for 2D poses.");
    m_translation << x, y;
    m_rotation = theta;
  }

  /**
   * @brief Parameterized constructor for 3D poses.
   *
   * @param x The x-coordinate of the translation.
   * @param y The y-coordinate of the translation.
   * @param z The z-coordinate of the translation.
   * @param quaternion The rotation quaternion.
   */
  Pose(value_t x, value_t y, value_t z, const Eigen::Quaternion<value_t>& quaternion)
  {
    static_assert(DIMENSION == 3u, "This constructor is only for 3D poses.");
    m_translation << x, y, z;
    m_rotation = quaternion;
  }

  /**
   * @brief Copy constructor.
   *
   * @param other The other Pose object to copy from.
   */
  Pose(const Pose& other) = default;

  /**
   * @brief Move constructor.
   *
   * @param other The other Pose object to move from.
   */
  Pose(Pose&& other) noexcept = default;

  /**
   * @brief Copy assignment operator.
   *
   * @param other The other Pose object to copy from.
   * @return Reference to this Pose object.
   */
  Pose& operator=(const Pose& other) = default;

  /**
   * @brief Move assignment operator.
   *
   * @param other The other Pose object to move from.
   * @return Reference to this Pose object.
   */
  Pose& operator=(Pose&& other) noexcept = default;

  /**
   * @brief Destructor.
   */
  ~Pose() = default;

  /**
   * @brief Get the translation component.
   *
   * @return const reference to the translation vector.
   */
  const translation_t& translation() const { return m_translation; }

  /**
   * @brief Get the rotation component.
   *
   * @return const reference to the rotation component.
   */
  const rotation_t& rotation() const { return m_rotation; }

  /**
   * @brief Get a mutable reference to the translation component.
   *
   * @return reference to the translation vector.
   */
  translation_t& translation() { return m_translation; }

  /**
   * @brief Get a mutable reference to the rotation component.
   *
   * @return reference to the rotation component.
   */
  rotation_t& rotation() { return m_rotation; }

  /**
   * @brief Print the pose to an output stream.
   *
   * @param os The output stream.
   * @return reference to the output stream.
   */
  std::ostream& print(std::ostream& os) const
  {
    os << "Pose(translation: " << m_translation.transpose() << ", rotation: ";
    if constexpr (DIMENSION == 2u)
    {
      os << m_rotation;
    }
    else
    {
      os << m_rotation.coeffs().transpose();
    }
    os << ")";
    return os;
  }

private:
  /// @brief The translation component.
  translation_t m_translation;

  /// @brief The rotation component.
  rotation_t m_rotation;
};

/**
 * @brief Overload of the << operator to print the pose.
 *
 * @param os The output stream.
 * @param pose The pose to print.
 * @return reference to the output stream.
 */
template <typename T, std::size_t Dimension>
std::ostream& operator<<(std::ostream& os, const Pose<T, Dimension>& pose)
{
  return pose.print(os);
}

}  // namespace pose
}  // namespace slam_vdb

#endif  // SLAM_VDB_POSE_POSE_HPP_

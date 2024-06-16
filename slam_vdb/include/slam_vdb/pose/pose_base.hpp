#ifndef SLAM_VDB_POSE_POSE_BASE_HPP_
#define SLAM_VDB_POSE_POSE_BASE_HPP_

#include <ostream>

namespace slam_vdb {
namespace pose {

template <typename Derived, typename T>
class PoseBase
{
public:
  using value_t = T;

  // Access the derived class
  Derived& derived() { return *static_cast<Derived*>(this); }
  const Derived& derived() const { return *static_cast<const Derived*>(this); }

  // Print the pose
  friend std::ostream& operator<<(std::ostream& os, const PoseBase& pose)
  {
    return pose.derived().print(os);
  }
};

}  // namespace pose
}  // namespace slam_vdb

#endif  // SLAM_VDB_POSE_POSE_BASE_HPP_

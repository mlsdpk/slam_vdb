#include <gtest/gtest.h>

#include "slam_vdb/slam_vdb.hpp"

using namespace slam_vdb::pose;

TEST(pose, DefaultConstructor)
{
  Pose<float, 2> pose2d;
  EXPECT_EQ(pose2d.translation(), Eigen::Vector2f::Zero());
  EXPECT_EQ(pose2d.rotation(), 0.0f);

  Pose<float, 3> pose3d;
  EXPECT_EQ(pose3d.translation(), Eigen::Vector3f::Zero());
  EXPECT_EQ(pose3d.rotation(), Eigen::Quaternionf::Identity());
}

TEST(pose, ParameterizedConstructor)
{
  Eigen::Vector2f translation2d(1.0f, 2.0f);
  float rotation2d = 3.0f;
  Pose<float, 2> pose2d(translation2d, rotation2d);
  EXPECT_EQ(pose2d.translation(), translation2d);
  EXPECT_EQ(pose2d.rotation(), rotation2d);

  Eigen::Vector3f translation3d(1.0f, 2.0f, 3.0f);
  Eigen::Quaternionf rotation3d(Eigen::AngleAxisf(0.25f * M_PI, Eigen::Vector3f::UnitZ()));
  Pose<float, 3> pose3d(translation3d, rotation3d);
  EXPECT_EQ(pose3d.translation(), translation3d);
  EXPECT_EQ(pose3d.rotation().coeffs(), rotation3d.coeffs());
}

TEST(pose, Accessors)
{
  Pose<float, 2> pose(1.0f, 2.0f, 0.5f);
  EXPECT_FLOAT_EQ(pose.translation()(0), 1.0f);
  EXPECT_FLOAT_EQ(pose.translation()(1), 2.0f);
  EXPECT_FLOAT_EQ(pose.rotation(), 0.5f);

  pose.translation() = Pose<float, 2>::translation_t(3.0f, 4.0f);
  pose.rotation() = 1.0f;
  EXPECT_FLOAT_EQ(pose.translation()(0), 3.0f);
  EXPECT_FLOAT_EQ(pose.translation()(1), 4.0f);
  EXPECT_FLOAT_EQ(pose.rotation(), 1.0f);

  Eigen::Quaterniond q(1.0, 0.0, 0.0, 0.0);
  Pose<double, 3> pose3d(1.0, 2.0, 3.0, q);
  EXPECT_DOUBLE_EQ(pose3d.translation()(0), 1.0);
  EXPECT_DOUBLE_EQ(pose3d.translation()(1), 2.0);
  EXPECT_DOUBLE_EQ(pose3d.translation()(2), 3.0);
  EXPECT_EQ(pose3d.rotation().coeffs(), q.coeffs());

  Eigen::Quaterniond q2(0.0, 1.0, 0.0, 0.0);
  pose3d.translation() = Pose<double, 3>::translation_t(4.0, 5.0, 6.0);
  pose3d.rotation() = q2;
  EXPECT_DOUBLE_EQ(pose3d.translation()(0), 4.0);
  EXPECT_DOUBLE_EQ(pose3d.translation()(1), 5.0);
  EXPECT_DOUBLE_EQ(pose3d.translation()(2), 6.0);
  EXPECT_EQ(pose3d.rotation().coeffs(), q2.coeffs());
}

TEST(pose, PrintFunction)
{
  Pose<float, 2> pose2d(1.0f, 2.0f, 3.0f);
  std::ostringstream oss2d;
  oss2d << pose2d;
  EXPECT_EQ(oss2d.str(), "Pose(translation: 1 2, rotation: 3)");

  Pose<float, 3> pose3d(1.0f, 2.0f, 3.0f, Eigen::Quaternionf::Identity());
  std::ostringstream oss3d;
  oss3d << pose3d;
  EXPECT_EQ(oss3d.str(), "Pose(translation: 1 2 3, rotation: 0 0 0 1)");
}

TEST(pose, CopyConstructor)
{
  Pose<float, 2> pose2d(1.0f, 2.0f, 3.0f);
  Pose<float, 2> copy2d(pose2d);
  EXPECT_EQ(copy2d.translation(), pose2d.translation());
  EXPECT_EQ(copy2d.rotation(), pose2d.rotation());

  Pose<float, 3> pose3d(1.0f, 2.0f, 3.0f, Eigen::Quaternionf::Identity());
  Pose<float, 3> copy3d(pose3d);
  EXPECT_EQ(copy3d.translation(), pose3d.translation());
  EXPECT_EQ(copy3d.rotation().coeffs(), pose3d.rotation().coeffs());
}

TEST(pose, MoveConstructor)
{
  Pose<float, 2> pose2d(1.0f, 2.0f, 3.0f);
  Pose<float, 2> moved2d(std::move(pose2d));
  EXPECT_EQ(moved2d.translation(), (Eigen::Vector2f{1.0f, 2.0f}));
  EXPECT_EQ(moved2d.rotation(), 3.0f);

  Pose<float, 3> pose3d(1.0f, 2.0f, 3.0f, Eigen::Quaternionf::Identity());
  Pose<float, 3> moved3d(std::move(pose3d));
  EXPECT_EQ(moved3d.translation(), (Eigen::Vector3f{1.0f, 2.0f, 3.0f}));
  EXPECT_EQ(moved3d.rotation().coeffs(), Eigen::Quaternionf::Identity().coeffs());
}

TEST(pose, CopyAssignment)
{
  Pose<float, 2> pose2d(1.0f, 2.0f, 3.0f);
  Pose<float, 2> copy2d;
  copy2d = pose2d;
  EXPECT_EQ(copy2d.translation(), pose2d.translation());
  EXPECT_EQ(copy2d.rotation(), pose2d.rotation());

  Pose<float, 3> pose3d(1.0f, 2.0f, 3.0f, Eigen::Quaternionf::Identity());
  Pose<float, 3> copy3d;
  copy3d = pose3d;
  EXPECT_EQ(copy3d.translation(), pose3d.translation());
  EXPECT_EQ(copy3d.rotation().coeffs(), pose3d.rotation().coeffs());
}

TEST(pose, MoveAssignment)
{
  Pose<float, 2> pose2d(1.0f, 2.0f, 3.0f);
  Pose<float, 2> moved2d;
  moved2d = std::move(pose2d);
  EXPECT_EQ(moved2d.translation(), (Eigen::Vector2f{1.0f, 2.0f}));
  EXPECT_EQ(moved2d.rotation(), 3.0f);

  Pose<float, 3> pose3d(1.0f, 2.0f, 3.0f, Eigen::Quaternionf::Identity());
  Pose<float, 3> moved3d;
  moved3d = std::move(pose3d);
  EXPECT_EQ(moved3d.translation(), (Eigen::Vector3f{1.0f, 2.0f, 3.0f}));
  EXPECT_EQ(moved3d.rotation().coeffs(), Eigen::Quaternionf::Identity().coeffs());
}
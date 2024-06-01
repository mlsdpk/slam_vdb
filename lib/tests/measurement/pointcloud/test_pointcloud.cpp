#include <gtest/gtest.h>

#include <vector>

#include "slam_vdb/slam_vdb.hpp"

using namespace slam_vdb::measurement::point;
using namespace slam_vdb::measurement::pointcloud;

// Define a simple Point class for testing
template <typename T>
class TestPoint
{
public:
  static constexpr std::size_t DIMENSION = 3;

  T& operator[](std::size_t index) { return data[index]; }
  const T& operator[](std::size_t index) const { return data[index]; }

private:
  T data[DIMENSION] = {T(), T(), T()};
};

// Type aliases for testing
using TestPointInt = TestPoint<int>;
using TestPointFloat = TestPoint<float>;
using TestPointDouble = TestPoint<double>;

using pcl_int_t = PointCloud<TestPointInt>;

TEST(pointcloud_PointCloudInitialization, DefaultValues)
{
  pcl_int_t pc_int;
  PointCloud<TestPointFloat> pc_float;
  PointCloud<TestPointDouble> pc_double;

  EXPECT_EQ(pc_int.size(), 0);
  EXPECT_EQ(pc_float.size(), 0);
  EXPECT_EQ(pc_double.size(), 0);
}

TEST(pointcloud_PointCloudOperations, InsertAndAccess)
{
  PointCloud<TestPointInt> pc;

  TestPointInt point;
  point[0] = 1;
  point[1] = 2;
  point[2] = 3;
  pc.insert(point);

  EXPECT_EQ(pc[0][0], 1);
  EXPECT_EQ(pc[0][1], 2);
  EXPECT_EQ(pc[0][2], 3);
}

TEST(pointcloud_PointCloudOperations, Clear)
{
  PointCloud<TestPointInt> pc;

  TestPointInt point;
  point[0] = 1;
  point[1] = 2;
  point[2] = 3;
  pc.insert(point);

  pc.clear();
  EXPECT_EQ(pc.size(), 0);
}

TEST(pointcloud_PointCloudCopyAndMove, CopyConstructor)
{
  PointCloud<TestPointInt> pc1;

  TestPointInt point;
  point[0] = 1;
  point[1] = 2;
  point[2] = 3;
  pc1.insert(point);

  PointCloud<TestPointInt> pc2 = pc1;
  EXPECT_EQ(pc2[0][0], 1);
  EXPECT_EQ(pc2[0][1], 2);
  EXPECT_EQ(pc2[0][2], 3);
}

TEST(pointcloud_PointCloudCopyAndMove, CopyAssignment)
{
  PointCloud<TestPointInt> pc1;

  TestPointInt point;
  point[0] = 1;
  point[1] = 2;
  point[2] = 3;
  pc1.insert(point);

  PointCloud<TestPointInt> pc2;
  pc2 = pc1;
  EXPECT_EQ(pc2[0][0], 1);
  EXPECT_EQ(pc2[0][1], 2);
  EXPECT_EQ(pc2[0][2], 3);
}

TEST(pointcloud_PointCloudCopyAndMove, MoveConstructor)
{
  PointCloud<TestPointInt> pc1;

  TestPointInt point;
  point[0] = 1;
  point[1] = 2;
  point[2] = 3;
  pc1.insert(point);

  PointCloud<TestPointInt> pc2 = std::move(pc1);
  EXPECT_EQ(pc2[0][0], 1);
  EXPECT_EQ(pc2[0][1], 2);
  EXPECT_EQ(pc2[0][2], 3);
}

TEST(pointcloud_PointCloudCopyAndMove, MoveAssignment)
{
  PointCloud<TestPointInt> pc1;

  TestPointInt point;
  point[0] = 1;
  point[1] = 2;
  point[2] = 3;
  pc1.insert(point);

  PointCloud<TestPointInt> pc2;
  pc2 = std::move(pc1);
  EXPECT_EQ(pc2[0][0], 1);
  EXPECT_EQ(pc2[0][1], 2);
  EXPECT_EQ(pc2[0][2], 3);
}

TEST(pointcloud_PointCloudBoundary, OutOfBoundsAccess)
{
  PointCloud<TestPointInt> pc;

  TestPointInt point;
  point[0] = 1;
  point[1] = 2;
  point[2] = 3;
  pc.insert(point);

  EXPECT_THROW(pc[1], std::out_of_range);
}
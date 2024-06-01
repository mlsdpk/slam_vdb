#include <gtest/gtest.h>

#include "slam_vdb/slam_vdb.hpp"

using namespace slam_vdb::measurement::point;

TEST(point_xyz_initialization, default_values)
{
  PointXYZ<int> p_int;
  PointXYZ<float> p_float;
  PointXYZ<double> p_double;

  EXPECT_EQ(p_int[0], 0);
  EXPECT_EQ(p_int[1], 0);
  EXPECT_EQ(p_int[2], 0);
  EXPECT_FLOAT_EQ(p_float[0], 0.f);
  EXPECT_FLOAT_EQ(p_float[1], 0.f);
  EXPECT_FLOAT_EQ(p_float[2], 0.f);
  EXPECT_DOUBLE_EQ(p_double[0], 0.0);
  EXPECT_DOUBLE_EQ(p_double[1], 0.0);
  EXPECT_DOUBLE_EQ(p_double[2], 0.0);
}

TEST(point_xyz_member_functions, setters_and_getters)
{
  PointXYZ<int> p_int;
  PointXYZ<float> p_float;
  PointXYZ<double> p_double;

  p_int[0] = 100;
  p_int[1] = 200;
  p_int[2] = 300;
  EXPECT_EQ(p_int[0], 100);
  EXPECT_EQ(p_int[1], 200);
  EXPECT_EQ(p_int[2], 300);

  p_int[index_t::x] = 1000;
  p_int[index_t::y] = 2000;
  p_int[index_t::z] = 3000;
  EXPECT_EQ(p_int[index_t::x], 1000);
  EXPECT_EQ(p_int[index_t::y], 2000);
  EXPECT_EQ(p_int[index_t::z], 3000);

  p_float[0] = 100.f;
  p_float[1] = 200.f;
  p_float[2] = 300.f;
  EXPECT_EQ(p_float[0], 100.f);
  EXPECT_EQ(p_float[1], 200.f);
  EXPECT_EQ(p_float[2], 300.f);

  p_float[index_t::x] = 1000.f;
  p_float[index_t::y] = 2000.f;
  p_float[index_t::z] = 3000.f;
  EXPECT_EQ(p_float[index_t::x], 1000.f);
  EXPECT_EQ(p_float[index_t::y], 2000.f);
  EXPECT_EQ(p_float[index_t::z], 3000.f);

  p_double[0] = 100.0;
  p_double[1] = 200.0;
  p_double[2] = 300.0;
  EXPECT_EQ(p_double[0], 100.0);
  EXPECT_EQ(p_double[1], 200.0);
  EXPECT_EQ(p_double[2], 300.0);

  p_double[index_t::x] = 1000.0;
  p_double[index_t::y] = 2000.0;
  p_double[index_t::z] = 3000.0;
  EXPECT_EQ(p_double[index_t::x], 1000.0);
  EXPECT_EQ(p_double[index_t::y], 2000.0);
  EXPECT_EQ(p_double[index_t::z], 3000.0);
}

TEST(point_xyz_copy_and_move, copy_constructor)
{
  PointXYZ<int> p1;
  p1[index_t::x] = 10;
  p1[index_t::y] = 20;
  p1[index_t::z] = 30;

  PointXYZ<int> p2 = p1;
  EXPECT_EQ(p2[index_t::x], 10);
  EXPECT_EQ(p2[index_t::y], 20);
  EXPECT_EQ(p2[index_t::z], 30);
}

TEST(point_xyz_copy_and_move, copy_assignment)
{
  PointXYZ<int> p1;
  p1[index_t::x] = 10;
  p1[index_t::y] = 20;
  p1[index_t::z] = 30;

  PointXYZ<int> p2;
  p2 = p1;
  EXPECT_EQ(p2[index_t::x], 10);
  EXPECT_EQ(p2[index_t::y], 20);
  EXPECT_EQ(p2[index_t::z], 30);
}

TEST(point_xyz_copy_and_move, move_constructor)
{
  PointXYZ<int> p1;
  p1[index_t::x] = 10;
  p1[index_t::y] = 20;
  p1[index_t::z] = 30;

  PointXYZ<int> p2 = std::move(p1);
  EXPECT_EQ(p2[index_t::x], 10);
  EXPECT_EQ(p2[index_t::y], 20);
  EXPECT_EQ(p2[index_t::z], 30);
}

TEST(point_xyz_copy_and_move, move_assignment)
{
  PointXYZ<int> p1;
  p1[index_t::x] = 10;
  p1[index_t::y] = 20;
  p1[index_t::z] = 30;

  PointXYZ<int> p2;
  p2 = std::move(p1);
  EXPECT_EQ(p2[index_t::x], 10);
  EXPECT_EQ(p2[index_t::y], 20);
  EXPECT_EQ(p2[index_t::z], 30);
}

TEST(point_xyz_stream_output, output_operator)
{
  PointXYZ<int> p;
  p[index_t::x] = 1;
  p[index_t::y] = 2;
  p[index_t::z] = 3;

  std::ostringstream oss;
  oss << p;
  EXPECT_EQ(oss.str(), "(1, 2, 3)");
}
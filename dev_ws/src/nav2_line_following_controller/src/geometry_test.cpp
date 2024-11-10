#include "nav2_line_following_controller/geometry.h"
#include "gtest/gtest.h"

TEST(AngleTest, Subtraction) {
  Angle a = Angle::degrees(90);
  Angle b = Angle::degrees(45);
  Angle c = a - b;
  const double tolerance = 1e-6;
  EXPECT_NEAR(c.degrees(), 45.0, tolerance);

  a = Angle::degrees(0);
  b = Angle::degrees(180);
  c = a - b;
  EXPECT_NEAR(c.degrees(), -180.0, tolerance);

  a = Angle::degrees(179.9);
  b = Angle::degrees(0);
  c = a - b;
  EXPECT_NEAR(c.degrees(), 179.9, tolerance);

  a = Angle::degrees(270);
  b = Angle::degrees(90);
  c = a - b;
  EXPECT_NEAR(c.degrees(), -180.0, tolerance);

  a = Angle::degrees(10);
  b = Angle::degrees(20);
  c = a - b;
  EXPECT_NEAR(c.degrees(), -10.0, tolerance);

  a = Angle::degrees(-10);
  b = Angle::degrees(10);
  c = a - b;
  EXPECT_NEAR(c.degrees(), -20.0, tolerance);

  a = Angle::degrees(-10);
  b = Angle::degrees(-20);
  c = a - b;
  EXPECT_NEAR(c.degrees(), 10.0, tolerance);

  a = Angle::degrees(-10+360);
  b = Angle::degrees(-20);
  c = a - b;
  EXPECT_NEAR(c.degrees(), 10.0, tolerance);

  a = Angle::degrees(359);
  b = Angle::degrees(1);
  c = a - b;
  EXPECT_NEAR(c.degrees(), -2.0, tolerance);


}
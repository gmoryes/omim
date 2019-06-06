#include "testing/testing.hpp"

#include "poly_borders/help_structures.hpp"

#include "geometry/point2d.hpp"

#include <vector>

using namespace poly_borders;
using namespace std;

namespace
{
UNIT_TEST(PolyBordersPostprocessor_AreaTests_1)
{
  vector<m2::PointD> const points;
  TEST_EQUAL(FindPolygonArea(points, false /* convertToMeters */), 0.0, ());
}

UNIT_TEST(PolyBordersPostprocessor_AreaTests_2)
{
  vector<m2::PointD> points = {
      {1.0, 1.0}
  };
  TEST_EQUAL(FindPolygonArea(points, false /* convertToMeters */), 0.0, ());

  points = {
      {1.0, 2.0}
  };
  TEST_EQUAL(FindPolygonArea(points, false /* convertToMeters */), 0.0, ());
}

UNIT_TEST(PolyBordersPostprocessor_AreaTests_3)
{
  vector<m2::PointD> points = {
      {1.0, 1.0}, {3.0, 10.0}
  };
  TEST_EQUAL(FindPolygonArea(points, false /* convertToMeters */), 0.0, ());

  points = {
      {1.0, 2.0}, {-100.0, 200.0}
  };
  TEST_EQUAL(FindPolygonArea(points, false /* convertToMeters */), 0.0, ());

  points = {
      {0.0, 0.0}, {-100.0, 200.0}
  };
  TEST_EQUAL(FindPolygonArea(points, false /* convertToMeters */), 0.0, ());

  points = {
      {0.0, 0.0}, {0.0, 0.0}
  };
  TEST_EQUAL(FindPolygonArea(points, false /* convertToMeters */), 0.0, ());

  points = {
      {42.0, 42.0}, {42.0, 42.0}
  };
  TEST_EQUAL(FindPolygonArea(points, false /* convertToMeters */), 0.0, ());
}

UNIT_TEST(PolyBordersPostprocessor_AreaTests_4)
{
  vector<m2::PointD> points = {
      {42.0, 42.0}, {42.0, 42.0}, {42.0, 42.0}
  };
  TEST_EQUAL(FindPolygonArea(points, false /* convertToMeters */), 0.0, ());

  points = {
      {-3.0, -1.0}, {0.0, 2.0}, {3.0, -1.0}
  };
  TEST_EQUAL(FindPolygonArea(points, false /* convertToMeters */), 9.0, ());

  points = {
      {2.0, 0.0}, {0.0, 0.0}, {0.0, 2.0}
  };
  TEST_EQUAL(FindPolygonArea(points, false /* convertToMeters */), 2.0, ());

  points = {
      {1.0, 3.0}, {4.0, 4.0}, {5.0, 2.0}
  };
  TEST_EQUAL(FindPolygonArea(points, false /* convertToMeters */), 3.5, ());
}

UNIT_TEST(PolyBordersPostprocessor_AreaTests_5)
{
  vector<m2::PointD> points = {
      {-1.0, -1.0}, {-1.0, 1.0}, {1.0, 1.0}, {1.0, -1.0}
  };
  TEST_EQUAL(FindPolygonArea(points, false /* convertToMeters */), 4.0, ());

  points = {
      {1.0, 1.0}, {1.0, 7.0}, {4.0, 7.0}, {4.0, 1.0}
  };
  TEST_EQUAL(FindPolygonArea(points, false /* convertToMeters */), 18.0, ());

  points = {
      {-4.0, 3.0}, {-2.0, 6.0}, {1.0, 7.0}, {2.0, 0.0}
  };
  TEST_EQUAL(FindPolygonArea(points, false /* convertToMeters */), 23.0, ());
}

UNIT_TEST(PolyBordersPostprocessor_AreaTests_6)
{
  vector<m2::PointD> points = {
      {-1.0, 1.0}, {-3.0, 2.0}, {-4.0, 3.0}, {-1.0, 5.0}, {2.0, 2.0}, {-1.0, 2.0}
  };
  TEST_EQUAL(FindPolygonArea(points, false /* convertToMeters */), 11, ());
}
}  // namespace

#include "testing/testing.hpp"

#include "geometry/distance_on_sphere.hpp"
#include "geometry/point2d.hpp"
#include "geometry/mercator.hpp"

#include "base/math.hpp"

#include <fstream>
#include <iomanip>

std::vector<m2::PointD> CreateCircleGeometry(m2::PointD const & center, double radiusMercator,
                                             double angleStepDegree)
{
  std::vector<m2::PointD> result;
  double const radStep = base::DegToRad(angleStepDegree);
  for (double angleRad = 0; angleRad <= 2 * math::pi; angleRad += radStep)
  {
    result.emplace_back(center.x + radiusMercator * cos(angleRad),
                        center.y + radiusMercator * sin(angleRad));
  }
  return result;
}

UNIT_TEST(DistanceOnSphere)
{
  TEST_LESS(fabs(ms::DistanceOnSphere(0, -180, 0, 180)), 1.0e-6, ());
  TEST_LESS(fabs(ms::DistanceOnSphere(30, 0, 30, 360)), 1.0e-6, ());
  TEST_LESS(fabs(ms::DistanceOnSphere(-30, 23, -30, 23)), 1.0e-6, ());
  TEST_LESS(fabs(ms::DistanceOnSphere(90, 0, 90, 120)), 1.0e-6, ());
  TEST_LESS(fabs(ms::DistanceOnSphere(0, 0, 0, 180) - math::pi), 1.0e-6, ());
  TEST_LESS(fabs(ms::DistanceOnSphere(90, 0, -90, 120) - math::pi), 1.0e-6, ());
}

UNIT_TEST(DistanceOnEarth)
{
  TEST_LESS(fabs(ms::DistanceOnEarth(30, 0, 30, 180) * 0.001 - 13358), 1, ());
  TEST_LESS(fabs(ms::DistanceOnEarth(30, 0, 30, 45) * 0.001 - 4309), 1, ());
  TEST_LESS(fabs(ms::DistanceOnEarth(-30, 0, -30, 45) * 0.001 - 4309), 1, ());
  TEST_LESS(fabs(ms::DistanceOnEarth(47.37, 8.56, 53.91, 27.56) * 0.001 - 1519), 1, ());
  TEST_LESS(fabs(ms::DistanceOnEarth(43, 132, 38, -122.5) * 0.001 - 8302), 1, ());

  std::ofstream out("/tmp/points");
  out << std::setprecision(20);

  auto points = CreateCircleGeometry(mercator::FromLatLon({55.750441, 37.6175138}), mercator::MetersToMercator(300000), 5);
  out << points.size() << std::endl;
  for (auto const & point : points)
  {
    auto latlon = mercator::ToLatLon(point);
    out << latlon.m_lat << " " << latlon.m_lon << std::endl;
  }
}

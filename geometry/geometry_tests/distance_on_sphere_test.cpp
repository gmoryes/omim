#include "testing/testing.hpp"

#include "geometry/distance_on_sphere.hpp"
#include "geometry/mercator.hpp"

#include "base/math.hpp"

UNIT_TEST(DistanceOnSphere) // 165052298405
{
  double lat1 = 12.34, lon1 = 56.78;
  double lat2 = 87.65, lon2 = 43.21;
  auto dist = mercator::DistanceOnEarth2(ms::LatLon(lat1, lon1), ms::LatLon(lat2, lon2), false);
  auto dist2 = ms::DistanceOnSphere(lat1, lon1, lat2, lon2);
  TEST_ALMOST_EQUAL_ABS(dist, dist2, 1e-5, ());
//  TEST_LESS(fabs(mercator::DistanceOnEarth2(ms::LatLon(90, 0), ms::LatLon(90, 120), false)), 1.0e-6, ());
//  TEST_LESS(fabs(ms::DistanceOnSphere(30, 0, 30, 360)), 1.0e-6, ());
//  TEST_LESS(fabs(ms::DistanceOnSphere(-30, 23, -30, 23)), 1.0e-6, ());
//  TEST_LESS(fabs(ms::DistanceOnSphere(90, 0, 90, 120)), 1.0e-6, ());
//  TEST_LESS(fabs(ms::DistanceOnSphere(0, 0, 0, 180) - math::pi), 1.0e-6, ());
//  TEST_LESS(fabs(ms::DistanceOnSphere(90, 0, -90, 120) - math::pi), 1.0e-6, ());
}

UNIT_TEST(DistanceOnEarth)
{
  TEST_LESS(fabs(ms::DistanceOnEarth(30, 0, 30, 180) * 0.001 - 13358), 1, ());
  TEST_LESS(fabs(ms::DistanceOnEarth(30, 0, 30, 45) * 0.001 - 4309), 1, ());
  TEST_LESS(fabs(ms::DistanceOnEarth(-30, 0, -30, 45) * 0.001 - 4309), 1, ());
  TEST_LESS(fabs(ms::DistanceOnEarth(47.37, 8.56, 53.91, 27.56) * 0.001 - 1519), 1, ());
  TEST_LESS(fabs(ms::DistanceOnEarth(43, 132, 38, -122.5) * 0.001 - 8302), 1, ());
}

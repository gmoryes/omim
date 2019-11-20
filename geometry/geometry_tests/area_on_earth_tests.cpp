#include "testing/testing.hpp"

#include "geometry/area_on_earth.hpp"
#include "geometry/distance_on_sphere.hpp"
#include "geometry/mercator.hpp"

#include "base/math.hpp"
#include "base/timer.hpp"

#include <iostream>
#include <sstream>
#include <string>

UNIT_TEST(benchmark)
{
  CHECK(getenv("N"), ());
  std::stringstream ss;
  ss << std::string(getenv("N"));
  size_t n;
  ss >> n;
  auto latlon1 = ms::LatLon(12.34, 56.78);
  auto latlon2 = ms::LatLon(87.65, 43.21);

  auto point1 = mercator::FromLatLon(latlon1);
  auto point2 = mercator::FromLatLon(latlon2);

  LOG(LINFO, ("Current start. N =", n));
  base::HighResTimer timer;
  for (size_t i = 0; i < n; ++i)
  {
    auto const dist = mercator::DistanceOnEarth(point1, point2);
    point1 += m2::PointD(1e-6, 1e-8);
  }
  LOG(LINFO, ("Current time:", timer.ElapsedNano() / 1e6, "ms."));
}
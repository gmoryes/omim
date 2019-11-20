#include "testing/testing.hpp"

#include "geometry/area_on_earth.hpp"
#include "geometry/distance_on_sphere.hpp"
#include "geometry/mercator.hpp"

#include "base/math.hpp"
#include "base/timer.hpp"

#include <iostream>
#include <sstream>
#include <string>

UNIT_TEST(benchmark_0)
{
  auto latlon1 = ms::LatLon(12.34, 56.78);
  auto latlon2 = ms::LatLon(87.65, 43.21);

  auto point1 = mercator::FromLatLon(latlon1);
  auto point2 = mercator::FromLatLon(latlon2);

  auto const distCur = mercator::DistanceOnEarth(point1, point2);
  auto const distMisha = mercator::DistanceOnEarth2(latlon1, latlon2);
  TEST_ALMOST_EQUAL_ABS(distCur, distMisha, 1e-5, ());
}

UNIT_TEST(benchmark_1)
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
  }
  LOG(LINFO, ("Current time:", timer.ElapsedNano() / 1e6, "ms."));
}

UNIT_TEST(benchmark_2)
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
    auto const dist = mercator::DistanceOnEarth2(mercator::ToLatLon(point1), mercator::ToLatLon(point2));
  }
  LOG(LINFO, ("Current time:", timer.ElapsedNano() / 1e6, "ms."));
}
#include "testing/testing.hpp"

#include "geometry/area_on_earth.hpp"
#include "geometry/distance_on_sphere.hpp"
#include "geometry/mercator.hpp"

#include "base/math.hpp"
#include "base/timer.hpp"

#include <iostream>
#include <sstream>
#include <string>
#include <fstream>

std::vector<std::pair<m2::PointD, m2::PointD>> GetTestsPoints()
{
  std::vector<std::pair<m2::PointD, m2::PointD>> result;
  std::ifstream input("/tmp/points");
  m2::PointD p1, p2;
  while (input >> p1.x >> p1.y >> p2.x >> p2.y)
    result.emplace_back(p1, p2);
  LOG(LINFO, ("Read:", result.size(), "queries."));
  return result;
}

UNIT_TEST(benchmark_0)
{
  auto point1 = m2::PointD(7.7574100000000001387, 55.735217082024135493);
  auto point2 = m2::PointD(114.37600000000000477, -8.2415037947815203978);

  auto latlon1 = mercator::ToLatLon(point1);
  auto latlon2 = mercator::ToLatLon(point2);

  auto const distCurSphere = ms::DistanceOnSphere(latlon1.m_lat, latlon1.m_lon, latlon2.m_lat, latlon2.m_lon);
  auto const distCurMishaSpehe = mercator::DistanceOnEarth2(latlon1, latlon2, false);
  auto const distCur = mercator::DistanceOnEarth(point1, point2);
  auto const distMisha = mercator::DistanceOnEarth2(latlon1, latlon2);
  TEST_ALMOST_EQUAL_ABS(distCur, distMisha, 1e-5, ());

  auto const q = GetTestsPoints();
  for (auto const & p : q)
  {
    auto const & p1 = p.first;
    auto const & p2 = p.second;
    auto const ll1 = mercator::ToLatLon(p1);
    auto const ll2 = mercator::ToLatLon(p2);

    auto const distCurSphere = ms::DistanceOnSphere(ll1.m_lat, ll1.m_lon, ll2.m_lat, ll2.m_lon);
    auto const distCurMishaSpehe = mercator::DistanceOnEarth2(ll1, ll2, false);
    auto const distCur = mercator::DistanceOnEarth(p1, p2);
    auto const distMisha = mercator::DistanceOnEarth2(ll1, ll2);
    TEST_ALMOST_EQUAL_ABS(distCur, distMisha, 1.0, ());
  }
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

  LOG(LINFO, ("Old start. N =", n));
  auto const q = GetTestsPoints();
  base::HighResTimer timer;
  for (size_t i = 0; i < n; ++i)
  {
    for (auto const & p : q)
    {
      auto const & p1 = p.first;
      auto const & p2 = p.second;
      auto const dist = mercator::DistanceOnEarth(p1, p2);
    }
  }
  LOG(LINFO, ("Old time:", timer.ElapsedNano() / 1e6, "ms."));
}

UNIT_TEST(benchmark_2)
{
  CHECK(getenv("N"), ());
  std::stringstream ss;
  ss << std::string(getenv("N"));
  size_t n;
  ss >> n;

  LOG(LINFO, ("New start. N =", n));
  auto const q = GetTestsPoints();
  base::HighResTimer timer;
  for (size_t i = 0; i < n; ++i)
  {
    for (auto const & p : q)
    {
      auto const & p1 = p.first;
      auto const & p2 = p.second;
      auto const dist = mercator::DistanceOnEarth2(mercator::ToLatLon(p1), mercator::ToLatLon(p2));
    }
  }
  LOG(LINFO, ("New time:", timer.ElapsedNano() / 1e6, "ms."));
}
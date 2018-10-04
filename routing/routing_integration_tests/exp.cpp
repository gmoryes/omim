#include "testing/testing.hpp"

#include "routing/routing_integration_tests/routing_test_tools.hpp"
#include "routing_common/car_model.hpp"

#include "storage/country_parent_getter.hpp"

#include "geometry/mercator.hpp"

#include <limits>
#include <string>
#include <cmath>
#include <memory>

using namespace routing;

namespace
{
static auto constexpr kEps = 10.0;

double TestRoute(m2::PointD const & start, m2::PointD const & end, bool enableLandmarks)
{
  TRouteResult routeResult =
    CalculateRoute(integration::GetVehicleComponents<VehicleType::Car>(), start, {0., 0.}, end, enableLandmarks);
  RouterResultCode const result = routeResult.second;
  auto route = routeResult.first;
  TEST_EQUAL(result, RouterResultCode::NoError, ());
  return route->GetTotalDistanceMeters();
}
/*
UNIT_TEST(BenchmarkTest_Landmarks_1_Yes)
{
  double yes, no;
  auto from = MercatorBounds::FromLatLon(55.797529, 37.538253);
  auto end = MercatorBounds::FromLatLon(55.721194, 37.622323);

  yes = TestRoute(from, end, true);
  no = TestRoute(from, end, false);

  TEST(base::AlmostEqualAbs(yes, no, kEps), ());
}

UNIT_TEST(BenchmarkTest_Landmarks_2_Yes)
{
  double yes, no;
  auto from = MercatorBounds::FromLatLon(55.7891799, 37.635028);
  auto end = MercatorBounds::FromLatLon(55.6970868, 37.5044513);

  yes = TestRoute(from, end, true);
  no = TestRoute(from, end, false);

  TEST(base::AlmostEqualAbs(yes, no, kEps), ());
}

UNIT_TEST(BenchmarkTest_Landmarks_3_Yes)
{
  double yes, no;
  auto from = MercatorBounds::FromLatLon(55.7692285, 37.6458008);
  auto end = MercatorBounds::FromLatLon(55.7578737, 37.5855697);

  yes = TestRoute(from, end, true);
  no = TestRoute(from, end, false);

  TEST(base::AlmostEqualAbs(yes, no, kEps), ());
}
*/
UNIT_TEST(BenchmarkTest_Landmarks_4_Yes)
{
  double yes, no;
  auto from = MercatorBounds::FromLatLon(55.7526598, 37.5849678);
  auto end = MercatorBounds::FromLatLon(55.655283, 37.6052212);

  yes = TestRoute(from, end, true);
  no = TestRoute(from, end, false);

  TEST(base::AlmostEqualAbs(yes, no, kEps), ());
}
/*
UNIT_TEST(BenchmarkTest_Landmarks_5_Yes)
{
  double yes, no;
  auto from = MercatorBounds::FromLatLon(55.6641332, 37.7544071);
  auto end = MercatorBounds::FromLatLon(55.8075016, 37.4656657);

  yes = TestRoute(from, end, true);
  no = TestRoute(from, end, false);

  TEST(base::AlmostEqualAbs(yes, no, kEps), ());
}

UNIT_TEST(BenchmarkTest_Landmarks_6_Yes)
{
  double yes, no;
  auto from = MercatorBounds::FromLatLon(55.6645119, 37.7536088);
  auto end = MercatorBounds::FromLatLon(55.8075016, 37.4656657);

  yes = TestRoute(from, end, true);
  no = TestRoute(from, end, false);

  TEST(base::AlmostEqualAbs(yes, no, kEps), ());
}

UNIT_TEST(BenchmarkTest_Landmarks_7_Yes)
{
  double yes, no;
  auto from = MercatorBounds::FromLatLon(55.666451, 37.7408925);
  auto end = MercatorBounds::FromLatLon(55.8138161, 37.4722449);

  yes = TestRoute(from, end, true);
  no = TestRoute(from, end, false);

  TEST(base::AlmostEqualAbs(yes, no, kEps), ());
}
*/
/*
UNIT_TEST(BigTest_Landmarks_1_6)
{
  // 1 yes
  {
    auto from = MercatorBounds::FromLatLon(55.797529, 37.538253);
    auto end = MercatorBounds::FromLatLon(55.721194, 37.622323);
    TestRoute(from, end, true);
  }
  // 1 no
  {
    auto from = MercatorBounds::FromLatLon(55.797529, 37.538253);
    auto end = MercatorBounds::FromLatLon(55.721194, 37.622323);
    TestRoute(from, end, false);
  }

  // 2 yes
  {
    auto from = MercatorBounds::FromLatLon(55.7891799, 37.635028);
    auto end = MercatorBounds::FromLatLon(55.6970868, 37.5044513);
    TestRoute(from, end, true);
  }
  // 2 no
  {
    auto from = MercatorBounds::FromLatLon(55.7891799, 37.635028);
    auto end = MercatorBounds::FromLatLon(55.6970868, 37.5044513);
    TestRoute(from, end, false);
  }

  // 3 yes
  {
    auto from = MercatorBounds::FromLatLon(55.7692285, 37.6458008);
    auto end = MercatorBounds::FromLatLon(55.7578737, 37.5855697);
    TestRoute(from, end, true);
  }
  // 3 no
  {
    auto from = MercatorBounds::FromLatLon(55.7692285, 37.6458008);
    auto end = MercatorBounds::FromLatLon(55.7578737, 37.5855697);
    TestRoute(from, end, false);
  }

  // 4 yes
  {
    auto from = MercatorBounds::FromLatLon(55.7526598, 37.5849678);
    auto end = MercatorBounds::FromLatLon(55.655283, 37.6052212);
    TestRoute(from, end, true);
  }
  // 4 no
  {
    auto from = MercatorBounds::FromLatLon(55.7526598, 37.5849678);
    auto end = MercatorBounds::FromLatLon(55.655283, 37.6052212);
    TestRoute(from, end, false);
  }

  // 5 yes
  {
    auto from = MercatorBounds::FromLatLon(55.6641332, 37.7544071);
    auto end = MercatorBounds::FromLatLon(55.8075016, 37.4656657);
    TestRoute(from, end, true);
  }
  // 5 no
  {
    auto from = MercatorBounds::FromLatLon(55.6641332, 37.7544071);
    auto end = MercatorBounds::FromLatLon(55.8075016, 37.4656657);
    TestRoute(from, end, false);
  }
  // 6, yes
  {
    auto from = MercatorBounds::FromLatLon(55.6645119, 37.7536088);
    auto end = MercatorBounds::FromLatLon(55.8075016, 37.4656657);
    TestRoute(from, end, true);
  }
  // 6, no
  {
    auto from = MercatorBounds::FromLatLon(55.6645119, 37.7536088);
    auto end = MercatorBounds::FromLatLon(55.8075016, 37.4656657);
    TestRoute(from, end, false);
  }
}
*/
/*
UNIT_TEST(BenchmarkTest_Checker)
{
  std::ifstream input("/tmp/checker");

  double fromLat, fromLon, toLat, toLon, dist;
  size_t landmarkIndex;
  bool enableLandmarks = false;
  double from2Landmark;
  uint32_t featureId, segmentId;
  while (input >> fromLat >> fromLon >> toLat >> toLon >> dist >> landmarkIndex >> from2Landmark >>
         featureId >> segmentId)
  {
    m2::PointD start(MercatorBounds::FromLatLon(fromLat, fromLon));
    m2::PointD end(MercatorBounds::FromLatLon(toLat, toLon));
    TRouteResult routeResult =
     CalculateRoute(integration::GetVehicleComponents<VehicleType::Car>(), start, {0., 0.}, end,
                    enableLandmarks);

    RouteWeight const weight = routeResult.first->m_routeWeight;
    auto const result = routeResult.second;

    TEST(result == RouterResultCode::NoError,
        ("Can not build route from:", MercatorBounds::ToLatLon(
          start), "to:", MercatorBounds::ToLatLon(start)));
    LOG(LINFO, ("total time:", routeResult.first->GetTotalTimeSec()));

    auto fromLatLon = ms::LatLon({fromLat, fromLon});
    auto endLatLon = ms::LatLon({toLat, toLon});

    static auto constexpr kEps = RouteWeight(0.1);
    if (RouteWeight(dist) > weight + kEps)
    {
      LOG(LINFO, ("Dist from(", fromLatLon, ") to(", endLatLon, ") landmark(",
        landmarkIndex, ") is more than another - dist:", dist, "true:", weight.GetWeight(),
        "from2Landmark:", from2Landmark, "featureId:", featureId, "segmentId:", segmentId));
    }
  }
}*/
}  // namespace

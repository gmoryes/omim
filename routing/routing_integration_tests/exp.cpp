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

void TestRoute(m2::PointD const & start, m2::PointD const & end, bool enableLandmarks)
{
  TRouteResult routeResult =
    CalculateRoute(integration::GetVehicleComponents<VehicleType::Car>(), start, {0., 0.}, end, enableLandmarks);
  RouterResultCode const result = routeResult.second;
  TEST_EQUAL(result, RouterResultCode::NoError, ());
}

UNIT_TEST(BenchmarkTest_Landmarks_Yes)
{
  auto from = MercatorBounds::FromLatLon(55.797529, 37.538253);
  auto end = MercatorBounds::FromLatLon(55.721194, 37.622323);
  TestRoute(from, end, true);
}

/*UNIT_TEST(BenchmarkTest_Landmarks_No)
{
  auto from = MercatorBounds::FromLatLon(55.797529, 37.538253);
  auto end = MercatorBounds::FromLatLon(55.721194, 37.622323);
  TestRoute(from, end, false);
}*/
/*
UNIT_TEST(BenchmarkTest_Checker)
{
  std::ifstream input("/tmp/checker");

  double fromLat, fromLon, toLat, toLon, dist;
  size_t landmarkIndex;
  bool enableLandmarks = false;
  double from2Landmark, to2Landmark;
  uint32_t featureId, segmentId;
  while (input >> fromLat >> fromLon >> toLat >> toLon >> dist >> landmarkIndex >> from2Landmark >> to2Landmark >>
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

    if (!(RouteWeight(dist) < weight))
    {
      LOG(LINFO, ("Dist from(", fromLatLon, ") to(", endLatLon, ") landmark(",
        landmarkIndex, ") is more than another - dist:", dist, "true:", weight.GetWeight(),
        "from2Landmark:", from2Landmark, ", to2Landmark:", to2Landmark,
        "featureId:", featureId, "segmentId:", segmentId));
    }
  }
}
*/
}  // namespace

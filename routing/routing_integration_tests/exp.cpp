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
double TestRoute(m2::PointD const & start, m2::PointD const & end)
{
  TRouteResult routeResult =
    CalculateRoute(integration::GetVehicleComponents<VehicleType::Car>(), start, {0., 0.}, end);
  RouterResultCode const result = routeResult.second;
  auto route = routeResult.first;
  TEST_EQUAL(result, RouterResultCode::NoError, ());
  return route->GetTotalDistanceMeters();
}
/*
UNIT_TEST(BenchmarkTest_CrossMwm_1)
{
  //54.290218, 28.842382; 55.066221, 32.688343; 57.817394, 28.334366; 59.938732, 30.316230;

  {
    auto from = MercatorBounds::FromLatLon(54.290218, 28.842382);
    auto end = MercatorBounds::FromLatLon(55.066221, 32.688343);
    TestRoute(from, end);
  }
  {
    auto from = MercatorBounds::FromLatLon(55.066221, 32.688343);
    auto end = MercatorBounds::FromLatLon(57.817394, 28.334366);
    TestRoute(from, end);
  }
  {
    auto from = MercatorBounds::FromLatLon(57.817394, 28.334366);
    auto end = MercatorBounds::FromLatLon(59.938732, 30.316230);
    TestRoute(from, end);
  }
}
*/
UNIT_TEST(BenchmarkTest_CrossMwm_2)
{
  //54.290218, 28.842382; 55.066221, 32.688343; 57.817394, 28.334366; 59.938732, 30.316230;

  {
    auto from = MercatorBounds::FromLatLon(51.1503946, 15.0019905);
    auto end = MercatorBounds::FromLatLon(51.1500819, 14.9989472);
    TestRoute(from, end);
  }
}
}  // namespace
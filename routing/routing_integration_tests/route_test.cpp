#include "testing/testing.hpp"

#include "routing/routing_callbacks.hpp"

#include "routing/routing_integration_tests/routing_test_tools.hpp"

#include "geometry/mercator.hpp"

#include "std/limits.hpp"

using namespace routing;

namespace
{
  UNIT_TEST(EnglandToFranceRouteLeMansTest)
  {
    integration::CalculateRouteAndTestRouteLength(
        integration::GetVehicleComponents<VehicleType::Car>(),
        MercatorBounds::FromLatLon(51.09276, 1.11369), {0., 0.},
        MercatorBounds::FromLatLon(50.93227, 1.82725), 60498.);
  }
}  // namespace

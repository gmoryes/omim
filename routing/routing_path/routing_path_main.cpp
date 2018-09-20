#include "routing/routing_integration_tests/routing_test_tools.hpp"

#include "routing_common/car_model.hpp"
#include "routing/routing_callbacks.hpp"
#include "routing/router_delegate.hpp"
#include "routing/vehicle_mask.hpp"

#include "geometry/point2d.hpp"

#include "base/assert.hpp"
#include "base/logging.hpp"

#include <iostream>
#include <cstdio>
#include <string>
#include <cassert>
#include <utility>
#include <memory>

using namespace routing;

using TRouteResult = std::pair<std::shared_ptr<Route>, RouterResultCode>;

TRouteResult CalculateRoute(m2::PointD const & start, m2::PointD const & end)
{
  RouterDelegate delegate;
  std::shared_ptr<Route> route = std::make_shared<Route>("mapsme", 0 /* route id */);
  RouterResultCode result = integration::GetVehicleComponents<VehicleType::Car>().GetRouter().CalculateRoute(
    Checkpoints(start, end), {0., 0.}, false /* adjust */, delegate, *route, false /* enableLandmarks */);
  ASSERT(route, ());
  return TRouteResult(route, result);
}

int main(int argc, char ** argv)
{
  double fromLat, fromLon;
  double toLat, toLon;

  if (argc != 5)
  {
    std::cout << "./routing_path fromLat fromLon toLat toLon" << std::endl;
    return 0;
  }

  sscanf(argv[1], "%lf", &fromLat);
  sscanf(argv[2], "%lf", &fromLon);

  sscanf(argv[3], "%lf", &toLat);
  sscanf(argv[4], "%lf", &toLon);

  m2::PointD start(MercatorBounds::FromLatLon({fromLat, fromLon}));
  m2::PointD end(MercatorBounds::FromLatLon({toLat, toLon}));
  TRouteResult routeResult = CalculateRoute(start, end);

  RouteWeight const weight = routeResult.first->m_routeWeight;
  auto const result = routeResult.second;
  if (result != RouterResultCode::NoError)
  {
    std::cout << "error" << std::endl;
    return 0;
  }

  std::cout << "route_weight = " << weight.GetWeight() << std::endl;

  return 0;
}

#include "testing/testing.hpp"

#include "routing/routing_callbacks.hpp"

#include "routing/routing_integration_tests/routing_test_tools.hpp"

#include "geometry/mercator.hpp"

#include <fstream>
#include <limits>
#include <string>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <functional>
#include <queue>
#include <atomic>
#include <sstream>

using namespace routing;

namespace
{
size_t GetValue(std::string const & key)
{
  std::stringstream ss;
  CHECK(std::getenv(key.c_str()), ("No such key:", key));
  std::string str = std::string(std::getenv(key.c_str()));
  ss << str;
  size_t result;
  ss >> result;
  return result;
}

void CheckEnvVar(std::string const & str)
{
  bool exists = std::getenv(str.c_str()) && !std::string(std::getenv(str.c_str())).empty();
  CHECK(exists, ("Can not find", str, "env variable!"));
}

UNIT_TEST(BulkTest)
{
  std::vector<std::string> absent;
  CheckEnvVar("COORDS");
  //std::string allMwmsPath = std::string(std::getenv("MWMS_DIR"));
  std::string pathToCoord = std::string(std::getenv("COORDS"));
  std::ifstream input(pathToCoord);
  if (!input) {
    LOG(LINFO, ("Can not open:", pathToCoord));
    CHECK(false, ("can not open:", pathToCoord));
  }

  size_t left = GetValue("LEFT");
  size_t right = GetValue("RIGHT");

  ms::LatLon start{};
  ms::LatLon end{};

  std::vector<std::pair<ms::LatLon, ms::LatLon>> data;
  std::string type;
  size_t counter = 0;

//  std::ofstream tmp("/home/m.gorbushin/list_more_70km");
//  tmp << std::setprecision(20);
//  std::ifstream asd("/Users/m.gorbushin/projects/Mail/fork_omim/vlad");
//  std::ofstream dsa("/Users/m.gorbushin/projects/Mail/fork_omim/vlad_x_y");
//  while (asd >> start.lat >> start.lon >> end.lat >> end.lon)
//  {
//    auto a = MercatorBounds::FromLatLon(start);
//    auto b = MercatorBounds::FromLatLon(end);
//
//    dsa << a.x << ' ' << a.y << ' ' << b.x << ' ' << b.y << std::endl;
//  }
//
//  return;
  
  while (input >> start.lat >> start.lon >> end.lat >> end.lon)
  {
    if (left <= counter && counter < right)
      data.emplace_back(start, end);

    counter++;
    if (counter >= right)
      break;
//    auto a = MercatorBounds::FromLatLon(start);
//    auto b = MercatorBounds::FromLatLon(end);
//    if (MercatorBounds::DistanceOnEarth(a, b) >= 70000)
//    {
//      tmp << start.lat << ' ' << start.lon << ' ' << end.lat << ' ' << end.lon << std::endl;
//    }
  }

  //return;

  for (size_t i = 0; i < data.size(); ++i)
  {
    start = data[i].first;
    end = data[i].second;

    LOG(LINFO, ("N =>", i + left, "start:", start, "finish:", end));

    auto & routerComponents = integration::GetVehicleComponents<VehicleType::Car>();

    TRouteResult const routeResult =
      integration::CalculateRoute(routerComponents,
                                  MercatorBounds::FromLatLon(start), {0.0, 0.0},
                                  MercatorBounds::FromLatLon(end));

    RouterResultCode const result = routeResult.second;
    if (result != RouterResultCode::NoError)
      LOG(LINFO, ("Can not build route."));
  }
}
}  // namespace

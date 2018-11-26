#include "testing/testing.hpp"

#include "routing/routing_integration_tests/routing_test_tools.hpp"

#include "routing/routing_tests/tools.hpp"

#include "routing/route.hpp"
#include "routing/routing_callbacks.hpp"
#include "routing/routing_helpers.hpp"
#include "routing/routing_session.hpp"
#include "routing/speed_camera_manager.hpp"

#include "platform/location.hpp"
#include "platform/measurement_utils.hpp"

#include "geometry/point2d.hpp"

#include "base/assert.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

using namespace routing;
using namespace routing::turns;
using namespace std;

namespace
{
string const kCameraOnTheWay = "Speed camera on the way";

location::GpsInfo MoveTo(ms::LatLon const & coords, double speed = -1)
{
  static auto constexpr kGpsAccuracy = 0.01;

  location::GpsInfo info;
  info.m_horizontalAccuracy = kGpsAccuracy;
  info.m_verticalAccuracy = kGpsAccuracy;
  info.m_latitude = coords.lat;
  info.m_longitude = coords.lon;
  info.m_speedMpS = speed;
  return info;
}

void ChangePosition(ms::LatLon const & coords, double speedKmPH, RoutingSession & routingSession)
{
  routingSession.OnLocationPositionChanged(MoveTo({coords.lat, coords.lon}, KMPH2MPS(speedKmPH)));
}

void InitRoutingSession(ms::LatLon const & from, ms::LatLon const & to, RoutingSession & routingSession,
                        SpeedCameraManagerMode mode = SpeedCameraManagerMode::Auto)
{
  TRouteResult const routeResult =
    integration::CalculateRoute(integration::GetVehicleComponents<VehicleType::Car>(),
                                MercatorBounds::FromLatLon(from), m2::PointD::Zero(),
                                MercatorBounds::FromLatLon(to));

  Route & route = *routeResult.first;
  RouterResultCode const result = routeResult.second;
  TEST_EQUAL(result, RouterResultCode::NoError, ());

  routingSession.Init(nullptr /* RoutingStatisticsCallback */,
                      nullptr /* PointCheckCallback */);
  routingSession.SetRoutingSettings(routing::GetRoutingSettings(routing::VehicleType::Car));
  routingSession.AssignRouteForTesting(make_shared<Route>(route), result);
  routingSession.SetTurnNotificationsUnits(measurement_utils::Units::Metric);
  routingSession.GetSpeedCamManager().SetMode(mode);
  string const engShortJson = R"(
    {
      "unknown_camera": ")" + kCameraOnTheWay + R"("
    }
  )";
  routingSession.SetLocaleWithJsonForTesting(engShortJson, "en");
}

bool CheckVoiceNotification(RoutingSession & routingSession)
{
  vector<string> notifications;
  routingSession.GenerateNotifications(notifications);
  return any_of(notifications.begin(), notifications.end(), [](auto const & item) {
    return item == kCameraOnTheWay;
  });
}

bool CheckBeepSignal(RoutingSession & routingSession)
{
  return routingSession.GetSpeedCamManager().ShouldPlayWarningSignal();
}

bool CheckZone(RoutingSession & routingSession, double speedKmPH, SpeedCameraManager::Interval interval)
{
  SpeedCameraOnRoute const & closestCamera = routingSession.GetSpeedCamManager().GetClosestCamForTests();
  CHECK(closestCamera.IsValid(), ("No speed camera found."));

  double speedMpS = routing::KMPH2MPS(speedKmPH);
  double passedDist = routingSession.GetRouteForTests()->GetCurrentDistanceFromBeginMeters();
  double distToCamera = closestCamera.m_distFromBeginMeters - passedDist;

  return interval == SpeedCameraManager::GetIntervalByDistToCam(distToCamera, speedMpS);
}

bool NoCameraFound(RoutingSession & routingSession)
{
  SpeedCameraOnRoute const & closestCamera = routingSession.GetSpeedCamManager().GetClosestCamForTests();
  return !closestCamera.IsValid();
}

// Mode: Auto/Always
// ____Notification___|___beep____|_____(exceed speed limit here) Impact camera zone_____|
// Expected: Beep signal.
UNIT_TEST(SpeedCameraNotification_AutoAlwaysMode_1)
{
  std::vector<SpeedCameraManagerMode> modes = {SpeedCameraManagerMode::Auto, SpeedCameraManagerMode::Always};
  for (auto const mode : modes)
  {
    RoutingSession routingSession;
    InitRoutingSession({55.67931, 37.53268} /* from */,
                       {55.68764, 37.54508} /* to   */,
                       routingSession,
                       mode);

    {
      double speedKmPH = 100.0;
      ChangePosition({55.68126, 37.53551}, speedKmPH, routingSession);
      TEST(CheckZone(routingSession, speedKmPH, SpeedCameraManager::Interval::ImpactZone), ());
      TEST(!CheckVoiceNotification(routingSession), ());
      TEST(CheckBeepSignal(routingSession), ());
    }
  }
}

// Mode: Auto/Always
// ____Notification___|___beep____|_____(exceed speed limit here) Impact camera zone_____|
// Expected: Beep signal.
UNIT_TEST(SpeedCameraNotification_AutoAlwaysMode_2)
{
  std::vector<SpeedCameraManagerMode> modes = {SpeedCameraManagerMode::Auto, SpeedCameraManagerMode::Always};
  for (auto const mode : modes)
  {
    RoutingSession routingSession;
    InitRoutingSession({55.74070, 37.61681} /* from */,
                       {55.74885, 37.61036} /* to   */,
                       routingSession,
                       mode);

    {
      double speedKmPH = 100.0;
      ChangePosition({55.74505, 37.61384}, speedKmPH, routingSession);
      TEST(CheckZone(routingSession, speedKmPH, SpeedCameraManager::Interval::ImpactZone), ());
      TEST(!CheckVoiceNotification(routingSession), ());
      TEST(CheckBeepSignal(routingSession), ());
    }
  }
}

// Mode: Auto/Always
// ____Notification___|___(exceed speed limit here) beep____|_____Impact camera zone_____|
// Expected: Beep signal.
UNIT_TEST(SpeedCameraNotification_AutoAlwaysMode_3)
{
  std::vector<SpeedCameraManagerMode> modes = {SpeedCameraManagerMode::Auto, SpeedCameraManagerMode::Always};
  for (auto const mode : modes)
  {
    RoutingSession routingSession;
    InitRoutingSession({55.76801, 37.59363} /* from */,
                       {55.75947, 37.58484} /* to   */,
                       routingSession,
                       mode);

    // No danger here.
    {
      double speedKmPH = 100.0;
      ChangePosition({55.76766, 37.59260}, speedKmPH, routingSession);
      TEST(NoCameraFound(routingSession), ());
      TEST(!CheckVoiceNotification(routingSession), ());
      TEST(!CheckBeepSignal(routingSession), ());
    }

    // Exceed speed limit in beep zone.
    {
      double speedKmPH = 100.0;
      ChangePosition({55.76589, 37.58999}, speedKmPH, routingSession);
      TEST(CheckZone(routingSession, speedKmPH, SpeedCameraManager::Interval::BeepSignalZone), ());
      TEST(!CheckVoiceNotification(routingSession), ());
      TEST(CheckBeepSignal(routingSession), ());
    }
  }
}

// Next tests about camera which is not part of way in OSM.
// This link (camera and way) was found by geometry index.

// Mode: Auto/Always
// ____Notification___|___beep____|_____(exceed speed limit here) Impact camera zone_____|
// Expected: Beep signal.
UNIT_TEST(SpeedCameraNotification_AutoAlwaysMode_4)
{
  std::vector<SpeedCameraManagerMode> modes = {SpeedCameraManagerMode::Auto, SpeedCameraManagerMode::Always};
  for (auto const mode : modes)
  {
    RoutingSession routingSession;
    InitRoutingSession({55.65601, 37.53822} /* from */,
                       {55.65760, 37.52312} /* to   */,
                       routingSession,
                       mode);

    {
      double speedKmPH = 100.0;
      ChangePosition({55.65647, 37.53643}, speedKmPH, routingSession);
      TEST(NoCameraFound(routingSession), ());
      TEST(!CheckVoiceNotification(routingSession), ());
      TEST(!CheckBeepSignal(routingSession), ());
    }
    {
      double speedKmPH = 100.0;
      ChangePosition({55.65671, 37.53236}, speedKmPH, routingSession);
      TEST(CheckZone(routingSession, speedKmPH, SpeedCameraManager::Interval::ImpactZone), ());
      TEST(!CheckVoiceNotification(routingSession), ());
      TEST(CheckBeepSignal(routingSession), ());
    }
  }
}

// Mode: Auto/Always
// ____(exceed speed limit here) Notification___|___beep____|_____Impact camera zone_____|
// Expected: Voice notification.
UNIT_TEST(SpeedCameraNotification_AutoAlwaysMode_5)
{
  std::vector<SpeedCameraManagerMode> modes = {SpeedCameraManagerMode::Auto, SpeedCameraManagerMode::Always};
  for (auto const mode : modes)
  {
    RoutingSession routingSession;
    InitRoutingSession({55.76801, 37.59363} /* from */,
                       {55.75947, 37.58484} /* to   */,
                       routingSession,
                       mode);

    // No danger here.
    {
      double speedKmPH = 100.0;
      ChangePosition({55.76766, 37.59260}, speedKmPH, routingSession);
      TEST(NoCameraFound(routingSession), ());
      TEST(!CheckVoiceNotification(routingSession), ());
      TEST(!CheckBeepSignal(routingSession), ());
    }

    // Exceed speed limit before beep zone.
    {
      double speedKmPH = 100.0;
      ChangePosition({55.7660589, 37.5907827}, speedKmPH, routingSession);
      TEST(CheckZone(routingSession, speedKmPH, SpeedCameraManager::Interval::VoiceNotificationZone), ());
      TEST(CheckVoiceNotification(routingSession), ());
      TEST(!CheckBeepSignal(routingSession), ());
    }
  }
}

// Mode: Auto/Always
// ____(exceed speed limit here) Notification___|___(and here) beep____|_____Impact camera zone_____|
// Expected: Voice notification, after it beep signal.
UNIT_TEST(SpeedCameraNotification_AutoAlwaysMode_6)
{
  std::vector<SpeedCameraManagerMode> modes = {SpeedCameraManagerMode::Auto, SpeedCameraManagerMode::Always};
  for (auto const mode : modes)
  {
    RoutingSession routingSession;
    InitRoutingSession({55.76801, 37.59363} /* from */,
                       {55.75947, 37.58484} /* to   */,
                       routingSession,
                       mode);

    // Exceed speed limit before beep zone.
    {
      double speedKmPH = 100.0;
      ChangePosition({55.7660589, 37.5907827}, speedKmPH, routingSession);
      TEST(CheckZone(routingSession, speedKmPH, SpeedCameraManager::Interval::VoiceNotificationZone), ());
      TEST(CheckVoiceNotification(routingSession), ());
      TEST(!CheckBeepSignal(routingSession), ());
    }

    // Need intermediate ChangePosition to calculate passedDistance correctly.
    {
      double speedKmPH = 100.0;
      ChangePosition({55.7656051, 37.5901564}, speedKmPH, routingSession);
      TEST(CheckZone(routingSession, speedKmPH, SpeedCameraManager::Interval::VoiceNotificationZone), ());
      TEST(!CheckVoiceNotification(routingSession), ());
      TEST(!CheckBeepSignal(routingSession), ());
    }

    // Exceed speed limit in beep zone.
    {
      double speedKmPH = 100.0;
      ChangePosition({55.7654092, 37.5898876}, speedKmPH, routingSession);
      TEST(CheckZone(routingSession, speedKmPH, SpeedCameraManager::Interval::BeepSignalZone), ());
      TEST(!CheckVoiceNotification(routingSession), ());
      TEST(CheckBeepSignal(routingSession), ());
    }
  }
}

// Mode: Auto/Always
// ___(exceed speed limit here) Notification___|___(not exceed speed limit here) beep____|____Impact camera zone____|
// We must hear beep signal after voice notification, no matter whether we exceed speed limit or not.
// Expected: Voice notification, after it beep signal.
UNIT_TEST(SpeedCameraNotification_AutoAlwaysMode_7)
{
  std::vector<SpeedCameraManagerMode> modes = {SpeedCameraManagerMode::Auto, SpeedCameraManagerMode::Always};
  for (auto const mode : modes)
  {
    RoutingSession routingSession;
    InitRoutingSession({55.76801, 37.59363} /* from */,
                       {55.75947, 37.58484} /* to   */,
                       routingSession,
                       mode);

    // Exceed speed limit before beep zone.
    {
      double speedKmPH = 100.0;
      ChangePosition({55.7659465, 37.590631}, speedKmPH, routingSession);
      TEST(CheckZone(routingSession, speedKmPH, SpeedCameraManager::Interval::VoiceNotificationZone), ());
      TEST(CheckVoiceNotification(routingSession), ());
      TEST(!CheckBeepSignal(routingSession), ());
    }

    // Need intermediate ChangePosition to calculate passedDistance correctly.
    {
      double speedKmPH = 60.0;
      ChangePosition({55.7657867, 37.5904073}, speedKmPH, routingSession);
      TEST(CheckZone(routingSession, speedKmPH, SpeedCameraManager::Interval::VoiceNotificationZone), ());
      TEST(!CheckVoiceNotification(routingSession), ());
      TEST(!CheckBeepSignal(routingSession), ());
    }

    // No exceed speed limit in beep zone.
    {
      double speedKmPH = 40.0;
      ChangePosition({55.7656548, 37.5902138}, speedKmPH, routingSession);
      TEST(CheckZone(routingSession, speedKmPH, SpeedCameraManager::Interval::BeepSignalZone), ());
      TEST(!CheckVoiceNotification(routingSession), ());
      TEST(CheckBeepSignal(routingSession), ());
    }
  }
}

// Mode: Always
// ____Notification___|___beep____|_____Impact camera zone_____|
// --------------------------------^ - We are here. No exceed speed limit.
//                                     In case |Always| mode we should hear voice notification.
// Expected: Voice notification in |Always| mode.
UNIT_TEST(SpeedCameraNotification_AlwaysMode_1)
{
  {
    RoutingSession routingSession;
    InitRoutingSession({55.76801, 37.59363} /* from */,
                       {55.75947, 37.58484} /* to   */,
                       routingSession,
                       SpeedCameraManagerMode::Always);

    {
      double speedKmPH = 40.0;
      ChangePosition({55.7647619, 37.5890578}, speedKmPH, routingSession);
      TEST(CheckZone(routingSession, speedKmPH, SpeedCameraManager::Interval::ImpactZone), ());
      TEST(CheckVoiceNotification(routingSession), ());
      TEST(!CheckBeepSignal(routingSession), ());
    }
  }
}

// Mode: Auto
// ____Notification___|___beep____|_____Impact camera zone_____|
// --------------------------------^ - We are here. No exceed speed limit.
//                                     In case |Auto| mode we should hear nothing.
// Expected: and nothing in mode: |Auto|.
UNIT_TEST(SpeedCameraNotification_AutoMode_1)
{
  {
    RoutingSession routingSession;
    InitRoutingSession({55.76801, 37.59363} /* from */,
                       {55.75947, 37.58484} /* to   */,
                       routingSession,
                       SpeedCameraManagerMode::Auto);

    {
      double speedKmPH = 40.0;
      ChangePosition({55.7647619, 37.5890578}, speedKmPH, routingSession);
      TEST(NoCameraFound(routingSession), ());
      TEST(!CheckVoiceNotification(routingSession), ());
      TEST(!CheckBeepSignal(routingSession), ());
    }
  }
}

UNIT_TEST(SpeedCameraNotification_NeverMode_1)
{
  {
    RoutingSession routingSession;
    InitRoutingSession({55.76801, 37.59363} /* from */,
                       {55.75947, 37.58484} /* to   */,
                       routingSession,
                       SpeedCameraManagerMode::Never);

    {
      double speedKmPH = 100.0;
      ChangePosition({55.7647619, 37.5890578}, speedKmPH, routingSession);
      TEST(NoCameraFound(routingSession), ());
      TEST(!CheckVoiceNotification(routingSession), ());
      TEST(!CheckBeepSignal(routingSession), ());
    }

    {
      double speedKmPH = 200.0;
      ChangePosition({55.7644126, 37.5886567}, speedKmPH, routingSession);
      TEST(NoCameraFound(routingSession), ());
      TEST(!CheckVoiceNotification(routingSession), ());
      TEST(!CheckBeepSignal(routingSession), ());
    }

    {
      double speedKmPH = 300.0;
      ChangePosition({55.7633558, 37.587675}, speedKmPH, routingSession);
      TEST(NoCameraFound(routingSession), ());
      TEST(!CheckVoiceNotification(routingSession), ());
      TEST(!CheckBeepSignal(routingSession), ());
    }
  }
}
}  // namespace

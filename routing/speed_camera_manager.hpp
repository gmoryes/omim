#include <utility>

#pragma once

#include "platform/location.hpp"

#include "routing/route.hpp"
#include "routing/routing_callbacks.hpp"
#include "routing/speed_camera.hpp"
#include "routing/turns_notification_manager.hpp"

#include "base/assert.hpp"

#include <cstdint>
#include <memory>
#include <queue>
#include <vector>

namespace routing
{
class SpeedCameraManager
{
public:
  enum class Mode
  {
    Auto,
    Always,
    Never
  };

  explicit SpeedCameraManager(turns::sound::NotificationManager & notificationManager)
    : m_notificationManager(notificationManager) {}

  void SetRoute(std::shared_ptr<Route> route) { m_route = std::move(route); }

  void SetSpeedCamShowCallback(SpeedCameraShowCallback && callback)
  {
    m_speedCamShowCallback = std::move(callback);
  }

  void SetSpeedCamClearCallback(SpeedCameraClearCallback && callback)
  {
    m_speedCamClearCallback = std::move(callback);
  }

  bool Enable() const { return m_mode != Mode::Never; }

  void OnLocationPositionChanged(location::GpsInfo const & info)
  {
    if (!Enable())
      return;

    auto const passedDistanceMeters = m_route->GetCurrentDistanceFromBeginMeters();

    // Step 1. Find new cameras and cache them.
    FindCamerasOnRouteAndCache(passedDistanceMeters);

    // Step 2. Process warned cameras.
    while (!m_warnedSpeedCameras.empty())
    {
      auto const & oldestCamera = m_warnedSpeedCameras.front();
      double const distBetweenCameraAndCurrentPos = passedDistanceMeters - oldestCamera.m_distFromBeginMeters;
      if (distBetweenCameraAndCurrentPos < SpeedCameraOnRoute::kInfluenceZoneMeters)
        break;

      m_warnedSpeedCameras.pop();
    }

    // Step 3. Check cached cameras.
    if (!m_cachedSpeedCameras.empty())
    {
      // Do not use reference here, because ProcessCameraWarning() can
      // invalidate |closestSpeedCamera|.
      auto const closestSpeedCamera = m_cachedSpeedCameras.front();
      if (closestSpeedCamera.m_distFromBeginMeters < passedDistanceMeters)
      {
        PassCameraToWarned();
      }
      else
      {
        auto const distanceToCameraMeters = closestSpeedCamera.m_distFromBeginMeters - passedDistanceMeters;
        if (closestSpeedCamera.IsDangerous(distanceToCameraMeters, info.m_speedMpS))
          ProcessCameraWarning();

        if (closestSpeedCamera.NeedShow(distanceToCameraMeters))
          PassCameraToUI(closestSpeedCamera);
      }
    }

    // Step 4. Check UI camera (stop or not stop to highlight it).
    if (m_currentHighlightedSpeedCamera.IsValid())
    {
      auto const distanceToCameraMeters =
        m_currentHighlightedSpeedCamera.m_distFromBeginMeters - passedDistanceMeters;

      if (!m_currentHighlightedSpeedCamera.InInfluenseZone(distanceToCameraMeters))
      {
        m_speedCamClearCallback();
        m_currentHighlightedSpeedCamera.Invalidate();
      }
    }
  }

  void GenerateNotifications(vector<string> & notifications)
  {
    if (!Enable())
      return;

    if (m_makeNotificationAboutSpeedCam)
    {
      notifications.emplace_back(m_notificationManager.GenerateSpeedCameraText());
      m_makeNotificationAboutSpeedCam = false;
    }
  }

  void Reset()
  {
    m_firstNotCheckedSpeedCameraIndex = 1;
    m_makeNotificationAboutSpeedCam = false;
    m_warnedSpeedCameras = std::queue<SpeedCameraOnRoute>();
    m_cachedSpeedCameras = std::queue<SpeedCameraOnRoute>();
  }

  void SetMode(Mode mode) { m_mode = mode; }
  Mode GetMode() const { return m_mode; }

private:
  void FindCamerasOnRouteAndCache(double passedDistanceMeters)
  {
    CHECK(Enable(), ("Speed camera manager is off."));

    auto const & segments = m_route->GetRouteSegments();
    size_t firstNotChecked = m_firstNotCheckedSpeedCameraIndex;
    if (firstNotChecked == segments.size())
      return;

    CHECK_LESS(firstNotChecked, segments.size(), ());

    double distToPrevSegment = segments[firstNotChecked].GetDistFromBeginningMeters();
    double distFromCurPosToLatestCheckedSegmentM = distToPrevSegment - passedDistanceMeters;

    while (firstNotChecked < segments.size() &&
           distFromCurPosToLatestCheckedSegmentM < SpeedCameraOnRoute::kLookAheadDistanceMeters)
    {
      ASSERT_GREATER(firstNotChecked, 0, ());

      auto const & lastSegment = segments[firstNotChecked];
      auto const & prevSegment = segments[firstNotChecked - 1];

      auto const & endPoint = lastSegment.GetJunction().GetPoint();
      auto const & startPoint = prevSegment.GetJunction().GetPoint();
      auto const direction = endPoint - startPoint;

      auto const & speedCamsVector = lastSegment.GetSpeedCams();
      double segmentLength = m_route->GetSegLenMeters(firstNotChecked);

      for (auto const & speedCam : speedCamsVector)
      {
        segmentLength *= speedCam.m_coef;
        m_cachedSpeedCameras.emplace(distToPrevSegment + segmentLength, speedCam.m_maxSpeedKmPH,
                                     startPoint + direction * speedCam.m_coef);
      }

      distToPrevSegment = lastSegment.GetDistFromBeginningMeters();
      distFromCurPosToLatestCheckedSegmentM = distToPrevSegment - passedDistanceMeters;
      ++firstNotChecked;
    }

    m_firstNotCheckedSpeedCameraIndex = firstNotChecked;
  }

  void ProcessCameraWarning()
  {
    PassCameraToWarned();
    m_makeNotificationAboutSpeedCam = true;  // Sound about camera appearing.
  }

  void PassCameraToWarned()
  {
    CHECK(!m_cachedSpeedCameras.empty(), ());
    m_warnedSpeedCameras.push(m_cachedSpeedCameras.front());
    m_cachedSpeedCameras.pop();
  }

  void PassCameraToUI(SpeedCameraOnRoute const & camera)
  {
    // Clear previous speed cam in UI.
    m_speedCamClearCallback();

    m_currentHighlightedSpeedCamera = camera;
    m_speedCamShowCallback(camera.m_position);
  }

private:
  // Queue of speedCams, warnings about which has been pronounced.
  std::queue<SpeedCameraOnRoute> m_warnedSpeedCameras;

  // Queue of speedCams, that we have found, but they are too far, to make warning about them.
  std::queue<SpeedCameraOnRoute> m_cachedSpeedCameras;

  // Info about camera, that is highlighted now.
  SpeedCameraOnRoute m_currentHighlightedSpeedCamera;

  // Flag of doing sound notification about camera on a way.
  bool m_makeNotificationAboutSpeedCam = false;

  size_t m_firstNotCheckedSpeedCameraIndex = 1;
  std::shared_ptr<Route> m_route;
  turns::sound::NotificationManager & m_notificationManager;

  SpeedCameraShowCallback m_speedCamShowCallback = [](m2::PointD const & point) {};
  SpeedCameraClearCallback m_speedCamClearCallback = []() {};

  SpeedCameraManager::Mode m_mode = SpeedCameraManager::Mode::Auto;
};
}  // namespace routing

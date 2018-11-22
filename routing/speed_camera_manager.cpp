#include "routing/speed_camera_manager.hpp"

namespace routing
{
std::string const SpeedCameraManager::kSpeedCamModeKey = "speed_cam_mode";

SpeedCameraManager::SpeedCameraManager(turns::sound::NotificationManager & notificationManager)
  : m_notificationManager(notificationManager)
{
  int mode;
  if (settings::Get(kSpeedCamModeKey, mode))
    m_mode = static_cast<Mode>(mode);
  else
    m_mode = Mode::Auto;
}

void SpeedCameraManager::OnLocationPositionChanged(location::GpsInfo const & info)
{
  if (!Enable())
    return;

  CHECK(m_route, ());

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

void SpeedCameraManager::GenerateNotifications(std::vector<std::string> & notifications)
{
  if (!Enable())
    return;

  if (m_makeNotificationAboutSpeedCam)
  {
    notifications.emplace_back(m_notificationManager.GenerateSpeedCameraText());
    m_makeNotificationAboutSpeedCam = false;
  }
}

void SpeedCameraManager::Reset()
{
  m_firstNotCheckedSpeedCameraIndex = 1;
  m_makeNotificationAboutSpeedCam = false;
  m_warnedSpeedCameras = std::queue<SpeedCameraOnRoute>();
  m_cachedSpeedCameras = std::queue<SpeedCameraOnRoute>();
  m_route = nullptr;
}

void SpeedCameraManager::FindCamerasOnRouteAndCache(double passedDistanceMeters)
{
  CHECK(Enable(), ("Speed camera manager is off."));
  CHECK(m_route, ());

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
}  // namespace routing

#include <utility>

#pragma once


#include "routing/route.hpp"
#include "routing/routing_callbacks.hpp"
#include "routing/speed_camera.hpp"
#include "routing/turns_notification_manager.hpp"

#include "platform/location.hpp"

#include "base/assert.hpp"

#include <cstdint>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace routing
{

class SpeedCameraManager
{
public:
  static std::string const kSpeedCamModeKey;

  enum class Mode
  {
    Auto,
    Always,
    Never
  };

  explicit SpeedCameraManager(turns::sound::NotificationManager & notificationManager);

  void SetRoute(Route * route) { m_route = route; }

  void SetSpeedCamShowCallback(SpeedCameraShowCallback && callback)
  {
    m_speedCamShowCallback = std::move(callback);
  }

  void SetSpeedCamClearCallback(SpeedCameraClearCallback && callback)
  {
    m_speedCamClearCallback = std::move(callback);
  }

  bool Enable() const { return m_mode != Mode::Never; }

  void OnLocationPositionChanged(location::GpsInfo const & info);

  void GenerateNotifications(std::vector<std::string> & notifications);

  void Reset();

  void SetMode(Mode mode)
  {
    m_mode = mode;
    settings::Set(kSpeedCamModeKey, static_cast<int>(mode));
  }

  Mode GetMode() const { return m_mode; }

private:
  void FindCamerasOnRouteAndCache(double passedDistanceMeters);

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
  Route * m_route = nullptr;
  turns::sound::NotificationManager & m_notificationManager;

  SpeedCameraShowCallback m_speedCamShowCallback = [](m2::PointD const & point) {};
  SpeedCameraClearCallback m_speedCamClearCallback = []() {};

  SpeedCameraManager::Mode m_mode = SpeedCameraManager::Mode::Auto;
};
}  // namespace routing

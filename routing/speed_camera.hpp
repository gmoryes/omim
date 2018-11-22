#pragma once

#include "routing/speed_camera_manager.hpp"

#include "geometry/point2d.hpp"

#include <cstdint>
#include <limits>

namespace routing
{
struct SpeedCameraOnRoute
{
  SpeedCameraOnRoute() = default;
  SpeedCameraOnRoute(double distFromBegin, uint8_t maxSpeedKmH, m2::PointD const & position)
    : m_distFromBeginMeters(distFromBegin), m_maxSpeedKmH(maxSpeedKmH), m_position(position)
  {}

  // According to https://en.wikibooks.org/wiki/Physics_Study_Guide/Frictional_coefficients
  // average friction of rubber on asphalt is 0.68
  //
  // According to: https://en.wikipedia.org/wiki/Braking_distance
  // a =- \mu * g, where \mu - average friction of rubber on asphalt.
  static double constexpr kAverageFrictionOfRubberOnAspalt = 0.68;
  static double constexpr kGravitationalAcceleration = 9.80655;

  // Used for calculating distance needed to slow down before a speed camera.
  static double constexpr kAverageAccelerationOfBraking =
    -kAverageFrictionOfRubberOnAspalt * kGravitationalAcceleration; // Meters per second squared.

  static_assert(kAverageAccelerationOfBraking != 0, "");
  static_assert(kAverageAccelerationOfBraking < 0, "AverageAccelerationOfBraking must be negative");

  // We highlight camera in UI if the closest camera is placed in |kShowCameraDistanceM|
  // from us.
  static double constexpr kShowCameraDistanceM = 700.0;
  static double constexpr kInfluenceZoneMeters = 450.0;  // Influence zone of speed camera.
  static double constexpr kDistanceEpsilonMeters = 10.0;
  static double constexpr kDistToReduceSpeedBeforeUnknownCameraM = 50.0;
  static uint8_t constexpr kNoSpeedInfo = std::numeric_limits<uint8_t>::max();

  // Additional time for user about make a decision about slow down.
  // Get from: https://en.wikipedia.org/wiki/Braking_distance
  static double constexpr kTimeForDecision = 2.0;

  // Distance that we use for look ahead to search camera on the route.
  static double constexpr kLookAheadDistanceMeters = 750.0;

  /// \breaf Return true if user must be warned about camera and false otherwise.
  bool IsDangerous(double distanceToCameraMeters, double speedMpS, SpeedCameraManager::Mode mode) const;

  /// \brief Return true if current camera should highlight in UI.
  bool NeedShow(double distanceToCameraMeters) const;

  bool IsValid() const;
  void Invalidate();
  bool InInfluenseZone(double distToCameraMeters) const;

  double m_distFromBeginMeters = 0.0;    // Distance from beginning of route to current camera.
  uint8_t m_maxSpeedKmH = kNoSpeedInfo;  // Maximum speed allowed by the camera.
  m2::PointD m_position = m2::PointD::Zero();
};
}  // namespace routing

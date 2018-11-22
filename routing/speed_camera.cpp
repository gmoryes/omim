#include "routing/speed_camera.hpp"

#include "routing/routing_helpers.hpp"
#include "speed_camera.hpp"


namespace routing
{
bool SpeedCameraOnRoute::IsDangerous(double distanceToCameraMeters, double speedMpS) const
{
  if (distanceToCameraMeters < kInfluenceZoneMeters + kDistanceEpsilonMeters)
    return true;

  if (m_maxSpeedKmH == kNoSpeedInfo)
    return distanceToCameraMeters < kInfluenceZoneMeters + kDistToReduceSpeedBeforeUnknownCameraM;

  double const distToDangerousZone = distanceToCameraMeters - kInfluenceZoneMeters;

  if (speedMpS < routing::KMPH2MPS(m_maxSpeedKmH))
    return false;

  double timeToSlowSpeed =
    (routing::KMPH2MPS(m_maxSpeedKmH) - speedMpS) / kAverageAccelerationOfBraking;

  // Look to: https://en.wikipedia.org/wiki/Acceleration#Uniform_acceleration
  // S = V_0 * t + at^2 / 2, where
  //   V_0 - current speed
  //   a - kAverageAccelerationOfBraking
  double distanceNeedsToSlowDown = timeToSlowSpeed * speedMpS +
                                   (kAverageAccelerationOfBraking * timeToSlowSpeed * timeToSlowSpeed) / 2;
  distanceNeedsToSlowDown += kTimeForDecision * speedMpS;

  if (distToDangerousZone < distanceNeedsToSlowDown + kDistanceEpsilonMeters)
    return true;

  return false;
}

bool SpeedCameraOnRoute::NeedShow(double distanceToCameraMeters) const
{
  return distanceToCameraMeters < kShowCameraDistanceM;
}

bool SpeedCameraOnRoute::IsValid() const
{
  return m_position != m2::PointD::Zero();
}

bool SpeedCameraOnRoute::InInfluenseZone(double distToCameraMeters) const
{
  return std::abs(distToCameraMeters) < kInfluenceZoneMeters;
}

void SpeedCameraOnRoute::Invalidate()
{
  m_position = m2::PointD::Zero();
}
}  // namespace routing

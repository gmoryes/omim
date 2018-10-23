#pragma once

#include "routing/road_point.hpp"
#include "routing/route_weight.hpp"
#include "routing/segment.hpp"

#include "base/assert.hpp"

namespace routing
{
class JointSegment
{
public:
  JointSegment(NumMwmId numMwmId, uint32_t featureId, uint32_t startPointId, uint32_t endPointId)
    : m_featureId(featureId), m_numMwmId(numMwmId)
  {
    CHECK_NOT_EQUAL(startPointId, endPointId, ());

    if (startPointId < endPointId)
    {
      m_forward = true;
      m_firstSegmentId = startPointId;
      m_lastSegmentId = endPointId - 1;
    }
    else
    {
      m_forward = false;
      m_firstSegmentId = startPointId - 1;
      m_lastSegmentId = endPointId;
    }
  }

  void SetWeight(RouteWeight const & weight) { m_weight = weight; }
  uint32_t GetFeatureId() const { return m_featureId; }
  NumMwmId GetMwmId() const { return m_numMwmId; }
  uint32_t GetStartSegmentId() const { return m_firstSegmentId; }
  uint32_t GetEndSegmentId() const { return m_lastSegmentId; }
  bool IsForward() const { return m_forward; }

  uint32_t GetOppositeSegmentId(uint32_t segmentId) const
  {
    ASSERT(m_firstSegmentId == segmentId || m_lastSegmentId == segmentId, ());

    if (segmentId == m_firstSegmentId)
      return m_lastSegmentId;
    return m_firstSegmentId;
  }

  RouteWeight const & GetWeight() const { return m_weight; }

private:
  RouteWeight m_weight = RouteWeight(0.0);
  uint32_t m_featureId;
  uint32_t m_firstSegmentId;
  uint32_t m_lastSegmentId;
  NumMwmId m_numMwmId;
  bool m_forward;
};
}  // namespace routing

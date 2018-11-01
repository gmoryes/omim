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
  JointSegment(Segment const & from, Segment const & to)
  {
    if (from.GetMwmId() == kFakeNumMwmId || to.GetMwmId() == kFakeNumMwmId)
    {
      m_numMwmId = kFakeNumMwmId;
    }
    else
    {
      CHECK_EQUAL(from.GetMwmId(), to.GetMwmId(), ("Different mwmIds in segments for JointSegment"));
      m_numMwmId = from.GetMwmId();

      CHECK_EQUAL(from.IsForward(), to.IsForward(), ("Different forward in segments for JointSegment"));
      m_forward = from.IsForward();
    }
  }

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

//  JointSegment(RouteWeight const & weight,  NumMwmId numMwmId, uint32_t featureId, uint32_t firstSegmentId,
//               uint32_t lastSegmentId, bool forward)
//    : m_weight(weight), m_featureId(featureId), m_firstSegmentId(firstSegmentId),
//      m_lastSegmentId(lastSegmentId), m_numMwmId(numMwmId), m_forward(forward) {}

//  void SetWeight(RouteWeight const & weight) { m_weight = weight; }
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

  template <typename Sink>
  static void Serialize(Sink & sink, JointSegment const & jointSegment)
  {
    RouteWeight::Serialize(sink, jointSegment.GetWeight());

    uint32_t featureId = jointSegment.GetFeatureId();
    sink.Write(&featureId, sizeof(featureId));

    uint32_t firstSegmentId = jointSegment.GetStartSegmentId();
    sink.Write(&firstSegmentId, sizeof(firstSegmentId));

    uint32_t lastSegmentId = jointSegment.GetEndSegmentId();
    sink.Write(&lastSegmentId, sizeof(lastSegmentId));

    bool forward = jointSegment.IsForward();
    sink.Write(&forward, sizeof(forward));
  }

  template <typename Source>
  static JointSegment Deserialize(Source & source, NumMwmId m_numMwmId)
  {
    RouteWeight routeWeight = RouteWeight::Deserialize(source);

    uint32_t featureId;
    source.Read(&featureId, sizeof(featureId));

    uint32_t firstSegmentId;
    source.Read(&firstSegmentId, sizeof(firstSegmentId));

    uint32_t lastSegmentId;
    source.Read(&lastSegmentId, sizeof(lastSegmentId));

    bool forward;
    source.Read(&forward, sizeof(forward));

    return {routeWeight, m_numMwmId, featureId, firstSegmentId, lastSegmentId, forward};
  }

  bool operator< (JointSegment const & jointSegment) const
  {
    if (jointSegment.GetWeight() != m_weight)
      return jointSegment.GetWeight() < m_weight;

    if (jointSegment.GetFeatureId() != m_featureId)
      return jointSegment.GetFeatureId() < m_featureId;

    if (jointSegment.GetStartSegmentId() != m_firstSegmentId)
      return jointSegment.GetStartSegmentId() < m_firstSegmentId;

    if (jointSegment.GetEndSegmentId() != m_lastSegmentId)
      return jointSegment.GetEndSegmentId() < m_lastSegmentId;

    if (jointSegment.GetMwmId() != m_numMwmId)
      return jointSegment.GetMwmId() < m_numMwmId;

    return jointSegment.IsForward() < m_forward;
  }

private:
  RouteWeight m_weight = RouteWeight(0.0);
  uint32_t m_featureId;
  uint32_t m_firstSegmentId;
  uint32_t m_lastSegmentId;
  NumMwmId m_numMwmId;
  bool m_forward;
};

class JointEdge
{
public:
  JointEdge(JointSegment const & target, RouteWeight const & weight) : m_target(target), m_weight(weight) {}

  JointSegment const & GetTarget() const { return m_target; }
  RouteWeight const & GetWeight() const { return m_weight; }

private:
  JointSegment m_target;
  RouteWeight m_weight;
};
}  // namespace routing

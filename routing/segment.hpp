#pragma once

#include "routing/fake_feature_ids.hpp"
#include "routing/road_point.hpp"
#include "routing/route_weight.hpp"

#include "routing_common/num_mwm_id.hpp"

#include <cstdint>
#include <functional>
#include <sstream>
#include <string>

namespace routing
{
// This is directed road segment used as vertex in index graph.
//
// You can imagine the segment as a material arrow.
// Head of each arrow is connected to fletchings of next arrows with invisible links:
// these are the edges of the graph.
//
// Position of the segment is a position of the arrowhead: GetPointId(true).
// This position is used in heuristic and edges weight calculations.
class Segment final
{
public:
  Segment() = default;

  constexpr Segment(NumMwmId mwmId, uint32_t featureId, uint32_t segmentIdx, bool forward)
    : m_featureId(featureId), m_segmentIdx(segmentIdx), m_mwmId(mwmId), m_forward(forward)
  {
  }

  NumMwmId GetMwmId() const { return m_mwmId; }
  uint32_t GetFeatureId() const { return m_featureId; }
  uint32_t GetSegmentIdx() const { return m_segmentIdx; }
  bool IsForward() const { return m_forward; }

  uint32_t GetPointId(bool front) const
  {
    return m_forward == front ? m_segmentIdx + 1 : m_segmentIdx;
  }

  uint32_t GetMinPointId() const { return m_segmentIdx; }
  uint32_t GetMaxPointId() const { return m_segmentIdx + 1; }

  RoadPoint GetRoadPoint(bool front) const { return RoadPoint(m_featureId, GetPointId(front)); }

  bool operator<(Segment const & seg) const
  {
    if (m_featureId != seg.m_featureId)
      return m_featureId < seg.m_featureId;

    if (m_segmentIdx != seg.m_segmentIdx)
      return m_segmentIdx < seg.m_segmentIdx;

    if (m_mwmId != seg.m_mwmId)
      return m_mwmId < seg.m_mwmId;

    return m_forward < seg.m_forward;
  }

  bool operator==(Segment const & seg) const
  {
    return m_featureId == seg.m_featureId && m_segmentIdx == seg.m_segmentIdx &&
           m_mwmId == seg.m_mwmId && m_forward == seg.m_forward;
  }

  bool operator!=(Segment const & seg) const { return !(*this == seg); }

  bool IsInverse(Segment const & seg) const
  {
    return m_featureId == seg.m_featureId && m_segmentIdx == seg.m_segmentIdx &&
           m_mwmId == seg.m_mwmId && m_forward != seg.m_forward;
  }

  bool IsRealSegment() const
  {
    return m_mwmId != kFakeNumMwmId && !FakeFeatureIds::IsTransitFeature(m_featureId);
  }

  Segment Next(bool forward) const
  {
    uint32_t nextSegmentId = forward ? m_segmentIdx + 1 : m_segmentIdx - 1;
    return {m_mwmId, m_featureId, nextSegmentId, m_forward};
  }

  struct Hash // 66282082795521
  {
    size_t operator() (Segment const & segment) const
    {
      uint32_t featureId = segment.GetFeatureId();
      uint32_t segmentId = segment.GetSegmentIdx();
      bool forward = segment.IsForward();

      static uint32_t constexpr kFirstBit = 1U << 31;
      ASSERT_EQUAL(segmentId & kFirstBit, 0, ());
      if (forward)
        segmentId |= kFirstBit;

      return std::hash<uint64_t>()(
        (static_cast<uint64_t>(featureId) << 32) + static_cast<uint64_t>(segmentId));
    }
  };

private:
  uint32_t m_featureId = 0;
  uint32_t m_segmentIdx = 0;
  NumMwmId m_mwmId = kFakeNumMwmId;
  bool m_forward = false;
};

class SegmentEdge final
{
public:
  SegmentEdge() = default;
  SegmentEdge(Segment const & target, RouteWeight const & weight)
    : m_target(target), m_weight(weight)
  {}

  Segment const & GetTarget() const { return m_target; }
  Segment & GetTarget() { return m_target; }
  RouteWeight const & GetWeight() const { return m_weight; }

  bool operator==(SegmentEdge const & edge) const
  {
    return m_target == edge.m_target && m_weight == edge.m_weight;
  }

  bool operator<(SegmentEdge const & edge) const
  {
    if (m_target != edge.m_target)
      return m_target < edge.m_target;
    return m_weight < edge.m_weight;
  }

  SegmentEdge & operator=(SegmentEdge const & rhs)
  {
    m_target = rhs.m_target;
    m_weight = rhs.m_weight;

    return *this;
  }

private:
  // Target is vertex going to for outgoing edges, vertex going from for ingoing edges.
  Segment m_target;
  RouteWeight m_weight;
};

inline std::string DebugPrint(Segment const & segment)
{
  std::ostringstream out;
  out << "Segment(" << segment.GetMwmId() << ", " << segment.GetFeatureId() << ", "
      << segment.GetSegmentIdx() << ", " << segment.IsForward() << ")";
  return out.str();
}

inline std::string DebugPrint(SegmentEdge const & edge)
{
  std::ostringstream out;
  out << "Edge(" << DebugPrint(edge.GetTarget()) << ", " << edge.GetWeight() << ")";
  return out.str();
}
}  // namespace routing

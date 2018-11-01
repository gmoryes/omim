#include "routing/joint_graph.hpp"

#include "routing/joint.hpp"

#include "base/assert.hpp"

#include <cstdint>
#include <utility>
#include <vector>

namespace routing
{
void JointGraph::GetOutgoingEdgesList(Segment const & segment, vector<SegmentEdge> & edges)
{
  GetEdgesList(segment, true /* IsOutgoing */, edges);
}

void JointGraph::GetIngoingEdgesList(Segment const & segment, vector<SegmentEdge> & edges)
{
  GetEdgesList(segment, false /* IsOutgoing */, edges);
}

RouteWeight JointGraph::HeuristicCostEstimate(Segment const & from, Segment const & to)
{
  return RouteWeight(
    m_indexGraph.GetEstimator().CalcHeuristic(
      m_indexGraph.GetPoint(from, true /* front */),
      m_indexGraph.GetPoint(to, true /* front */)));
}

bool JointGraph::CheckAndProcessTransitFeature(Segment const & segment, RouteWeight const & weight,
                                               bool isOutgoing, vector<SegmentEdge> & edges)
{
  bool isTransitFeature = m_singleVehicleWorldGraph.GetCrossMwmGraph().
    IsFeatureTransit(segment.GetMwmId(), segment.GetFeatureId());

  if (!isTransitFeature)
    return false;

  NumMwmId outerNumMwmId;
  uint32_t outerMwmFeatureId;
  std::tie(outerNumMwmId, outerMwmFeatureId) = m_singleVehicleWorldGraph.GetCrossMwmGraph().
    GetTwinFeature(segment, isOutgoing);

  // Case of twins absent
  if (outerMwmFeatureId == std::numeric_limits<uint32_t>::max())
    return false;

  edges.emplace_back(
    Segment(outerNumMwmId, outerMwmFeatureId, segment.GetSegmentIdx(), segment.IsForward()),
    weight);

  return true;
}

void JointGraph::GetEdgeListBoost(Segment const & from, bool isOutgoing, std::vector<SegmentEdge> & edges)
{
  std::vector<SegmentEdge> segmentEdges(edges);
  m_indexGraph.GetEdgeList(from, isOutgoing, segmentEdges);

  auto const & jointIndex = m_indexGraph.GetJointSegmentIndex();

  for (auto const & segmentEdge : segmentEdges)
  {
    auto segment = segmentEdge.GetTarget();

    // Case of transit feature
    if (CheckAndProcessTransitFeature(segment, segmentEdge.GetWeight(), isOutgoing, edges))
      continue;

    auto it = jointIndex.find(segment);
    if (it == jointIndex.cend() || jointIndex.find(from) == jointIndex.cend())
    {
      // Can not find such joint
      edges.emplace_back(segmentEdge);
      continue;
    }

    edges.emplace_back(
      Segment(it->second.GetMwmId(), it->second.GetFeatureId(),
              it->second.GetOppositeSegmentId(segment.GetSegmentIdx()),
              it->second.IsForward()),
      segmentEdge.GetWeight() + it->second.GetWeight());
  }
}

void JointGraph::GetEdgesList(Segment const & from, bool isOutgoing, vector<JointEdge> & edges)
{
  RoadIndex const & roadIndex = m_indexGraph.GetRoadIndex();

  std::vector<SegmentEdge> segmentEdges;
  IndexGraphStarter::GetEdgesList(from, isOutgoing, segmentEdges);

  for (auto const & segmentEdge : segmentEdges)
  {
    auto segment = segmentEdge.GetTarget();

    // Case of transit feature
    if (CheckAndProcessTransitFeature(segment, segmentEdge.GetWeight(), isOutgoing, edges))
      continue;

    auto const featureId = segment.GetFeatureId();
    if (!roadIndex.IsRoad(featureId))
      continue;
    RoadJointIds const & roadJointIds = roadIndex.GetRoad(featureId);

    uint32_t startPointId = segment.GetPointId(!isOutgoing /* front */);
    uint32_t const pointsNumber = m_indexGraph.GetGeometry().GetRoad(featureId).GetPointsCount();
    CHECK(startPointId < pointsNumber, ());

    uint32_t endPointId;
    std::tie(std::ignore, endPointId) = roadJointIds.FindNeighbor(startPointId,
                                                                  !(segment.IsForward() ^ isOutgoing),
                                                                  pointsNumber);

    GetSegmentEdge(from, segment, endPointId, isOutgoing, edges);
  }
}

bool JointGraph::ProcessFakeEdge(Segment const & prev, Segment const & current, bool isOutgoing,
                                 SegmentEdge & edge)
{
  return IndexGraphStarter::AddFakeEdgeForSegment(prev, current, isOutgoing, edge);
}

void JointGraph::GetSegmentEdge(Segment const & prevSegment, Segment const & firstNext, uint32_t lastPointId,
                                bool isOutgoing, std::vector<SegmentEdge> & edges)
{
  uint32_t currentPointId = firstNext.GetPointId(!isOutgoing /* front */);
  CHECK_NOT_EQUAL(currentPointId, lastPointId, ());

  auto const increment = [currentPointId, lastPointId](uint32_t const pointId) {
    return currentPointId < lastPointId ? pointId + 1 : pointId - 1;
  };

  RouteWeight summaryWeight;

  bool forward = currentPointId < lastPointId;
  Segment current = firstNext;
  Segment prev = prevSegment;
  SegmentEdge edge;

  do {
    if (m_indexGraph.GetNeighboringEdge(prev, current, isOutgoing, edge)) // Access ok
    {
      if (ProcessFakeEdge(prev, current, isOutgoing, edge))
      {
        if (prevSegment != prev)
        {
          edges.emplace_back(prev, summaryWeight);
          return;
        }

        summaryWeight += edge.GetWeight();
        edges.emplace_back(edge.GetTarget(), summaryWeight);
        return;
      }

      summaryWeight += edge.GetWeight();
    }
    else
      return;

    prev = current;
    current = current.Next(forward);

    currentPointId = increment(currentPointId);
  } while (currentPointId != lastPointId);

  edges.emplace_back(prev, summaryWeight);
}
}  // namespace routing

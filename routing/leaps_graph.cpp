#include "routing/leaps_graph.hpp"

#include "routing_common/num_mwm_id.hpp"

#include "base/assert.hpp"

#include <set>

namespace routing
{
LeapsGraph::LeapsGraph(IndexGraphStarter & starter) : m_starter(starter) {}

void LeapsGraph::GetOutgoingEdgesList(Segment const & Segment,
                                      std::vector<SegmentEdge> & edges)
{
  GetEdgesList(Segment, true /* isOutgoing */, edges);
}

void LeapsGraph::GetIngoingEdgesList(Segment const & Segment,
                                     std::vector<SegmentEdge> & edges)
{
  GetEdgesList(Segment, false /* isOutgoing */, edges);
}

RouteWeight LeapsGraph::HeuristicCostEstimate(Segment const & from, Segment const & to)
{
  // Can not use two const & to m2::PointD at the same moment.
  return m_starter.GetGraph().HeuristicCostEstimate(GetPoint(from, false /* isEnter */),
                                                    GetPoint(to, false /* isEnter */));
}

void LeapsGraph::GetEdgesList(Segment const & segment, bool isOutgoing,
                              std::vector<SegmentEdge> & edges)
{
  if (segment.IsRealSegment() && m_starter.IsRoutingOptionsGood(segment))
    return;

  edges.clear();

  if (segment == m_starter.GetStartSegment())
    return GetEdgesListForStart(segment, isOutgoing, edges);

  // Edge from finish mwm enter to finish.
  if (m_starter.GetFinishEnding().OverlapsWithMwm(segment.GetMwmId()))
    return GetEdgesListForFinish(segment, isOutgoing, edges);

  auto & crossMwmGraph = m_starter.GetGraph().GetCrossMwmGraph();
  if (crossMwmGraph.IsTransition(segment, isOutgoing))
    return GetEdgesForTwins(segment, isOutgoing, edges);
  else
    return crossMwmGraph.GetOutgoingEdgeList(segment, edges);
}

void LeapsGraph::GetEdgesForTwins(Segment const & segment, bool isOutgoing,
                                  std::vector<SegmentEdge> & edges)
{
  std::vector<Segment> twins;
  m_starter.GetGraph().GetTwinsInner(segment, isOutgoing, twins);
  for (auto const & twin : twins)
  {
    if (isOutgoing)
    {
      edges.emplace_back(twin, RouteWeight(0.0));
    }
    else
    {
      auto const it = m_weightCacheForBackwardWave.find(segment);
      CHECK(it != m_weightCacheForBackwardWave.cend(), ());

      edges.emplace_back(twin, it->second);
      m_weightCacheForBackwardWave.erase(it);
    }
  }
}

void LeapsGraph::GetEdgesListForStart(Segment const & segment, bool isOutgoing,
                                      std::vector<SegmentEdge> & edges)
{
  auto const & segmentPoint = GetPoint(segment, true /* front */);
  std::set<NumMwmId> seen;
  for (auto const mwmId : m_starter.GetStartEnding().m_mwmIds)
  {
    if (seen.insert(mwmId).second)
    {
      // Connect start to all exits (|isEnter| == false).
      for (auto const & transition : m_starter.GetGraph().GetTransitions(mwmId, false /* isEnter */))
      {
        auto const & transitionFrontPoint = GetPoint(transition, true /* front */);
        auto const weight = m_starter.GetGraph().CalcLeapWeight(segmentPoint, transitionFrontPoint);
        edges.emplace_back(transition, weight);
      }
    }
  }
}

void LeapsGraph::GetEdgesListForFinish(Segment const & segment, bool isOutgoing,
                                       std::vector<SegmentEdge> & edges)
{
  auto const & segmentPoint = GetPoint(segment, true /* front */);
  edges.emplace_back(m_starter.GetFinishSegment(),
                     m_starter.GetGraph().CalcLeapWeight(
                         segmentPoint, GetPoint(m_starter.GetFinishSegment(), true /* front */)));
}

m2::PointD const & LeapsGraph::GetPoint(Segment const & segment, bool front)
{
  return m_starter.GetPoint(segment, front);
}

RouteWeight LeapsGraph::GetAStarWeightEpsilon()
{
  return m_starter.GetAStarWeightEpsilon();
}
}  // namespace routing

#include "routing/leaps_graph.hpp"

#include "routing_common/num_mwm_id.hpp"

#include <set>

namespace routing
{
LeapsGraph::LeapsGraph(IndexGraphStarter & starter) : m_starter(starter) {}

void LeapsGraph::GetOutgoingEdgesList(LeapSegment const & segment, std::vector<LeapEdge> & edges)
{
  GetEdgesList(segment, true /* isOutgoing */, edges);
}

void LeapsGraph::GetIngoingEdgesList(LeapSegment const & segment, std::vector<LeapEdge> & edges)
{
  GetEdgesList(segment, false /* isOutgoing */, edges);
}

RouteWeight LeapsGraph::HeuristicCostEstimate(LeapSegment const & from, LeapSegment const & to)
{
  return m_starter.HeuristicCostEstimate(from.GetGate(false /* isEnter */), to.GetGate(false /* isEnter */));
}

void LeapsGraph::GetEdgesList(LeapSegment const & segment, bool isOutgoing, std::vector<LeapEdge> & edges)
{
  // Ingoing edges listing is not supported in LeapsOnly mode because we do not have enough
  // information to calculate |segment| weight. See https://jira.mail.ru/browse/MAPSME-5743 for
  // details.
  CHECK(isOutgoing, ("Ingoing edges listing is not supported in LeapsOnly mode."));

  edges.clear();

  if (segment.IsRealSegment() && !m_starter.IsRoutingOptionsGood(segment))
    return;

  if (segment == m_starter.GetStartSegment())
  {
    GetEdgesListForStart(segment, isOutgoing, edges);
    return;
  }

  // Edge from finish mwm enter to finish.
  if (m_starter.GetFinishEnding().OverlapsWithMwm(segment.GetMwmId()))
  {
    GetEdgesListForFinish(segment, isOutgoing, edges);
    return;
  }

  auto & crossMwmGraph = m_starter.GetGraph().GetCrossMwmGraph();
  if (crossMwmGraph.IsTransition(segment, isOutgoing))
  {
    std::vector<Segment> twins;
    m_starter.GetGraph().GetTwinsInner(segment, isOutgoing, twins);
    for (auto const & twin : twins)
    {
      // Weight is usually zero because twins correspond the same feature
      // in different mwms. But if we have mwms with different versions and a feature
      // was moved in one of them the weight is greater than zero.
      edges.emplace_back(twin, HeuristicCostEstimate(segment, twin));
    }
  }
  else
  {
    crossMwmGraph.GetOutgoingEdgeList(segment, edges);
  }
}

void LeapsGraph::GetEdgesListForStart(LeapSegment const & segment, bool isOutgoing,
                                      std::vector<LeapEdge> & edges)
{
  auto const & segmentPoint = GetPoint(segment, true /* front */);
  std::set<NumMwmId> seen;
  for (auto const mwmId : m_starter.GetStartEnding().m_mwmIds)
  {
    if (seen.insert(mwmId).second)
    {
      // Connect start to all exits (|isEnter| == false).
      for (auto const & s : m_starter.GetGraph().GetTransitions(mwmId, false /* isEnter */))
      {
        edges.emplace_back(
            s, m_starter.GetGraph().CalcLeapWeight(segmentPoint, GetPoint(s, true /* front */)));
      }
    }
  }
}

void LeapsGraph::GetEdgesListForFinish(LeapSegment const & segment, bool isOutgoing,
                                       std::vector<LeapEdge> & edges)
{
  auto const & segmentPoint = GetPoint(segment, true /* front */);
  edges.emplace_back(m_starter.GetFinishSegment(),
                     m_starter.GetGraph().CalcLeapWeight(
                         segmentPoint, GetPoint(m_starter.GetFinishSegment(), true /* front */)));
}

m2::PointD const & LeapsGraph::GetPoint(LeapSegment const & segment, bool front) const
{
  return m_starter.GetPoint(segment, front);
}
}  // namespace routing

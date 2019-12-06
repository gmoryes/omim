#include "routing/leaps_graph.hpp"

#include "routing_common/num_mwm_id.hpp"

#include "base/assert.hpp"

#include <set>

namespace routing
{
// static
LeapSegment const LeapsGraph::kStart = LeapSegment(kFakeNumMwmId, kStartId, kNoGate);
// static
LeapSegment const LeapsGraph::kFinish = LeapSegment(kFakeNumMwmId, kNoGate, kFinishId);

LeapsGraph::LeapsGraph(IndexGraphStarter & starter) : m_starter(starter)
{
  m_startPoint = m_starter.GetPoint(m_starter.GetStartSegment(), true /* front */);
  m_finishPoint = m_starter.GetPoint(m_starter.GetFinishSegment(), true /* front */);
  m_startSegment = m_starter.GetStartSegment();
  m_finishSegment = m_starter.GetFinishSegment();
  m_weightCacheForBackwardWave.emplace(kFinish, RouteWeight(0.0));
}

void LeapsGraph::GetOutgoingEdgesList(LeapSegment const & Segment,
                                      std::vector<LeapEdge> & edges)
{
  GetEdgesList(Segment, true /* isOutgoing */, edges);
}

void LeapsGraph::GetIngoingEdgesList(LeapSegment const & Segment,
                                     std::vector<LeapEdge> & edges)
{
  GetEdgesList(Segment, false /* isOutgoing */, edges);
}

RouteWeight LeapsGraph::HeuristicCostEstimate(LeapSegment const & from, LeapSegment const & to)
{
  // Can not use two const & to m2::PointD at the same moment.
  ASSERT(to == kStart || to == kFinish, ());
  bool const toFinish = to == kFinish;
  auto const & fromPoint = GetPoint(from, true /* isExit */);
  auto const & toPoint = toFinish ? m_finishPoint : m_startPoint;
  return m_starter.GetGraph().HeuristicCostEstimate(fromPoint, toPoint);
}

void LeapsGraph::GetEdgesList(LeapSegment const & leapSegment, bool isOutgoing,
                              std::vector<LeapEdge> & edges)
{
  auto const point = mercator::ToLatLon(GetPoint(leapSegment, isOutgoing));
  auto const debug = mercator::FromLatLon(59.938734499999988259, 30.316239599999999399);
  if (base::AlmostEqualAbs(GetPoint(leapSegment, true), debug, 1e-5))
  {
    int asd = 123;
    (void)asd;
  }
  std::ofstream points("/tmp/points", std::ofstream::app);
  points << std::setprecision(20);
  points << point.m_lat << " " << point.m_lon << std::endl;

  edges.clear();

  if (leapSegment == kStart)
    return GetEdgesListFromStart(leapSegment, isOutgoing, edges);

  if (leapSegment == kFinish)
    return GetEdgesListToFinish(leapSegment, isOutgoing, edges);

  if ((isOutgoing && leapSegment.GetGateId(false /* isEnter */) == kFinishId) ||
      (!isOutgoing && leapSegment.GetGateId(true /* isEnter */) == kStartId))
  {
    return GetLastEdge(leapSegment, isOutgoing, edges);
  }

  bool const isEnter = !isOutgoing;
  auto const & segment = GetSegment(leapSegment, isEnter);

  if (segment.IsRealSegment() && !m_starter.IsRoutingOptionsGood(segment))
    return;

  // An edge from start to start mwm exit.
  if (!isOutgoing && m_starter.GetStartEnding().OverlapsWithMwm(segment.GetMwmId()))
    return GetEdgesListFromStart(leapSegment, isOutgoing, edges);

  // Edge from finish mwm enter to finish.
  if (isOutgoing && m_starter.GetFinishEnding().OverlapsWithMwm(segment.GetMwmId()))
    return GetEdgesListToFinish(leapSegment, isOutgoing, edges);

  auto & crossMwmGraph = m_starter.GetGraph().GetCrossMwmGraph();
  ASSERT(crossMwmGraph.IsTransition(segment, isOutgoing), ());

  std::vector<Segment> twins;
  m_starter.GetGraph().GetTwinsInner(segment, isOutgoing, twins);

  LOG(LINFO, ("twin to:", leapSegment.GetGateId(!isOutgoing), segment));
  for (auto const & twin : twins)
  {
    LOG(LINFO, ("twin:", twin));
    if (isOutgoing)
    {
      return crossMwmGraph.GetOutgoingEdgeList(twin, edges);
    }
    else
    {
      crossMwmGraph.GetIngoingEdgeList(twin, edges);
      auto const weight = GetWeightForBackwardWave(leapSegment);
      for (auto & edge : edges)
      {
        LOG(LINFO, (leapSegment, "-->", edge.GetTarget()));
        bool inserted;

        std::tie(std::ignore, inserted) =
            m_weightCacheForBackwardWave.emplace(edge.GetTarget(), edge.GetWeight());

//        CHECK(inserted, ("Already exists:", edge.GetTarget()));
        edge.SetWeight(weight);
      }
    }
  }
}

void LeapsGraph::GetEdgesListFromStart(LeapSegment const & leapSegment, bool isOutgoing,
                                       std::vector<LeapEdge> & edges)
{
  RouteWeight outgoingEdgeWeight;
  if (!isOutgoing)
    outgoingEdgeWeight = GetWeightForBackwardWave(leapSegment);

  for (auto const mwmId : m_starter.GetStartEnding().m_mwmIds)
  {
    // Connect start to all exits (|isEnter| == false).
    auto const & exits = m_starter.GetGraph().GetTransitions(mwmId, false /* isEnter */);

    for (uint32_t exitId = 0; exitId < static_cast<uint32_t>(exits.size()); ++exitId)
    {
      auto const & exit = exits[exitId];
      auto const & exitFrontPoint = m_starter.GetPoint(exit, true /* front */);
      auto const weight = m_starter.GetGraph().CalcLeapWeight(m_startPoint, exitFrontPoint);

      auto const target = LeapSegment(exit.GetMwmId(), kStartId, exitId);
      if (isOutgoing)
      {
        edges.emplace_back(target, weight);
      }
      else
      {
        m_weightCacheForBackwardWave.emplace(target, weight);
        edges.emplace_back(target, outgoingEdgeWeight);
      }
    }
  }
}

void LeapsGraph::GetEdgesListToFinish(LeapSegment const & leapSegment, bool isOutgoing,
                                      std::vector<LeapEdge> & edges)
{
  for (auto const mwmId : m_starter.GetFinishEnding().m_mwmIds)
  {
    // Connect finish to all enters (|isEnter| == true).
    auto const & enters = m_starter.GetGraph().GetTransitions(mwmId, true /* isEnter */);
    for (uint32_t enterId = 0; enterId < static_cast<uint32_t>(enters.size()); ++enterId)
    {
      auto const & enter = enters[enterId];
      auto const & enterFrontPoint = m_starter.GetPoint(enter, true /* front */);
      auto const weight = m_starter.GetGraph().CalcLeapWeight(enterFrontPoint, m_finishPoint);

      auto const target = LeapSegment(enter.GetMwmId(), enterId, kFinishId);
      if (isOutgoing)
      {
        edges.emplace_back(target, weight);
      }
      else
      {
        m_weightCacheForBackwardWave.emplace(target, weight);
        // Weight from |kNoGate -> kFinish| to |enterId -> kFinish| is zero.
        edges.emplace_back(target, RouteWeight(0.0));
      }
    }
  }
}

RouteWeight LeapsGraph::GetWeightForBackwardWave(LeapSegment const & leapSegment)
{
  auto const it = m_weightCacheForBackwardWave.find(leapSegment);
  CHECK(it != m_weightCacheForBackwardWave.cend(), ());
  auto const result = it->second;
  m_weightCacheForBackwardWave.erase(it);
  return result;
}

void LeapsGraph::GetLastEdge(LeapSegment const & leapSegment, bool isOutgoing,
                             std::vector<LeapEdge> & edges)
{
  if (isOutgoing)
  {
    edges.emplace_back(kFinish, RouteWeight(0.0));
  }
  else
  {
    auto const weight = GetWeightForBackwardWave(leapSegment);
    edges.emplace_back(kStart, weight);
  }
}

Segment const & LeapsGraph::GetSegment(LeapSegment const & leapSegment, bool isEnter)
{
  if (leapSegment == kStart || leapSegment.GetGateId(isEnter) == kStartId)
    return m_startSegment;

  if (leapSegment == kFinish || leapSegment.GetGateId(isEnter) == kFinishId)
    return m_finishSegment;

  auto & crossMwmGraph = m_starter.GetGraph().GetCrossMwmGraph();
  auto const & transitions = crossMwmGraph.GetTransitions(leapSegment.GetMwmId(), isEnter);
  CHECK_LESS(leapSegment.GetGateId(isEnter), transitions.size(), ());
  return transitions[leapSegment.GetGateId(isEnter)];
}

m2::PointD const & LeapsGraph::GetPoint(LeapSegment const & leapSegment, bool isExit)
{
  bool const isEnter = !isExit;
  auto const & segment = GetSegment(leapSegment, isEnter);
  return m_starter.GetPoint(segment, true /* front */);
}

RouteWeight LeapsGraph::GetAStarWeightEpsilon()
{
  return m_starter.GetAStarWeightEpsilon();
}

LeapSegment const & LeapsGraph::GetStartSegment() const
{
  return kStart;
}

LeapSegment const & LeapsGraph::GetFinishSegment() const
{
  return kFinish;
}
}  // namespace routing

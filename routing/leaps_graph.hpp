#pragma once

#include "routing/base/astar_graph.hpp"

#include "routing/index_graph_starter.hpp"
#include "routing/leap_segment.hpp"
#include "routing/route_weight.hpp"
#include "routing/segment.hpp"

#include <vector>

namespace routing
{
class LeapsGraph : public AStarGraph<LeapSegment, LeapEdge, RouteWeight>
{
public:
  explicit LeapsGraph(IndexGraphStarter & starter);

  // AStarGraph overrides:
  // @{
  void GetOutgoingEdgesList(LeapSegment const & segment, std::vector<LeapEdge> & edges) override;
  void GetIngoingEdgesList(LeapSegment const & segment, std::vector<LeapEdge> & edges) override;
  RouteWeight HeuristicCostEstimate(LeapSegment const & from, LeapSegment const & to) override;
  RouteWeight GetAStarWeightEpsilon() override;
  // @}

  m2::PointD const & GetPoint(LeapSegment const & segment, bool isEnter);
  LeapSegment const & GetStartSegment() const;
  LeapSegment const & GetFinishSegment() const;

private:
  static uint32_t constexpr kStartId = std::numeric_limits<uint32_t>::max();
  static uint32_t constexpr kFinishId = std::numeric_limits<uint32_t>::max() - 1;
  static uint32_t constexpr kNoGate = std::numeric_limits<uint32_t>::max() - 2;

  void GetEdgesList(LeapSegment const & segment, bool isOutgoing, std::vector<LeapEdge> & edges);

  void GetEdgesListFromStart(LeapSegment const & segment, bool isOutgoing,
                             std::vector<LeapEdge> & edges);
  void GetEdgesListToFinish(LeapSegment const & segment, bool isOutgoing,
                             std::vector<LeapEdge> & edges);

  void GetEdgesForTwins(LeapSegment const & segment, bool isOutgoing, std::vector<LeapEdge> & edges);

  Segment const & GetSegment(LeapSegment const & leapSegment, bool isEnter);
  void GetLastEdge(LeapSegment const & leapSegment, bool isOutgoing, std::vector<LeapEdge> & edges);
  RouteWeight GetWeightForBackwardWave(LeapSegment const & leapSegment);
  bool IsLeapFake(LeapSegment const & leapSegment) const;

  static LeapSegment const kStart;
  static LeapSegment const kFinish;

  m2::PointD m_startPoint;
  m2::PointD m_finishPoint;

  Segment m_startSegment;
  Segment m_finishSegment;

  IndexGraphStarter & m_starter;
  std::map<LeapSegment, RouteWeight> m_weightCacheForBackwardWave;
};
}  // namespace

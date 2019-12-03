#pragma once

#include "routing/base/astar_graph.hpp"

#include "routing/index_graph_starter.hpp"
#include "routing/route_weight.hpp"
#include "routing/segment.hpp"

#include <vector>

namespace routing
{
class LeapsGraph : public AStarGraph<Segment, SegmentEdge, RouteWeight>
{
public:
  explicit LeapsGraph(IndexGraphStarter & starter);

  // AStarGraph overrides:
  // @{
  void GetOutgoingEdgesList(Segment const & segment, std::vector<SegmentEdge> & edges) override;
  void GetIngoingEdgesList(Segment const & segment, std::vector<SegmentEdge> & edges) override;
  RouteWeight HeuristicCostEstimate(Segment const & from, Segment const & to) override;
  RouteWeight GetAStarWeightEpsilon() override;
  // @}

  m2::PointD const & GetPoint(Segment const & segment, bool front);

private:
  void GetEdgesList(Segment const & segment, bool isOutgoing, std::vector<SegmentEdge> & edges);

  void GetEdgesListForStart(Segment const & segment, bool isOutgoing,
                            std::vector<SegmentEdge> & edges);
  void GetEdgesListForFinish(Segment const & segment, bool isOutgoing,
                             std::vector<SegmentEdge> & edges);

  void GetEdgesForTwins(Segment const & segment, bool isOutgoing, std::vector<SegmentEdge> & edges);

  IndexGraphStarter & m_starter;
  std::map<Segment, RouteWeight> m_weightCacheForBackwardWave;
};
}  // namespace

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

  // AStarGraph overridings:
  // @{
  void GetOutgoingEdgesList(LeapSegment const & segment, std::vector<LeapEdge> & edges) override;
  void GetIngoingEdgesList(LeapSegment const & segment, std::vector<LeapEdge> & edges) override;
  RouteWeight HeuristicCostEstimate(LeapSegment const & from, LeapSegment const & to) override;
  // @}

  m2::PointD const & GetPoint(Segment const & segment, bool front) const;

private:
  void GetEdgesList(LeapSegment const & segment, bool isOutgoing, std::vector<LeapEdge> & edges);
  void GetEdgesListForStart(LeapSegment const & segment, bool isOutgoing,
                            std::vector<LeapEdge> & edges);
  void GetEdgesListForFinish(LeapSegment const & segment, bool isOutgoing,
                             std::vector<LeapEdge> & edges);

  IndexGraphStarter & m_starter;
};
}  // namespace

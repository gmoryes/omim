#pragma once

#include "routing/base/astar_graph.hpp"

#include "routing/index_graph_starter.hpp"
#include "routing/segment.hpp"

#include <vector>

namespace routing
{
class LeapsGraph : AStarGraph<Segment, SegmentEdge, double>
{
public:
  explicit LeapsGraph(IndexGraphStarter & starter) : m_starter(starter) {}

  std::vector<Segment> FindPathWithLeaps(Segment const & start, Segment const & finsih);

private:
  IndexGraphStarter & m_starter;
};
}  // namespace routing

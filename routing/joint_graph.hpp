#pragma once

#include "routing/index_graph.hpp"
#include "routing/joint_segment.hpp"
#include "routing/route_weight.hpp"

namespace routing
{
class JointGraph : public IndexGraph
{
public:
  using Vertex = JointSegment;
  using Weight = RouteWeight;
  using Edge = JointEdge;

  void GetEdgesList(Segment const & from, bool isOutgoing, vector<JointEdge> & edges);
  void GetEdgeListBoost(Segment const & from, bool isOutgoing, vector<SegmentEdge> & edges);
private:

  bool ProcessFakeEdge(Segment const & prev, Segment const & current, bool isOutgoing,
                       SegmentEdge & edge);

  bool CheckAndProcessTransitFeature(Segment const & segment, RouteWeight const & weight,
                                     bool isOutgoing, vector<SegmentEdge> & edges);
  void GetSegmentEdge(Segment const & prevSegment, Segment const & firstNext, uint32_t lastPointId, bool isOutgoing,
                      std::vector<SegmentEdge> & edges);

};
}  // namespace routing

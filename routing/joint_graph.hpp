#pragma once

#include "routing/index_graph.hpp"
#include "routing/joint_segment.hpp"
#include "routing/route_weight.hpp"
#include "routing/world_graph.hpp"
#include "routing/single_vehicle_world_graph.hpp"
#include "routing/fake_graph.hpp"
#include "routing/fake_vertex.hpp"
#include "routing/index_graph_starter.hpp"

namespace routing
{
class JointGraph
{
public:
  using Vertex = Segment;
  using Weight = RouteWeight;
  using Edge = SegmentEdge;

  JointGraph(IndexGraph const & indexGraph, SingleVehicleWorldGraph & singleVehicleWorldGraph,
             IndexGraphStarter const & starter):
    m_indexGraph(indexGraph),
    m_singleVehicleWorldGraph(singleVehicleWorldGraph),
    m_indexGraphStarter(starter) {}

  // Interface for AStarAlgorithm:
  void GetOutgoingEdgesList(Segment const & segment, vector<SegmentEdge> & edges);
  void GetIngoingEdgesList(Segment const & segment, vector<SegmentEdge> & edges);
  RouteWeight HeuristicCostEstimate(Segment const & from, Segment const & to);

  void GetEdgeList(Segment const & from, bool isOutgoing, vector<SegmentEdge> & edges);
  void GetEdgeListBoost(Segment const & from, bool isOutgoing, vector<SegmentEdge> & edges);
private:

  bool ProcessFakeEdge(Segment const & prev, Segment const & current, bool isOutgoing,
                       SegmentEdge & edge);

  bool CheckAndProcessTransitFeature(Segment const & segment, RouteWeight const & weight,
                                     bool isOutgoing, vector<SegmentEdge> & edges);
  void GetSegmentEdge(Segment const & prevSegment, Segment const & firstNext, uint32_t lastPointId, bool isOutgoing,
                      std::vector<SegmentEdge> & edges);

  IndexGraph const & m_indexGraph;
  SingleVehicleWorldGraph & m_singleVehicleWorldGraph;
  IndexGraphStarter const & m_indexGraphStarter;
};
}  // namespace routing

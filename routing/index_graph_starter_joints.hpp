#pragma once

#include "routing/index_graph_starter.hpp"
#include "routing/joint_segment.hpp"
#include "routing/segment.hpp"

#include "geometry/point2d.hpp"

#include <map>
#include <queue>
#include <set>
#include <vector>

namespace routing
{
class IndexGraphStarterJoints
{
public:
  using Vertex = JointSegment;
  using Edge = JointEdge;
  using Weight = RouteWeight;

  explicit IndexGraphStarterJoints(IndexGraphStarter & starter) : m_starter(starter) {}
  IndexGraphStarterJoints(IndexGraphStarter & starter,
                          Segment const & startSegment,
                          Segment const & endSegment);

  void Init(Segment const & startSegment, Segment const & endSegment);

  JointSegment const & GetStartJoint() const { return m_startJoint; }
  JointSegment const & GetFinishJoint() const { return m_endJoint; }

  // These functions are A* interface.
  RouteWeight HeuristicCostEstimate(JointSegment const & from, JointSegment const & to);

  m2::PointD const & GetPoint(JointSegment const & jointSegment, bool start);

  void GetOutgoingEdgesList(JointSegment const & vertex, std::vector<JointEdge> & edges)
  {
    GetEdgeList(vertex, true /* isOutgoing */, edges);
  }

  void GetIngoingEdgesList(JointSegment const & vertex, std::vector<JointEdge> & edges)
  {
    GetEdgeList(vertex, false /* isOutgoing */, edges);
  }

  WorldGraph::Mode GetMode() const { return m_starter.GetMode(); }
  // End of A* interface.

  IndexGraphStarter & GetStarter() { return m_starter; }

  /// \brief Reconstructs JointSegment by segment after building the route.
  std::vector<Segment> ReconstructJoint(JointSegment const & joint);

private:
  static auto constexpr kInvisibleId = std::numeric_limits<uint32_t>::max() - 2;

  struct FakeJointSegment
  {
    FakeJointSegment() = default;
    FakeJointSegment(Segment const & start, Segment const & end)
      : m_start(start), m_end(end) {}

    Segment GetSegment(bool start) const
    {
      return start ? m_start : m_end;
    }

    Segment m_start;
    Segment m_end;
  };

  void AddFakeJoints(Segment const & segment, bool isOutgoing, std::vector<JointEdge> & edges);

  void GetEdgeList(JointSegment const & vertex, bool isOutgoing, std::vector<JointEdge> & edges);

  JointSegment CreateFakeJoint(Segment const & from, Segment const & to);

  bool IsJoint(Segment const & segment, bool fromStart);

  /// \brief Makes BFS from |startSegment| in direction |fromStart| and find the closest segments
  /// which end RoadPoints are joints. Thus we build fake joint segments graph.
  std::vector<JointEdge> FindFirstJoints(Segment const & startSegment, bool fromStart);

  JointSegment CreateInvisibleJoint(Segment const & segment, bool start);

  bool IsInvisible(JointSegment const & jointSegment) const;

  // For GetEdgeList from segments.
  IndexGraphStarter & m_starter;

  // Fake start and end joints.
  JointSegment m_startJoint;
  JointSegment m_endJoint;

  Segment m_startSegment;
  Segment m_endSegment;

<<<<<<< HEAD
  // See comments in |GetEdgeList()| about |m_savedWeight|.
  std::map<JointSegment, Weight> m_savedWeight;

  // JointSegment consists of two segments of one feature.
  // FakeJointSegment consists of two segments of different features.
  // So we create an invalid JointSegment (see |ToFake()| method), that
  // converts to FakeJointSegments. This std::map is converter.
  std::map<JointSegment, FakeJointSegment> m_fakeJointSegments;
  std::map<JointSegment, std::vector<Segment>> m_reconstructedFakeJoints;

  // List of JointEdges that are outgoing from start.
  std::vector<JointEdge> m_startOutEdges;
  // List of incoming to finish.
=======
  // В случае определения веса ребер для обратного поиска при нахождении связи:
  // "родитель" - "ребенок" мы в процессе построения joint'a посчитали вес ребенка, но
  // вернуть должны вес "родителя", поэтому надо запомнимать вес ребенка и возвращать его
  // тогда, когда он сам будет в роли родителя, после удалять из std::map.
  std::map<JointSegment, Weight> m_savedWeight;

  // Для фейковых joint'ов получаем начальный и конечный сегмент фейкового joint'a.
  // Нужно потому, что в JointSegment могут храниться сегменты только одной фичи, в случае
  // фейкового joint'a конец и начало могут принадлежать разным фичам (фейковый и реальные сегменты).
  std::map<JointSegment, FakeJointSegment> m_fakeJointSegments;
  std::map<JointSegment, std::vector<Segment>> m_reconstructedFakeJoints;

  // Список joint'ов выходящих из стартовой вершины.
  std::vector<JointEdge> m_startOutEdges;
  // Это список входящих в конечную.
>>>>>>> [routing] Add IndexGraphStarterJoints for routing between joints, instead of segments, routing_integration_tests OK.
  std::vector<JointEdge> m_endOutEdges;

  uint32_t m_fakeId = 0;
  bool m_init = false;
};
}  // namespace routing

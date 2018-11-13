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
template <typename Graph>
class IndexGraphStarterJoints
{
public:
  using Vertex = JointSegment;
  using Edge = JointEdge;
  using Weight = RouteWeight;

  explicit IndexGraphStarterJoints(Graph & graph) : m_graph(graph) {}
  IndexGraphStarterJoints(Graph & graph,
                          Segment const & startSegment,
                          Segment const & endSegment);

  IndexGraphStarterJoints(Graph & graph,
                          Segment const & startSegment);

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

  WorldGraphMode GetMode() const { return m_graph.GetMode(); }
  // End of A* interface.

  Graph & GetGraph() { return m_graph; }

  /// \brief Reconstructs JointSegment by segment after building the route.
  std::vector<Segment> ReconstructJoint(JointSegment const & joint);

private:
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

  bool IsJoint(Segment const & segment, bool fromStart)
  {
    return m_graph.IsJoint(segment, fromStart);
  }

  /// \brief Makes BFS from |startSegment| in direction |fromStart| and find the closest segments
  /// which end RoadPoints are joints. Thus we build fake joint segments graph.
  std::vector<JointEdge> FindFirstJoints(Segment const & startSegment, bool fromStart);

  JointSegment CreateInvisibleJoint(Segment const & segment, bool start);

  bool IsInvisible(JointSegment const & jointSegment) const;

  static auto constexpr kInvisibleId = std::numeric_limits<uint32_t>::max() - 2;

  JointSegment CreateInvisibleJoint(Segment const & segment, bool start);

  bool IsInvisible(JointSegment const & jointSegment) const
  {
    return jointSegment.GetStartSegmentId() == jointSegment.GetEndSegmentId() &&
           jointSegment.GetStartSegmentId() >= kInvisibleId &&
           jointSegment.GetStartSegmentId() != std::numeric_limits<uint32_t>::max();
  }

  // For GetEdgeList from segments.
  Graph & m_graph;

  // Fake start and end joints.
  JointSegment m_startJoint;
  JointSegment m_endJoint;

  Segment m_startSegment;
  Segment m_endSegment;

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
  std::vector<JointEdge> m_endOutEdges;

  uint32_t m_fakeId = 0;
  bool m_init = false;
};

template <typename Graph>
IndexGraphStarterJoints<Graph>::IndexGraphStarterJoints(Graph & graph,
                                                        Segment const & startSegment,
                                                        Segment const & endSegment)
  : m_graph(graph), m_startSegment(startSegment), m_endSegment(endSegment)
{
  Init(m_startSegment, m_endSegment);
}

template <typename Graph>
IndexGraphStarterJoints<Graph>::IndexGraphStarterJoints(Graph & graph,
                                                        Segment const & startSegment)
  : m_graph(graph), m_startSegment(startSegment), m_endSegment(Segment())
{
  Init(m_startSegment, m_endSegment);
}

template <typename Graph>
void IndexGraphStarterJoints<Graph>::Init(Segment const & startSegment, Segment const & endSegment)
{
  m_startSegment = startSegment;
  m_endSegment = endSegment;

  if (IsRealSegment(startSegment))
    m_startJoint = CreateInvisibleJoint(startSegment, true /* start */);
  else
    m_startJoint = CreateFakeJoint(m_graph.GetStartSegment(), m_graph.GetStartSegment());

  if (IsRealSegment(endSegment))
    m_endJoint = CreateInvisibleJoint(endSegment, false /* start */);
  else
    m_endJoint = CreateFakeJoint(m_graph.GetFinishSegment(), m_graph.GetFinishSegment());

  m_reconstructedFakeJoints[m_startJoint] = {m_startSegment};
  m_reconstructedFakeJoints[m_endJoint] = {m_endSegment};

  m_startOutEdges = FindFirstJoints(startSegment, true /* fromStart */);
  m_endOutEdges = FindFirstJoints(endSegment, false /* fromStart */);

  m_savedWeight[m_endJoint] = Weight(0.0);
  for (auto const & edge : m_endOutEdges)
    m_savedWeight[edge.GetTarget()] = edge.GetWeight();

  m_init = true;
}

template <typename Graph>
RouteWeight IndexGraphStarterJoints<Graph>::HeuristicCostEstimate(JointSegment const & from, JointSegment const & to)
{
  Segment fromSegment;
  Segment toSegment;

  if (to.IsFake() || IsInvisible(to))
    toSegment = m_fakeJointSegments[to].GetSegment(false /* start */);
  else
    toSegment = to.GetSegment(false /* start */);

  if (from.IsFake() || IsInvisible(from))
    fromSegment = m_fakeJointSegments[from].GetSegment(false /* start */);
  else
    fromSegment = from.GetSegment(false /* start */);

  return m_graph.HeuristicCostEstimate(fromSegment, toSegment);

}

template <typename Graph>
m2::PointD const & IndexGraphStarterJoints<Graph>::GetPoint(JointSegment const & jointSegment, bool start)
{
  Segment segment;
  if (jointSegment.IsFake())
    segment = m_fakeJointSegments[jointSegment].GetSegment(start);
  else
    segment = jointSegment.GetSegment(start);

  return m_graph.GetPoint(segment, jointSegment.IsForward());
}

template <typename Graph>
std::vector<Segment> IndexGraphStarterJoints<Graph>::ReconstructJoint(JointSegment const & joint)
{
  // Есть невидимые joint, которые стоят перед стартовыми сегментами, в случае
  // если они не фейковые, такие мы пропускаем, потому что это просто абстракция
  // для общности алгоритма.
  if (IsInvisible(joint))
    return {};

  // В случае фейковой вершины, мы возращаем ее предпостроенный путь.
  if (joint.IsFake())
  {
    auto it = m_reconstructedFakeJoints.find(joint);
    CHECK(it != m_reconstructedFakeJoints.end(), ("Can not find such fake joint"));

    return it->second;
  }

  // Иначе просто восстанавливаем подряд сегменты.
  std::vector<Segment> subpath;

  Segment currentSegment = joint.GetSegment(true /* start */);
  Segment lastSegment = joint.GetSegment(false /* start */);
  bool forward = currentSegment.GetSegmentIdx() < lastSegment.GetSegmentIdx();

  while (currentSegment != lastSegment)
  {
    subpath.emplace_back(currentSegment);
    currentSegment.Next(forward);
  }
  subpath.emplace_back(lastSegment);

  return subpath;
}

template <typename Graph>
void IndexGraphStarterJoints<Graph>::AddFakeJoints(Segment const & segment, bool isOutgoing,
                                            std::vector<JointEdge> & edges)
{
  // If |isOutgoing| is true, we need real segments, that are real parts
  // of fake joints, entered to finish and vice versa.
  std::vector<JointEdge> const & endings = isOutgoing ? m_endOutEdges : m_startOutEdges;

  bool opposite = !isOutgoing;
  for (auto const & edge : endings)
  {
    // Фейковые сегменты устроены таким образом, что один конец у них это начало/конец, а
    // второй конец указывает на какой-то сегмент ведущий к другим joint'ам.
    // Поэтому мы берем конец каждого ребра и проверяем - не ведет ли он к финишу/старту.
    Segment firstSegment = m_fakeJointSegments[edge.GetTarget()].GetSegment(!opposite /* start */);
    if (firstSegment == segment)
    {
      edges.emplace_back(edge);
      return;
    }
  }
}

template <typename Graph>
Segment IndexGraphStarterJoints<Graph>::GetEndOfFakeJoint(JointSegment const & joint)
{
  auto const it = m_fakeJointSegments.find(joint);
  CHECK(it != m_fakeJointSegments.cend(), ("No such fake joint:", joint, "in JointStarter."));

  return (it->second).GetSegment(false /* start */);
}

template <typename Graph>
void IndexGraphStarterJoints<Graph>::GetEdgeList(JointSegment const & vertex, bool isOutgoing,
                                          std::vector<JointEdge> & edges)
{
  CHECK(m_init, ("IndexGraphStarterJoints was not initialized."));

  edges.clear();

  Segment parentSegment;
  // В этот вектор мы сохраняем вес родительского ребра относительно всех детей.
  // Пояснение: когда мы идем от родителя к ребенку в обратном поиске, то
  // вычисляем вес "ребенок" -> "родитель" и для каждого ребенка этот вес будет разный.
  std::vector<Weight> parentWeights;
  size_t firstFakeId = 0;

  // Case of fake start or finish joints.
  // Note: startJoint and finishJoint are just loops
  //       from start to start or end to end vertex.
  if (vertex == GetStartJoint())
  {
    edges.insert(edges.end(), m_startOutEdges.begin(), m_startOutEdges.end());
    parentWeights.insert(parentWeights.end(), edges.size(), Weight(0.0));
    firstFakeId = edges.size();
  }
  else if (vertex == GetFinishJoint())
  {
    edges.insert(edges.end(), m_endOutEdges.begin(), m_endOutEdges.end());
    // Вес родительского сегмента будет ноль, так как первая вершина это петля веса 0.
    parentWeights.insert(parentWeights.end(), edges.size(), Weight(0.0));
  }
  else
  {
    bool opposite = !isOutgoing;
    if (vertex.IsFake())
    {
      CHECK(m_fakeJointSegments.find(vertex) != m_fakeJointSegments.end(),
            ("No such fake joint:", vertex, "in JointStarter."));

      FakeJointSegment fakeJointSegment = m_fakeJointSegments[vertex];

      auto const & startSegment = isOutgoing ? m_startSegment : m_endSegment;
      auto const & endSegment = isOutgoing ? m_endSegment : m_startSegment;
      auto const & endJoint = isOutgoing ? GetFinishJoint() : GetStartJoint();

      // В vertex уже записан путь до конечной вершины, мы почти дошли.
      // Но чтобы A* понял, что мы дошли надо добавить конечную вершину в ребра к текущей.
      // Тогда он увидит, что среди ребер есть вершина в которой противоположенный поиск уже был
      // и закончит свою работу.
      if (fakeJointSegment.GetSegment(opposite /* start */) == endSegment)
      {
        if (isOutgoing)
        {
          static auto constexpr kZeroWeight = RouteWeight(0.0);
          edges.emplace_back(endJoint, kZeroWeight);
        }
        else
        {
          auto it = m_savedWeight.find(vertex);
          CHECK(it != m_savedWeight.end(), ("Can not find weight for:", vertex));

          Weight const & weight = it->second;
          edges.emplace_back(endJoint, weight);
        }
        return;
      }

      CHECK(fakeJointSegment.GetSegment(!opposite /* start */) == startSegment, ());
      parentSegment = fakeJointSegment.GetSegment(opposite /* start */);
    }
    else
    {
      parentSegment = vertex.GetSegment(opposite /* start */);
    }

    std::vector<JointEdge> jointEdges;
    m_graph.GetGraph().GetEdgeList(parentSegment, isOutgoing, jointEdges, parentWeights);
    edges.insert(edges.end(), jointEdges.begin(), jointEdges.end());

    firstFakeId = edges.size();
    for (size_t i = 0; i < jointEdges.size(); ++i)
    {
      size_t prevSize = edges.size();
      AddFakeJoints(jointEdges[i].GetTarget().GetSegment(!opposite /* start */), isOutgoing, edges);
      // Если мы добавили фейковое ребро, то надо сохранить вес "ребенок[i]" -> "родитель"
      // Так как у фекового ребра стартовый сегмент это i-ый ребенок, то мы и сохраняем
      // тот же самый вес |parentWeight[i]|.
      if (edges.size() != prevSize)
      {
        CHECK_LESS(i, parentWeights.size(), ());
        parentWeights.emplace_back(parentWeights[i]);
      }
    }
  }

  if (!isOutgoing)
  {
    // |parentSegment| - это вершина-родитель от которой мы ищем детей
    // Надо взять вес joint'a, который оканчивается в |parentSegment|
    auto it = m_savedWeight.find(vertex);
    CHECK(it != m_savedWeight.end(), ("Can not find weight for:", vertex));

    Weight const & weight = it->second;
    for (size_t i = 0; i < edges.size(); ++i)
    {
      // Сохраняем вес детей-joint'ов для последующих потомков.
      //m_savedWeight[edges[i].GetTarget().GetSegment(false /* start */)] = edges[i].GetWeight();
      m_savedWeight[edges[i].GetTarget()] = edges[i].GetWeight();
      // Для родетельского joint'a мы знаем его вес без последнего сегмента, так
      // как для всех детей он будет разным, но(!) мы этот вес сохранили при вычислении ребер
      // и для каждого ребенка вес родительсого сегмента лежит в |parentWeights[i]|
      CHECK_LESS(i, parentWeights.size(), ());
      edges[i].GetWeight() = weight + parentWeights[i];
    }

    // Вес родительского joint'a нам больше не нужен, поэтому удаляем его из std::map.
    m_savedWeight.erase(it);
  }
  else
  {
    // Этот цикл для прямого поиска нужен для правильного веса фейковых joint'ов.
    for (size_t i = firstFakeId; i < edges.size(); ++i)
      edges[i].GetWeight() += parentWeights[i];
  }
}

template <typename Graph>
JointSegment IndexGraphStarterJoints<Graph>::CreateFakeJoint(Segment const & from, Segment const & to)
{
  JointSegment jointSegment;
  jointSegment.ToFake(m_fakeId++);

  FakeJointSegment fakeJointSegment(from, to);
  m_fakeJointSegments[jointSegment] = fakeJointSegment;

  return jointSegment;
}

template <typename Graph>
std::vector<JointEdge> IndexGraphStarterJoints<Graph>::FindFirstJoints(Segment const & startSegment,
                                                                       bool fromStart)
{
  Segment endSegment = fromStart ? m_endSegment : m_startSegment;

  std::queue<Segment> queue;
  queue.push(startSegment);

  std::map<Segment, Segment> parent;
  std::map<Segment, RouteWeight> weight;
  std::vector<JointEdge> result;

  auto const reconstructPath = [&parent, &startSegment, &endSegment](Segment current, bool forward) {
    std::vector<Segment> path;
    // В случае если мы можем дойти от старта до финиша без joint, то не добавляем последний сегмент в маршрут
    // для корректного восстановления маршрута (иначе последний сегмент повторится два раза).
    if (current != endSegment)
      path.emplace_back(current);

    if (current == startSegment)
      return path;

    for (;;)
    {
      Segment parentSegment = parent[current];

      path.emplace_back(parentSegment);
      if (parentSegment == startSegment)
        break;
      current = parentSegment;
    }

    if (forward)
      std::reverse(path.begin(), path.end());

    return path;
  };

  auto const addFake = [&](Segment const & segment, Segment const & beforeConvert)
  {
    JointSegment fakeJoint;
    fakeJoint = fromStart ? CreateFakeJoint(startSegment, segment) :
                            CreateFakeJoint(segment, startSegment);
    result.emplace_back(fakeJoint, weight[beforeConvert]);

    std::vector<Segment> path = reconstructPath(beforeConvert, fromStart);
    m_reconstructedFakeJoints[fakeJoint] = path;
  };

  auto const isEndOfSegment = [&](Segment const & fake, Segment const & segment)
  {
    CHECK(!IsRealSegment(fake), ());

    auto fakeEnd = m_graph.GetPoint(fake, fake.IsForward());
    auto realEnd = m_graph.GetPoint(segment, segment.IsForward());

    static auto constexpr kEps = 1e-5;
    return base::AlmostEqualAbs(fakeEnd, realEnd, kEps);
  };

  while (!queue.empty())
  {
    Segment segment = queue.front();
    queue.pop();
    Segment beforeConvert = segment;
    // Либо сегмент фейковый и из него получается реальный, который joint, либо он настоящий и joint.
    if (((!IsRealSegment(segment) && m_graph.ConvertToReal(segment) &&
          isEndOfSegment(beforeConvert, segment)) || IsRealSegment(beforeConvert)) &&
        IsJoint(segment, fromStart))
    {
      addFake(segment, beforeConvert);
      continue;
    }

    if (beforeConvert == endSegment)
    {
      addFake(segment, beforeConvert);
      continue;
    }

    std::vector<SegmentEdge> edges;
    m_graph.GetEdgesList(beforeConvert, fromStart, edges);
    for (auto const & edge : edges)
    {
      Segment child = edge.GetTarget();
      auto const & newWeight = weight[beforeConvert] + edge.GetWeight();
      if (weight.find(child) == weight.end() || weight[child] > newWeight)
      {
        parent[child] = beforeConvert;
        weight[child] = newWeight;
        queue.push(child);
      }
    }
  }

  return result;
}

template <typename Graph>
JointSegment IndexGraphStarterJoints<Graph>::CreateInvisibleJoint(Segment const & segment, bool start)
{
  JointSegment result;
  result.ToFake(start ? kInvisibleId : kInvisibleId + 1);
  m_fakeJointSegments[result] = FakeJointSegment(segment, segment);

  return result;
}
}  // namespace routing

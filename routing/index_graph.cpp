#include "routing/index_graph.hpp"

#include "routing/restrictions_serialization.hpp"

#include "base/assert.hpp"
#include "base/checked_cast.hpp"
#include "base/exception.hpp"
#include "base/timer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
using namespace base;
using namespace routing;
using namespace std;

bool IsUTurn(Segment const & u, Segment const & v)
{
  return u.GetFeatureId() == v.GetFeatureId() && u.GetSegmentIdx() == v.GetSegmentIdx() &&
         u.IsForward() != v.IsForward();
}

bool IsRestricted(RestrictionVec const & restrictions, Segment const & u, Segment const & v,
                  bool isOutgoing)
{
  uint32_t const featureIdFrom = isOutgoing ? u.GetFeatureId() : v.GetFeatureId();
  uint32_t const featureIdTo = isOutgoing ? v.GetFeatureId() : u.GetFeatureId();

  if (!binary_search(restrictions.cbegin(), restrictions.cend(),
                     Restriction(Restriction::Type::No, {featureIdFrom, featureIdTo})))
  {
    return false;
  }

  if (featureIdFrom != featureIdTo)
    return true;

  // @TODO(bykoianko) According to current code if a feature id is marked as a feature with
  // restrictricted U-turn it's restricted to make a U-turn on the both ends of the feature.
  // Generally speaking it's wrong. In osm there's information about the end of the feature
  // where the U-turn is restricted. It's necessary to pass the data to mwm and to use it here.
  // Please see test LineGraph_RestrictionF1F1No for details.
  //
  // Another example when it's necessary to be aware about feature end participated in restriction
  // is
  //        *---F1---*
  //        |        |
  // *--F3--A        B--F4--*
  //        |        |
  //        *---F2---*
  // In case of restriction F1-A-F2 or F1-B-F2 of any type (No, Only) the important information
  // is lost.
  return IsUTurn(u, v);
}
}  // namespace

namespace routing
{
IndexGraph::IndexGraph(shared_ptr<Geometry> geometry, shared_ptr<EdgeEstimator> estimator)
  : m_geometry(geometry), m_estimator(move(estimator))
{
  CHECK(m_geometry, ());
  CHECK(m_estimator, ());
}

void IndexGraph::GetEdgeList(Segment const & segment, bool isOutgoing, vector<SegmentEdge> & edges) const
{
  RoadPoint const roadPoint = segment.GetRoadPoint(isOutgoing);
  Joint::Id const jointId = m_roadIndex.GetJointId(roadPoint);

  if (jointId != Joint::kInvalidId)
  {
    m_jointIndex.ForEachPoint(jointId, [&](RoadPoint const & rp) {
      GetNeighboringEdges(segment, rp, isOutgoing, edges);
    });
  }
  else
  {
    GetNeighboringEdges(segment, roadPoint, isOutgoing, edges);
  }
}

void IndexGraph::Build(uint32_t numJoints)
{
  m_jointIndex.Build(m_roadIndex, numJoints);
}

void IndexGraph::BuildJointIndex(NumMwmId numMwmId)
{
  /***********************************************/
  base::HighResTimer timer;
  BuildJointSegmentIndex(m_jointIndex.GetNumJoints(), numMwmId);
  uint64_t counter = timer.ElapsedMillis();
  std::ofstream output("/tmp/counter", std::ofstream::app);
  output << std::setprecision(20);
  output << "joint_segment_index_build_time: " << counter << "ms" << std::endl;
}

uint64_t countMainLoop;
uint64_t emplaceToHashMap;
uint64_t geometryCount;
void IndexGraph::BuildRoadOfJoints(NumMwmId numMwmId, uint32_t featureId, RoadJointIds const & roadJointIds,
                                   uint32_t & maxJointId)
{
  base::HighResTimer timer;
  std::vector<JointSegment> jointSegments;
  uint32_t const pointsNumber = GetGeometry().GetRoad(featureId).GetPointsCount();
  geometryCount += timer.ElapsedNano();
  ASSERT_GREATER_OR_EQUAL(pointsNumber, 2, ());

  auto const IsStartPoint = [](uint32_t pointId) { return pointId == 0; };
  auto const IsEndPoint = [pointsNumber](uint32_t pointId) { return pointId == pointsNumber - 1; };
  auto const IsValidJointPoint = [&IsStartPoint, &IsEndPoint, &roadJointIds](uint32_t pointId)
  {
    return !(roadJointIds.GetJointId(pointId) == Joint::kInvalidId &&
             !IsStartPoint(pointId) && !IsEndPoint(pointId));
  };
  auto const Diff = [](uint32_t a, uint32_t b) { return (a > b) ? a - b : b - a; };
  bool forwardCase;
  auto const InitPrevPoint = [pointsNumber, &forwardCase]() {
    return forwardCase ? 0 : pointsNumber - 1;
  };
  auto const InitCurrPoint = [&forwardCase](uint32_t prevPointId) {
    return forwardCase ? prevPointId + 1 : prevPointId - 1;
  };
  auto const Increment = [&forwardCase](uint32_t & currentPointId) {
    forwardCase ? ++currentPointId : --currentPointId;
  };

  auto MainLoop = [&](bool isForward)
  {
    forwardCase = isForward;
    uint32_t prevPointId = InitPrevPoint();
    for (uint32_t curPointId = InitCurrPoint(prevPointId); curPointId < pointsNumber; Increment(curPointId))
    {
      if (!IsValidJointPoint(curPointId))
        continue;

      JointSegment jointSegment(numMwmId, featureId, prevPointId, curPointId);
      if (Diff(prevPointId, curPointId) == 1)
      {
        jointSegments.push_back(jointSegment);
        prevPointId = curPointId;
        continue;
      }

      RouteWeight weight(0.0);
      SegmentEdge edge;
      uint32_t prevId = prevPointId;
      prevPointId = curPointId;
      while (Diff(prevId, curPointId) > 1)
      {
        uint32_t prevSegmentId = forwardCase ? prevId : prevId - 1;
        uint32_t curSegmentId = forwardCase ? prevId + 1 :  prevId - 2;
        Segment prevSegment(numMwmId, featureId, prevSegmentId, forwardCase);
        Segment curSegment(numMwmId, featureId, curSegmentId, forwardCase);

        if (GetNeighboringEdge(prevSegment, curSegment, true /* isOutgoing */, edge))
        {
          // Access from |prevSegment| to |curSegment| is OK.
          weight += edge.GetWeight();
        }
        else
        {
          break;
        }

        forwardCase ? ++prevId : --prevId;
      }

      if (Diff(prevId, curPointId) > 1)
        continue;

      // We pass through all segments, that's means, all access is OK and we can jump
      // from |prevPointId| to |curPointId| in routing.
      jointSegment.SetWeight(weight);
      jointSegments.push_back(jointSegment);
    }
  };

  timer.Reset();
  MainLoop(true /* isForward */);
  MainLoop(false /* isForward */);
  countMainLoop += timer.ElapsedNano();

  timer.Reset();
  for (auto const & jointSegment : jointSegments)
  {
    NumMwmId numMwmId = jointSegment.GetMwmId();
    uint32_t featureId = jointSegment.GetFeatureId();
    uint32_t startSegmentId = jointSegment.GetStartSegmentId();
    uint32_t endSegmentId = jointSegment.GetEndSegmentId();
    bool forward = jointSegment.IsForward();

    m_jointSegmentIndex.emplace(Segment(numMwmId, featureId, startSegmentId, forward), jointSegment);
    m_jointSegmentIndex.emplace(Segment(numMwmId, featureId, endSegmentId, forward), jointSegment);
  }
  emplaceToHashMap += timer.ElapsedNano();
}

void IndexGraph::BuildJointSegmentIndex(uint32_t maxJointId, NumMwmId numMwmId)
{
  base::HighResTimer timer;
  countMainLoop = 0;
  emplaceToHashMap = 0;
  geometryCount = 0;
  auto const callback = [this, &maxJointId, numMwmId](uint32_t featureId, RoadJointIds const & roadJointIds) {
      BuildRoadOfJoints(numMwmId, featureId, roadJointIds, maxJointId);
  };

  m_roadIndex.ForEachRoad(callback);

  uint64_t counter = timer.ElapsedNano();
  LOG(LINFO, ("main_loop(", countMainLoop / 1e6, "ms ), emplace_to_map(", emplaceToHashMap / 1e6, "ms ),",
              "geometry(", geometryCount/ 1e6, " ms), all(", counter / 1e6, "ms )"));
}

void IndexGraph::Import(vector<Joint> const & joints)
{
  m_roadIndex.Import(joints);
  CHECK_LESS_OR_EQUAL(joints.size(), numeric_limits<uint32_t>::max(), ());
  Build(checked_cast<uint32_t>(joints.size()));
}

void IndexGraph::SetRestrictions(RestrictionVec && restrictions)
{
  ASSERT(is_sorted(restrictions.cbegin(), restrictions.cend()), ());
  m_restrictions = move(restrictions);
}

void IndexGraph::SetRoadAccess(RoadAccess && roadAccess) { m_roadAccess = move(roadAccess); }

void IndexGraph::GetOutgoingEdgesList(Segment const & segment, vector<SegmentEdge> & edges)
{
  edges.clear();
  GetEdgeList(segment, true /* isOutgoing */, edges);
}

void IndexGraph::GetIngoingEdgesList(Segment const & segment, vector<SegmentEdge> & edges)
{
  edges.clear();
  GetEdgeList(segment, false /* isOutgoing */, edges);
}

RouteWeight IndexGraph::HeuristicCostEstimate(Segment const & from, Segment const & to)
{
  return RouteWeight(
      m_estimator->CalcHeuristic(GetPoint(from, true /* front */), GetPoint(to, true /* front */)));
}

RouteWeight IndexGraph::CalcSegmentWeight(Segment const & segment) const
{
  return RouteWeight(
      m_estimator->CalcSegmentWeight(segment, m_geometry->GetRoad(segment.GetFeatureId())));
}

void IndexGraph::GetNeighboringEdges(Segment const & from, RoadPoint const & rp, bool isOutgoing,
                                     vector<SegmentEdge> & edges) const
{
  RoadGeometry const & road = m_geometry->GetRoad(rp.GetFeatureId());

  if (!road.IsValid())
    return;

  bool const bidirectional = !road.IsOneWay();

  if ((isOutgoing || bidirectional) && rp.GetPointId() + 1 < road.GetPointsCount())
  {
    GetNeighboringEdge(from,
                       Segment(from.GetMwmId(), rp.GetFeatureId(), rp.GetPointId(), isOutgoing),
                       isOutgoing, edges);
  }

  if ((!isOutgoing || bidirectional) && rp.GetPointId() > 0)
  {
    GetNeighboringEdge(
        from, Segment(from.GetMwmId(), rp.GetFeatureId(), rp.GetPointId() - 1, !isOutgoing),
        isOutgoing, edges);
  }
}

void IndexGraph::GetNeighboringEdge(Segment const & from, Segment const & to, bool isOutgoing,
                                    vector<SegmentEdge> & edges) const
{
  // Blocking U-turns on internal feature points.
  RoadPoint const rp = from.GetRoadPoint(isOutgoing);
  if (IsUTurn(from, to) && m_roadIndex.GetJointId(rp) == Joint::kInvalidId
      && !m_geometry->GetRoad(from.GetFeatureId()).IsEndPointId(rp.GetPointId()))
  {
    return;
  }

  if (IsRestricted(m_restrictions, from, to, isOutgoing))
    return;

  if (m_roadAccess.GetFeatureType(to.GetFeatureId()) == RoadAccess::Type::No)
    return;

  if (m_roadAccess.GetPointType(rp) == RoadAccess::Type::No)
    return;

  RouteWeight const weight = CalcSegmentWeight(isOutgoing ? to : from) +
                             GetPenalties(isOutgoing ? from : to, isOutgoing ? to : from);
  edges.emplace_back(to, weight);
}

bool IndexGraph::GetNeighboringEdge(Segment const & from, Segment const & to, bool isOutgoing,
                                    SegmentEdge & edge) const
{
  std::vector<SegmentEdge> edges;
  GetNeighboringEdge(from, to, isOutgoing, edges);
  if (!edges.empty())
  {
    edge = edges.back();
    return true;
  }

  return false;
}

RouteWeight IndexGraph::GetPenalties(Segment const & u, Segment const & v) const
{
  bool const fromPassThroughAllowed = m_geometry->GetRoad(u.GetFeatureId()).IsPassThroughAllowed();
  bool const toPassThroughAllowed = m_geometry->GetRoad(v.GetFeatureId()).IsPassThroughAllowed();
  // Route crosses border of pass-through/non-pass-through area if |u| and |v| have different
  // pass through restrictions.
  int32_t const passThroughPenalty = fromPassThroughAllowed == toPassThroughAllowed ? 0 : 1;

  // We do not distinguish between RoadAccess::Type::Private and RoadAccess::Type::Destination for now.
  bool const fromAccessAllowed = m_roadAccess.GetFeatureType(u.GetFeatureId()) == RoadAccess::Type::Yes;
  bool const toAccessAllowed = m_roadAccess.GetFeatureType(v.GetFeatureId()) == RoadAccess::Type::Yes;
  // Route crosses border of access=yes/access={private, destination} area if |u| and |v| have different
  // access restrictions.
  int32_t accessPenalty = fromAccessAllowed == toAccessAllowed ? 0 : 1;

  // RoadPoint between u and v is front of u.
  auto const rp = u.GetRoadPoint(true /* front */);
  // No double penalty for barriers on the border of access=yes/access={private, destination} area.
  if (m_roadAccess.GetPointType(rp) != RoadAccess::Type::Yes)
    accessPenalty = 1;

  auto const uTurnPenalty = IsUTurn(u, v) ? m_estimator->GetUTurnPenalty() : 0.0;

  return RouteWeight(uTurnPenalty /* weight */, passThroughPenalty, accessPenalty, 0.0 /* transitTime */);
}
}  // namespace routing

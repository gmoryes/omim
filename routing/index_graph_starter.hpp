#pragma once

#include "routing/base/routing_result.hpp"
#include "routing/fake_ending.hpp"
#include "routing/fake_feature_ids.hpp"
#include "routing/fake_graph.hpp"
#include "routing/fake_vertex.hpp"
#include "routing/index_graph.hpp"
#include "routing/joint.hpp"
#include "routing/route_point.hpp"
#include "routing/route_weight.hpp"
#include "routing/segment.hpp"
#include "routing/world_graph.hpp"

#include "routing_common/num_mwm_id.hpp"

#include "base/assert.hpp"

#include "geometry/mercator.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <set>
#include <utility>
#include <vector>

namespace routing
{
class FakeEdgesContainer;

// IndexGraphStarter adds fake start and finish vertices for AStarAlgorithm.
class IndexGraphStarter final
{
public:
  // AStarAlgorithm types aliases:
  using Vertex = IndexGraph::Vertex;
  using Edge = IndexGraph::Edge;
  using Weight = IndexGraph::Weight;

  friend class FakeEdgesContainer;

  static void CheckValidRoute(std::vector<Segment> const & segments);
  static size_t GetRouteNumPoints(std::vector<Segment> const & route);

  static bool IsFakeSegment(Segment const & segment)
  {
    return segment.GetFeatureId() == kFakeFeatureId;
  }

  // strictForward flag specifies which parts of real segment should be placed from the start
  // vertex. true: place exactly one fake edge to the m_segment with indicated m_forward. false:
  // place two fake edges to the m_segment with both directions.
  IndexGraphStarter(FakeEnding const & startEnding, FakeEnding const & finishEnding,
                    uint32_t fakeNumerationStart, bool strictForward, WorldGraph & graph);

  void Append(FakeEdgesContainer const & container);

  WorldGraph & GetGraph() const { return m_graph; }
  WorldGraph::Mode GetMode() const { return m_graph.GetMode(); }
  Junction const & GetStartJunction() const;
  Junction const & GetFinishJunction() const;
  Segment GetStartSegment() const { return GetFakeSegment(m_start.m_id); }
  Segment GetFinishSegment() const { return GetFakeSegment(m_finish.m_id); }
  // If segment is real returns true and does not modify segment.
  // If segment is part of real converts it to real and returns true.
  // Otherwise returns false and does not modify segment.
  bool ConvertToReal(Segment & segment) const;
  Junction const & GetJunction(Segment const & segment, bool front) const;
  Junction const & GetRouteJunction(std::vector<Segment> const & route, size_t pointIndex) const;
  m2::PointD const & GetPoint(Segment const & segment, bool front) const;
  uint32_t GetNumFakeSegments() const
  {
    // Maximal number of fake segments in fake graph is numeric_limits<uint32_t>::max()
    // because segment idx type is uint32_t.
    CHECK_LESS_OR_EQUAL(m_fake.GetSize(), std::numeric_limits<uint32_t>::max(), ());
    return static_cast<uint32_t>(m_fake.GetSize());
  }

  std::set<NumMwmId> GetMwms() const;
  std::set<NumMwmId> GetStartMwms() const;
  std::set<NumMwmId> GetFinishMwms() const;

  // Checks whether |weight| meets non-pass-through crossing restrictions according to placement of
  // start and finish in pass-through/non-pass-through area and number of non-pass-through crosses.
  bool CheckLength(RouteWeight const & weight);

  void GetEdgesList(Segment const & segment, bool isOutgoing,
                    std::vector<SegmentEdge> & edges) const;

  void GetOutgoingEdgesList(Vertex const & segment, std::vector<Edge> & edges) const
  {
    GetEdgesList(segment, true /* isOutgoing */, edges);
  }

  void GetIngoingEdgesList(Vertex const & segment, std::vector<Edge> & edges) const
  {
    GetEdgesList(segment, false /* isOutgoing */, edges);
  }

  bool IsGoodLandmark(std::pair<double, double> const & landmark) const
  {
    static auto constexpr kMax = std::numeric_limits<double>::max();
    static auto constexpr kBadPair = std::make_pair(kMax, kMax);
    return landmark != kBadPair;
  }

  RouteWeight HeuristicCostEstimateLandmarks(Vertex const & from, Vertex const & to, bool forward) const
  {
    auto const logg = [&](Vertex const & v)
    {
      {
        std::ofstream output("/tmp/landmarks_features_matched", std::ofstream::app);
        output << std::setprecision(20);
        auto p = MercatorBounds::ToLatLon(GetPoint(v, true));
        output << p.lat << ", " << p.lon << ")" << std::endl;
      }
    };

    Vertex firstSegment;
    Vertex lastSegment;
    if (forward)
    {
      firstSegment = from;
      lastSegment = m_lastSegmentDebug;
    }
    else
    {
      firstSegment = m_firstSegmentDebug;
      lastSegment = to;
    }

    std::vector<std::pair<double, double>> fromLandmarks = m_graph.GetLandmarks(firstSegment, logg);
    std::vector<std::pair<double, double>> toLandmarks = m_graph.GetLandmarks(lastSegment, logg);

    size_t minN = std::min(fromLandmarks.size(), toLandmarks.size());
    size_t maxN = std::max(fromLandmarks.size(), toLandmarks.size());

    //minN = 0;
    if (minN == 0)
    {
      // FUCK THIS SHIT, NELZYA!!!
      //return m_graph.HeuristicCostEstimate(GetPoint(from, true /* front */),
      //                                     GetPoint(to, true /* front */));
      return RouteWeight(0.0);
    }

    CHECK_EQUAL(minN, maxN, ("WHAT THE FUCK"));
    double maxi = 0;
    size_t maxIndex;
    double from2Landmark;
    //double to2Landmark;

    for (size_t i = 0; i < maxN; ++i)
    {
      static auto constexpr kMax = std::numeric_limits<double>::max();
      if (fromLandmarks[i].second != kMax && toLandmarks[i].second != kMax) {
        auto const prikol = fromLandmarks[i].second /* backward */ - toLandmarks[i].second /* backward */;
        if (maxi < prikol)
        {
          maxi = prikol;
          maxIndex = i;
          from2Landmark = fromLandmarks[i].second;
        }
      }

      if (fromLandmarks[i].first != kMax && toLandmarks[i].first != kMax) {
        auto const prikol = toLandmarks[i].first /* forward */ - fromLandmarks[i].first /* forward */;
        if (maxi < prikol)
        {
          maxi = prikol;
          maxIndex = i;
          from2Landmark = fromLandmarks[i].first;
        }
      }

    }

    auto fromLatLon = MercatorBounds::ToLatLon(GetPoint(from, true));
    //auto p2 = MercatorBounds::ToLatLon(GetPoint(to, true));
    //auto tmp = m_graph.HeuristicCostEstimate(GetPoint(from, true /* front */), GetPoint(to, true /* front */));
    //LOG(LINFO, ("Approximate from(", from, "):", fromLatLon, "to(", lastSegment, "):", p2, "is:", maxi, "silly:", tmp.GetWeight()));

    {
      auto end = MercatorBounds::ToLatLon(GetPoint(lastSegment, true));
      std::ofstream output("/tmp/checker", std::ofstream::app);
      output << std::setprecision(20);
      output << fromLatLon.lat << " "
             << fromLatLon.lon << " "
             << end.lat << " "
             << end.lon << " "
             << maxi << " "
             << maxIndex << " "
             << from2Landmark << " "
             << from.GetFeatureId() << " "
             << from.GetSegmentIdx()
             << std::endl;
    }

    return RouteWeight(maxi);
  }

  RouteWeight HeuristicCostEstimate(Vertex const & from, Vertex const & to) const
  {
    return m_graph.HeuristicCostEstimate(GetPoint(from, true /* front */),
                                         GetPoint(to, true /* front */));
  }

  RouteWeight CalcSegmentWeight(Segment const & segment) const;
  RouteWeight CalcRouteSegmentWeight(std::vector<Segment> const & route, size_t segmentIndex) const;
  double CalcSegmentETA(Segment const & segment) const;

  Segment m_lastSegmentDebug;
  Segment m_firstSegmentDebug;
private:
  // Start or finish ending information. 
  struct Ending
  {
    bool OverlapsWithMwm(NumMwmId mwmId) const;

    // Fake segment id.
    uint32_t m_id = 0;
    // Real segments connected to the ending.
    std::set<Segment> m_real;
  };

  static Segment GetFakeSegment(uint32_t segmentIdx)
  {
    // We currently ignore |isForward| and use FakeGraph to get ingoing/outgoing.
    // But all fake segments are oneway and placement of segment head and tail
    // correspond forward direction.
    return Segment(kFakeNumMwmId, kFakeFeatureId, segmentIdx, true /* isForward */);
  }

  static Segment GetFakeSegmentAndIncr(uint32_t & segmentIdx)
  {
    CHECK_LESS(segmentIdx, std::numeric_limits<uint32_t>::max(), ());
    return GetFakeSegment(segmentIdx++);
  }

  // Creates fake edges for fake ending and adds it to  fake graph. |otherEnding| used to generate
  // propper fake edges in case both endings have projections to the same segment.
  void AddEnding(FakeEnding const & thisEnding, FakeEnding const & otherEnding, bool isStart,
                 bool strictForward, uint32_t & fakeNumerationStart);
  void AddStart(FakeEnding const & startEnding, FakeEnding const & finishEnding, bool strictForward,
                uint32_t & fakeNumerationStart);
  void AddFinish(FakeEnding const & finishEnding, FakeEnding const & startEnding,
                 uint32_t & fakeNumerationStart);

  // Adds fake edges of type PartOfReal which correspond real edges from |edges| and are connected
  // to |segment|
  void AddFakeEdges(Segment const & segment, bool isOutgoing, std::vector<SegmentEdge> & edges) const;

  // Checks whether ending belongs to pass-through or non-pass-through zone.
  bool EndingPassThroughAllowed(Ending const & ending);
  // Start segment is located in a pass-through/non-pass-through area.
  bool StartPassThroughAllowed();
  // Finish segment is located in a pass-through/non-pass-through area.
  bool FinishPassThroughAllowed();

  static uint32_t constexpr kFakeFeatureId = FakeFeatureIds::kIndexGraphStarterId;
  WorldGraph & m_graph;
  // Start segment id
  Ending m_start;
  // Finish segment id
  Ending m_finish;
  double m_startToFinishDistanceM;
  FakeGraph<Segment, FakeVertex, Segment> m_fake;
};
}  // namespace routing

#include "routing/leap_segment.hpp"

namespace routing
{
LeapSegment::LeapSegment(NumMwmId mwmId, uint32_t enterId, uint32_t exitId, bool forward)
  : m_numMwmId(mwmId)
  , m_enterId(enterId)
  , m_exitId(exitId)
  , m_forward(forward)
{
}
uint32_t LeapSegment::GetGate(bool isEnter)
{
  return isEnter ? m_enterId : m_exitId;
}

LeapEdge::LeapEdge(LeapSegment const & target, RouteWeight weight)
  : m_target(target), m_weight(weight)
{
}
}  // namespace routing
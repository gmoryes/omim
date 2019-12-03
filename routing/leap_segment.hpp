#pragma once

#include "routing/segment.hpp"
#include "routing/route_weight.hpp"

#include "routing_common/num_mwm_id.hpp"

#include <cstdint>

namespace routing
{
class LeapSegment
{
public:
  LeapSegment(NumMwmId mwmId, uint32_t enterId, uint32_t exitId, bool forward);

  uint32_t GetGate(bool isEnter);

private:
  NumMwmId m_numMwmId;
  uint32_t m_enterId;
  uint32_t m_exitId;
  bool m_forward;
};

class LeapEdge
{
public:
  LeapEdge(LeapSegment const & target, RouteWeight weight);

private:
  LeapSegment m_target;
  RouteWeight m_weight;
};
}  // namespace routing
#pragma once

#include "routing/route_weight.hpp"

#include "routing_common/num_mwm_id.hpp"

#include <cstdint>
#include <string>

namespace routing
{
class LeapSegment
{
public:
  LeapSegment() = default;
  constexpr LeapSegment(NumMwmId mwmId, uint32_t enterId, uint32_t exitId)
    : m_numMwmId(mwmId), m_enterId(enterId), m_exitId(exitId)
  {
  }

  NumMwmId GetMwmId() const { return m_numMwmId; }
  uint32_t GetGateId(bool isEnter) const { return isEnter ? m_enterId : m_exitId; }

  bool operator<(LeapSegment const & rhs) const
  {
    if (m_enterId != rhs.m_enterId)
      return m_enterId < rhs.m_enterId;

    if (m_exitId != rhs.m_exitId)
      return m_exitId < rhs.m_exitId;

    return m_numMwmId < rhs.m_numMwmId;
  }

  bool operator==(LeapSegment const & rhs) const
  {
    return m_enterId == rhs.m_enterId && m_exitId == rhs.m_exitId && m_numMwmId == rhs.m_numMwmId;
  }

  bool operator!=(LeapSegment const & rhs) const
  {
    return !(*this == rhs);
  }

private:
  NumMwmId m_numMwmId;
  uint32_t m_enterId;
  uint32_t m_exitId;
};

class LeapEdge
{
public:
  constexpr LeapEdge(LeapSegment const & segment, RouteWeight const & weight)
    : m_target(segment), m_weight(weight)
  {
  }

  LeapSegment const & GetTarget() const { return m_target; }
  RouteWeight const & GetWeight() const { return m_weight; }
  void SetWeight(RouteWeight const & weight) { m_weight = weight; }

private:
  LeapSegment m_target;
  RouteWeight m_weight;
};

std::string DebugPrint(LeapSegment const & leapSegment);
}  // namespace routing

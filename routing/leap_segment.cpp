#include "routing/leap_segment.hpp"

#include <sstream>

namespace routing
{
std::string DebugPrint(LeapSegment const & leapSegment)
{
  std::stringstream ss;

  auto const transformToString = [](uint32_t gateId) -> std::string {
    auto constexpr kStart = std::numeric_limits<uint32_t>::max();
    auto constexpr kFinish = std::numeric_limits<uint32_t>::max() - 1;
    auto constexpr kNoGate = std::numeric_limits<uint32_t>::max() - 2;
    switch (gateId)
    {
    case kStart: return "Start";
    case kFinish: return "Finish";
    case kNoGate: return "No gate";
    default: return std::to_string(gateId);
    }
  };

  ss << "LeapSegment(" << leapSegment.GetMwmId() << ", "
     << transformToString(leapSegment.GetGateId(true /* isEnter */)) << " => "
     << transformToString(leapSegment.GetGateId(false /* isEnter */)) << ")";
  return ss.str();
}
}  // namespace routing
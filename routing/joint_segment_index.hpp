#pragma once

#include "routing/joint_segment.hpp"

#include <map>

namespace routing
{
class JointSegmentIndex
{
public:

private:
  std::set<JointSegment> m_index;
};
}  // namespace routing

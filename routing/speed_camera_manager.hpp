#include <utility>

#pragma once

#include "platform/location.hpp"

#include "routing/route.hpp"
#include "routing/speed_camera.hpp"
#include "routing/routing_callbacks.hpp"

#include "base/assert.hpp"

#include <cstdint>
#include <memory>
#include <queue>
#include <vector>

namespace routing
{
class SpeedCameraManager
{
public:
  enum class Mode
  {
    Auto,
    Always,
    Never
  };
};
}  // namespace routing

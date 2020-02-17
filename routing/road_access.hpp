#pragma once

#include "routing/segment.hpp"
#include "routing/vehicle_mask.hpp"

#include "base/assert.hpp"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "3party/skarupke/flat_hash_map.hpp"
#include "3party/opening_hours/opening_hours.hpp"

namespace routing
{
// This class provides information about road access classes.
// One instance of RoadAccess holds information about one
// mwm and one router type (also known as VehicleType).
class RoadAccess final
{
public:
  // The road access types are selected by analyzing the most
  // popular tags used when mapping roads in OSM.
  enum class Type : uint8_t
  {
    // Moving through the road is prohibited.
    No,

    // Moving through the road requires a special permission.
    Private,

    // No pass through the road is allowed; however, it can
    // be used if it is close enough to the destination point
    // of the route.
    Destination,

    // No restrictions, as in "access=yes".
    Yes,

    // The number of different road types.
    Count
  };

  class Conditional
  {
  public:
    struct Access
    {
      Access(RoadAccess::Type type, osmoh::OpeningHours && openingHours)
          : m_type(type), m_openingHours(std::move(openingHours))
      {
      }

      bool operator==(Access const & rhs) const
      {
        return m_type == rhs.m_type && m_openingHours == rhs.m_openingHours;
      }

      RoadAccess::Type m_type = RoadAccess::Type::Count;
      osmoh::OpeningHours m_openingHours;
    };

    Conditional() = default;

    void Insert(RoadAccess::Type type, osmoh::OpeningHours && openingHours)
    {
      m_accesses.emplace_back(type, std::move(openingHours));
    }

    size_t Size() const { return m_accesses.size(); }
    bool IsEmpty() const { return m_accesses.empty(); }
    std::vector<Access> const & GetAccesses() const { return m_accesses; }

    bool operator==(Conditional const & rhs) const { return m_accesses == rhs.m_accesses; }
    bool operator!=(Conditional const & rhs) const { return !(*this == rhs); }

  private:
    std::vector<Access> m_accesses;
  };

  using WayToAccess = ska::flat_hash_map<uint32_t, RoadAccess::Type>;
  using PointToAccess = ska::flat_hash_map<RoadPoint, RoadAccess::Type, RoadPoint::Hash>;
  using WayToAccessConditional = ska::flat_hash_map<uint32_t, Conditional>;
  using PointToAccessConditional = ska::flat_hash_map<RoadPoint, Conditional, RoadPoint::Hash>;
  // TODO (@gmoryes) add PointToAccessConditional

  WayToAccess const & GetWayToAccess() const { return m_wayToAccess; }
  PointToAccess const & GetPointToAccess() const { return m_pointToAccess; }
  WayToAccessConditional const & GetWayToAccessConditional() const
  {
    return m_wayToAccessConditional;
  }

  PointToAccessConditional const & GetPointToAccessConditional() const
  {
    return m_pointToAccessConditional;
  }

  Type GetAccess(uint32_t featureId) const;
  Type GetAccess(RoadPoint const & point) const;

  template <typename WayToAccess, typename PointToAccess>
  void SetAccess(WayToAccess && wayToAccess, PointToAccess && pointToAccess)
  {
    m_wayToAccess = std::forward<WayToAccess>(wayToAccess);
    m_pointToAccess = std::forward<PointToAccess>(pointToAccess);
  }

  template <typename WayConditional>
  void SetAccessConditional(WayConditional && wayToAccessConditional,
                            PointToAccessConditional && pointToAccessConditional)
  {
    m_wayToAccessConditional = std::forward<WayConditional>(wayToAccessConditional);
    m_pointToAccessConditional = std::forward<PointToAccessConditional>(pointToAccessConditional);
  }

  bool operator==(RoadAccess const & rhs) const;

  template <typename WayToAccess>
  void SetWayToAccessForTests(WayToAccess && wayToAccess)
  {
    m_wayToAccess = std::forward<WayToAccess>(wayToAccess);
  }

private:
  // If segmentIdx of a key in this map is 0, it means the
  // entire feature has the corresponding access type.
  // Otherwise, the information is about the segment with number (segmentIdx-1).
  WayToAccess m_wayToAccess;
  PointToAccess m_pointToAccess;
  WayToAccessConditional m_wayToAccessConditional;
  PointToAccessConditional m_pointToAccessConditional;
};

std::string ToString(RoadAccess::Type type);
void FromString(std::string const & s, RoadAccess::Type & result);

std::string DebugPrint(RoadAccess::Conditional const & conditional);
std::string DebugPrint(RoadAccess::Type type);
std::string DebugPrint(RoadAccess const & r);
}  // namespace routing

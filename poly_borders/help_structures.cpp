#include "poly_borders/help_structures.hpp"

#include "geometry/mercator.hpp"

#include <algorithm>
#include <cmath>
#include <tuple>

namespace poly_borders
{
// Link --------------------------------------------------------------------------------------------
bool Link::operator<(Link const & rhs) const
{
  if (m_borderId != rhs.m_borderId)
    return m_borderId < rhs.m_borderId;

  return m_pointId < rhs.m_pointId;
}

// ReplaceData -------------------------------------------------------------------------------------

//static
bool ReplaceData::IsReversedIntervals(size_t fromDst, size_t toDst, size_t fromSrc, size_t toSrc)
{
  return (fromDst > toDst) != (fromSrc > toSrc);
}

bool ReplaceData::operator<(ReplaceData const & rhs) const
{
  return std::tie(m_dstFrom, m_dstTo) < std::tie(rhs.m_dstFrom, rhs.m_dstTo);
}

// MarkedPoint -------------------------------------------------------------------------------------
void MarkedPoint::AddLink(size_t borderId, size_t pointId)
{
  std::lock_guard<std::mutex> lock(*m_mutex);
  m_links.emplace(borderId, pointId);
}

bool MarkedPoint::GetLink(size_t curBorderId, Link & linkToSave)
{
  if (m_links.empty() || m_links.size() > 1)
    return false;

  size_t anotherBorderId = m_links.begin()->m_borderId;
  if (anotherBorderId == curBorderId)
    return false;

  linkToSave = *m_links.begin();
  return true;
}

// Polygon -----------------------------------------------------------------------------------------
void Polygon::MakeFrozen(size_t a, size_t b)
{
  CHECK_LESS(a, b, ());

  if (b - a + 1 > 2)
    m_replaced.AddInterval(a + 1, b - 1);
}

bool Polygon::IsFrozen(size_t a, size_t b)
{
  // We use LESS_OR_EQUAL because we want sometimes to check if
  // point i (associated with interval: [i, i]) is frozen.
  CHECK_LESS_OR_EQUAL(a, b, ());

  return m_replaced.Intersects(a, b);
}

void Polygon::AddReplaceInfo(size_t dstFrom, size_t dstTo,
                             size_t srcFrom, size_t srcTo, size_t srcBorderId,
                             bool reversed)
{
  CHECK_LESS_OR_EQUAL(dstFrom, dstTo, ());
  CHECK_LESS(srcFrom, srcTo, ());

  CHECK(!IsFrozen(dstFrom, dstTo), ());
  MakeFrozen(dstFrom, dstTo);

  m_replaceData.emplace(dstFrom, dstTo, srcFrom, srcTo, srcBorderId, reversed);
}

std::set<ReplaceData>::const_iterator Polygon::FindReplaceData(size_t index)
{
  for (auto it = m_replaceData.begin(); it != m_replaceData.end(); ++it)
  {
    if (it->m_dstFrom <= index && index <= it->m_dstTo)
      return it;
  }

  return m_replaceData.cend();
}

double FindPolygonArea(std::vector<m2::PointD> const & points, bool convertToMeters /* = true */)
{
  if (points.empty())
    return 0.0;

  double result = 0.0;
  for (size_t i = 0; i < points.size(); ++i)
  {
    auto const & prev = i == 0 ? points.back() : points[i - 1];
    auto const & cur = points[i];

    static m2::PointD const kZero = m2::PointD::Zero();
    auto const prevLen =
        convertToMeters ? MercatorBounds::DistanceOnEarth(kZero, prev) : prev.Length();
    auto const curLen =
        convertToMeters ? MercatorBounds::DistanceOnEarth(kZero, cur) : cur.Length();

    if (base::AlmostEqualAbs(prevLen, 0.0, 1e-20) || base::AlmostEqualAbs(curLen, 0.0, 1e-20))
      continue;

    double sinAlpha = CrossProduct(prev, cur) / (prev.Length() * cur.Length());

    result += prevLen * curLen * sinAlpha / 2.0;
  }

  return std::abs(result);
}
}  // namespace poly_borders

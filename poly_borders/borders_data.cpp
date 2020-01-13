#include "poly_borders/borders_data.hpp"

#include "poly_borders/help_structures.hpp"

#include "generator/borders.hpp"
#include "generator/routing_city_boundaries_processor.hpp"

#include "platform/platform.hpp"

#include "geometry/mercator.hpp"
#include "geometry/point2d.hpp"
#include "geometry/rect2d.hpp"
#include "geometry/region2d.hpp"

#include "base/assert.hpp"
#include "base/file_name_utils.hpp"
#include "base/logging.hpp"
#include "base/scope_guard.hpp"
#include "base/string_utils.hpp"
#include "base/thread_pool_computational.hpp"
#include "base/timer.hpp"

#include <algorithm>
#include <fstream>
#include <future>
#include <iostream>
#include <iterator>
#include <thread>
#include <tuple>
#include <utility>

namespace
{
using namespace poly_borders;

void PrintWithSpaces(std::string const & str, size_t maxN)
{
  std::cout << str;
  if (maxN <= str.size())
    return;

  maxN -= str.size();
  for (size_t i = 0; i < maxN; ++i)
    std::cout << " ";
}

std::string RemoveIndexFromMwmName(std::string const & mwmName)
{
  auto const pos = mwmName.find(BordersData::kBorderExtension);
  auto const end = mwmName.begin() + pos + BordersData::kBorderExtension.size();
  std::string result(mwmName.cbegin(), end);
  return result;
}

bool IsReversedIntervals(size_t fromDst, size_t toDst, size_t fromSrc, size_t toSrc)
{
  return (fromDst > toDst) != (fromSrc > toSrc);
}

std::vector<m2::PointD> CopyPointsWithAnyDirection(std::vector<MarkedPoint> const & copyFrom, 
                                                   size_t from, size_t to)
{
  std::vector<m2::PointD> result;
  if (from > to)
    std::swap(from, to);

  for (size_t k = from; k <= to; ++k)
    result.emplace_back(copyFrom[k].m_point);
  
  return result;
}

double AbsAreaDiff(std::vector<m2::PointD> const & firstPolygon,
                   std::vector<m2::PointD> const & secondPolygon)
{
  auto const currentArea = generator::routing_city_boundaries::AreaOnEarth(firstPolygon);
  auto const anotherArea = generator::routing_city_boundaries::AreaOnEarth(secondPolygon);
  return std::abs(anotherArea - currentArea);
}

bool NeedReplace(std::vector<m2::PointD> const & curSubpolygon,
                 std::vector<m2::PointD> const & anotherSubpolygon)
{
  auto const areaDiff = AbsAreaDiff(curSubpolygon, anotherSubpolygon);
  double constexpr kMaxAreaDiff = 20000.0;
  if (areaDiff > kMaxAreaDiff)
    return false;

  if (base::AlmostEqualAbs(areaDiff, 0.0, BordersData::kEqualityEpsilon))
    return false;

  // We know that |curSize| is always greater than 1, because we construct it such way, but we know
  // nothing about |anotherSize|, and we do not want to replace current subpolygon of several points
  // to subpolygon consists of one point.
  CHECK_GREATER(curSubpolygon.size(), 1, ());
  return anotherSubpolygon.size() > 1 &&
         std::max(curSubpolygon.size(), anotherSubpolygon.size()) < 10;
}

bool ShouldLog(size_t i, size_t n)
{
  return (i % 100 == 0) || (i + 1 == n);
}

void CopyWithReversedOption(size_t from, size_t to, bool reversed,
                            std::vector<MarkedPoint> const & copyFrom,
                            std::vector<MarkedPoint> & copyTo)
{
  if (!reversed)
  {
    for (size_t k = from; k < to; ++k)
      copyTo.emplace_back(copyFrom[k].m_point);
  }
  else
  {
    for (size_t k = to; k > from; --k)
      copyTo.emplace_back(copyFrom[k].m_point);
  }
}

m2::RectD FindRectForPolygon(std::vector<MarkedPoint> const & points)
{
  m2::RectD rect;
  for (auto const & point : points)
    rect.Add(point.m_point);

  return rect;
}

void MakeSwapIfNeeded(size_t & a, size_t & b)
{
  if (a > b)
    std::swap(a, b);
}
}  // namespace

namespace poly_borders
{
// BordersData::Processor --------------------------------------------------------------------------
void BordersData::Processor::operator()(size_t borderId)
{
  if (ShouldLog(borderId, m_data.m_bordersPolygons.size()))
    LOG(LINFO, ("Marking:", borderId + 1, "/", m_data.m_bordersPolygons.size()));

  auto const & polygon = m_data.m_bordersPolygons[borderId];
  for (size_t pointId = 0; pointId < polygon.m_points.size(); ++pointId)
    m_data.MarkPoint(borderId, pointId);
}

// BordersData -------------------------------------------------------------------------------------
void BordersData::Init(std::string const & bordersDir)
{
  size_t prevIndex = 0;
  LOG(LINFO, ("Borders path:", bordersDir));

  std::vector<std::string> files;
  Platform::GetFilesByExt(bordersDir, kBorderExtension, files);

  LOG(LINFO, ("Start reading data from .poly files."));
  for (auto const & file : files)
  {
    auto const fullPath = base::JoinPath(bordersDir, file);
    auto fileCopy = file;
    size_t id = 1;

    std::vector<m2::RegionD> borders;
    borders::LoadBorders(fullPath, borders);
    for (auto const & region : borders)
    {
      Polygon polygon;
      // Some mwms could have several polygons, for example if Russia_Moscow have 2 polygons, we
      // will name mwm such way:
      // Russia_Moscow.poly1
      // Russia_Moscow.poly2
      fileCopy = file + std::to_string(id);

      m_indexToMwmName[prevIndex] = fileCopy;
      m_mwmNameToIndex[fileCopy] = prevIndex++;
      for (auto const & point : region.Data())
        polygon.m_points.emplace_back(point);

      polygon.m_rect = region.GetRect();
      ++id;
      m_bordersPolygons.emplace_back(std::move(polygon));
    }
  }

  RemoveDuplicatePoints();
}

void BordersData::MarkPoints()
{
  size_t const threadsNumber = std::thread::hardware_concurrency();
  LOG(LINFO, ("Start marking points, threads number:", threadsNumber));

  base::thread_pool::computational::ThreadPool threadPool(threadsNumber);

  std::vector<std::future<void>> tasks;
  for (size_t i = 0; i < m_bordersPolygons.size(); ++i)
  {
    Processor processor(*this);
    tasks.emplace_back(threadPool.Submit(processor, i));
  }

  for (auto & task : tasks)
    task.wait();
}

void BordersData::DumpPolyFiles(std::string const & targetDir)
{
  size_t n = m_bordersPolygons.size();
  for (size_t i = 0; i < n; ++i)
  {
    if (ShouldLog(i, n))
      LOG(LINFO, ("Dumping poly files:", i + 1, "/", n));

    // Russia_Moscow.poly1 -> Russia_Moscow.poly
    auto name = RemoveIndexFromMwmName(m_indexToMwmName[i]);
    std::vector<m2::RegionD> regions;
    for (;;)
    {
      m2::RegionD region;
      for (auto const & markedPoint : m_bordersPolygons[i].m_points)
        region.AddPoint(markedPoint.m_point);

      regions.emplace_back(std::move(region));

      if (i + 1 < n && name == RemoveIndexFromMwmName(m_indexToMwmName[i + 1]))
        ++i;
      else
        break;
    }

    CHECK(strings::ReplaceFirst(name, kBorderExtension, ""), (name));
    borders::DumpBorderToPolyFile(targetDir, name, regions);
  }
}

void BordersData::RemoveDuplicatePoints()
{
  size_t count = 0;
  for (auto & polygon : m_bordersPolygons)
  {
    std::vector<MarkedPoint> withoutDuplicates;
    CHECK(!polygon.m_points.empty(), ());
    withoutDuplicates.emplace_back(polygon.m_points[0].m_point);
    for (size_t i = 1; i < polygon.m_points.size(); ++i)
    {
      if (!base::AlmostEqualAbs(withoutDuplicates.back().m_point, polygon.m_points[i].m_point,
                                kEqualityEpsilon))
      {
        withoutDuplicates.emplace_back(polygon.m_points[i].m_point);
      }
    }

    CHECK_GREATER_OR_EQUAL(polygon.m_points.size(), withoutDuplicates.size(), ());
    count += polygon.m_points.size() - withoutDuplicates.size();
    polygon.m_points = std::move(withoutDuplicates);
  }

  m_dublicatePointsCount += count;
  LOG(LINFO, ("Remove:", count, "duplicate points."));
}

void BordersData::MarkPoint(size_t curBorderId, size_t pointId)
{
  MarkedPoint & markedPoint = m_bordersPolygons[curBorderId].m_points[pointId];

  for (size_t borderId = 0; borderId < m_bordersPolygons.size(); ++borderId)
  {
    if (curBorderId == borderId)
      continue;

    if (markedPoint.m_marked)
      return;

    Polygon & anotherPolygon = m_bordersPolygons[borderId];

    if (!anotherPolygon.m_rect.IsPointInside(markedPoint.m_point))
      continue;

    for (size_t anotherPointId = 0; anotherPointId < anotherPolygon.m_points.size(); ++anotherPointId)
    {
      auto & anotherMarkedPoint = anotherPolygon.m_points[anotherPointId];

      if (base::AlmostEqualAbs(anotherMarkedPoint.m_point, markedPoint.m_point, kEqualityEpsilon))
      {
        anotherMarkedPoint.m_marked = true;
        markedPoint.m_marked = true;

        // We say to our point that border with id: |borderId| has the same point with id:
        // |anotherPointId|.
        markedPoint.AddLink(borderId, anotherPointId);
        // And vice versa.
        anotherMarkedPoint.AddLink(curBorderId, pointId);

        return;
      }
    }
  }
}

void BordersData::PrintDiff()
{
  using Info = std::tuple<double, std::string, size_t, size_t>;

  std::set<Info> info;

  size_t allNumberBeforeCount = 0;
  size_t maxMwmNameLength = 0;
  static double constexpr kAreaEpsMetersSqr = 1e-4;
  for (size_t i = 0; i < m_bordersPolygons.size(); ++i)
  {
    auto const & mwmName = m_indexToMwmName[i];

    auto const all = static_cast<int32_t>(m_bordersPolygons[i].m_points.size());
    auto const allBefore = static_cast<int32_t>(m_prevCopy[i].m_points.size());

    CHECK_GREATER_OR_EQUAL(allBefore, all, ());
    m_removePointsCount += allBefore - all;
    allNumberBeforeCount += allBefore;

    static std::string const kEpsString = std::to_string(kAreaEpsMetersSqr);
    double area = 0.0;
    if (m_additionalAreaMetersSqr[i] >= kAreaEpsMetersSqr)
      area = m_additionalAreaMetersSqr[i];

    maxMwmNameLength = std::max(maxMwmNameLength, mwmName.size());
    info.emplace(area, mwmName, allBefore, all);
  }

  for (auto const & item : info)
  {
    double area;
    std::string name;
    size_t allBefore;
    size_t all;

    std::tie(area, name, allBefore, all) = item;

    size_t diff = allBefore - all;
    PrintWithSpaces(name, maxMwmNameLength + 1);
    PrintWithSpaces("-" + std::to_string(diff) + " points", 17);

    std::cout << " summary changed area: " << area << " m^2" << std::endl;
  }

  std::cout << "Number of removed points: " << m_removePointsCount << std::endl;
  std::cout << "All amount of removed point (+duplicates): "
            << m_removePointsCount + m_dublicatePointsCount << std::endl;
  std::cout << "Points number before processing: " << allNumberBeforeCount << ", remove("
            << static_cast<double>(m_removePointsCount + m_dublicatePointsCount) /
                   allNumberBeforeCount * 100.0
            << "%)" << std::endl;
}

void BordersData::RemoveEmptySpaceBetweenBorders()
{
  LOG(LINFO, ("Start removing empty space between borders."));

  for (size_t curBorderId = 0; curBorderId < m_bordersPolygons.size(); ++curBorderId)
  {
    LOG(LDEBUG, ("Get:", m_indexToMwmName[curBorderId]));

    if (ShouldLog(curBorderId, m_bordersPolygons.size()))
      LOG(LINFO, ("Removing empty spaces:", curBorderId + 1, "/", m_bordersPolygons.size()));

    auto & curPolygon = m_bordersPolygons[curBorderId];
    for (size_t curPointId = 0; curPointId < curPolygon.m_points.size(); ++curPointId)
    {
      if (curPolygon.IsFrozen(curPointId, curPointId) || !HasLinkAt(curBorderId, curPointId))
        continue;

      size_t constexpr kMaxLookAhead = 5;
      for (size_t shift = 1; shift <= kMaxLookAhead; ++shift)
      {
        if (TryToReplace(curBorderId, curPointId /* curLeftPointId */,
                         curPointId + shift /* curRightPointId */) == base::ControlFlow::Break)
        {
          break;
        }
      }
    }
  }

  DoReplace();
}

base::ControlFlow BordersData::TryToReplace(size_t curBorderId, size_t & curLeftPointId,
                                            size_t curRightPointId)
{
  auto & curPolygon = m_bordersPolygons[curBorderId];
  if (curRightPointId >= curPolygon.m_points.size())
    return base::ControlFlow::Break;

  if (curPolygon.IsFrozen(curRightPointId, curRightPointId))
  {
    curLeftPointId = curRightPointId - 1;
    return base::ControlFlow::Break;
  }

  auto & leftMarkedPoint = curPolygon.m_points[curLeftPointId];
  auto & rightMarkedPoint = curPolygon.m_points[curRightPointId];

  auto op = rightMarkedPoint.GetLink(curBorderId);
  if (!op)
    return base::ControlFlow::Continue;

  Link rightLink = *op;
  Link leftLink = *leftMarkedPoint.GetLink(curBorderId);

  if (leftLink.m_borderId != rightLink.m_borderId)
    return base::ControlFlow::Continue;

  auto const anotherBorderId = leftLink.m_borderId;
  auto const anotherLeftPointId = leftLink.m_pointId;
  auto const anotherRightPointId = rightLink.m_pointId;
  auto & anotherPolygon = m_bordersPolygons[anotherBorderId];

  if (anotherPolygon.IsFrozen(std::min(anotherLeftPointId, anotherRightPointId),
                              std::max(anotherLeftPointId, anotherRightPointId)))
  {
    return base::ControlFlow::Continue;
  }

  auto const anotherSubpolygon =
      CopyPointsWithAnyDirection(anotherPolygon.m_points, anotherLeftPointId, anotherRightPointId);

  auto const curSubpolygon =
      CopyPointsWithAnyDirection(curPolygon.m_points, curLeftPointId, curRightPointId);

  if (!NeedReplace(curSubpolygon, anotherSubpolygon))
    return base::ControlFlow::Break;

  // We want to decrease the amount of points in polygons. So we will replace the greater amounts of
  // points by smaller amounts of points.
  bool const curLenIsLess = curSubpolygon.size() < anotherSubpolygon.size();

  size_t dstFrom = curLenIsLess ? anotherLeftPointId : curLeftPointId;
  size_t dstTo   = curLenIsLess ? anotherRightPointId : curRightPointId;

  size_t srcFrom = curLenIsLess ? curLeftPointId  : anotherLeftPointId;
  size_t srcTo   = curLenIsLess ? curRightPointId : anotherRightPointId;

  size_t const borderIdWhereAreaWillBeChanged = curLenIsLess ? anotherBorderId : curBorderId;
  size_t const srcBorderId = curLenIsLess ? curBorderId : anotherBorderId;

  bool const reversed = IsReversedIntervals(dstFrom, dstTo, srcFrom, srcTo);

  m_additionalAreaMetersSqr[borderIdWhereAreaWillBeChanged] +=
      AbsAreaDiff(curSubpolygon, anotherSubpolygon);

  MakeSwapIfNeeded(dstFrom, dstTo);
  MakeSwapIfNeeded(srcFrom, srcTo);

  // Save info for |borderIdWhereAreaWillBeChanged| - where from it should gets info about
  // replacement.
  m_bordersPolygons[borderIdWhereAreaWillBeChanged].AddReplaceInfo(
      dstFrom, dstTo, srcFrom, srcTo, srcBorderId, reversed);

  // And say for |srcBorderId| that points in segment: [srcFrom, srcTo] are frozen and can not
  // be used anywhere (because we use them to replace points in segment: [dstFrom, dstTo]).
  m_bordersPolygons[srcBorderId].MakeFrozen(srcFrom, srcTo);

  curLeftPointId = curRightPointId - 1;
  return base::ControlFlow::Break;
}

void BordersData::DoReplace()
{
  LOG(LINFO, ("Start replacing intervals."));

  std::vector<Polygon> newMwmsPolygons;

  for (size_t borderId = 0; borderId < m_bordersPolygons.size(); ++borderId)
  {
    if (ShouldLog(borderId, m_bordersPolygons.size()))
      LOG(LINFO, ("Replacing:", borderId + 1, "/", m_bordersPolygons.size()));

    auto & polygon = m_bordersPolygons[borderId];

    newMwmsPolygons.emplace_back();
    auto & newPolygon = newMwmsPolygons.back();

    for (size_t i = 0; i < polygon.m_points.size(); ++i)
    {
      auto const replaceDataIter = polygon.FindReplaceData(i);
      bool const noReplaceData = replaceDataIter == polygon.m_replaceData.cend();
      if (noReplaceData)
      {
        newPolygon.m_points.emplace_back(polygon.m_points[i].m_point);
        continue;
      }

      auto const srcBorderId = replaceDataIter->m_srcBorderId;
      size_t const srcFrom = replaceDataIter->m_srcReplaceFrom;
      size_t const srcTo = replaceDataIter->m_srcReplaceTo;
      size_t const nextI = replaceDataIter->m_dstTo;
      bool const reversed = replaceDataIter->m_reversed;

      CHECK_EQUAL(i, replaceDataIter->m_dstFrom, ());

      auto const & srcPolygon = m_bordersPolygons[srcBorderId];

      CopyWithReversedOption(srcFrom, srcTo, reversed, srcPolygon.m_points, newPolygon.m_points);

      polygon.m_replaceData.erase(replaceDataIter);
      i = nextI - 1;
    }

    newPolygon.m_rect = FindRectForPolygon(newPolygon.m_points);
  }

  m_prevCopy = std::move(m_bordersPolygons);
  m_bordersPolygons = std::move(newMwmsPolygons);
}

Polygon const & BordersData::GetBordersPolygonByName(std::string const & name) const
{
  auto id = m_mwmNameToIndex.at(name);
  return m_bordersPolygons.at(id);
}

bool BordersData::HasLinkAt(size_t curBorderId, size_t pointId)
{
  auto & leftMarkedPoint = m_bordersPolygons[curBorderId].m_points[pointId];
  return leftMarkedPoint.GetLink(curBorderId).has_value();
}
}  // namespace poly_borders

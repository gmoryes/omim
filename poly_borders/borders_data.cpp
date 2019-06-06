#include "poly_borders/borders_data.hpp"

#include "poly_borders/help_structures.hpp"

#include "generator/borders.hpp"

#include "platform/platform.hpp"

#include "geometry/mercator.hpp"
#include "geometry/point2d.hpp"
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
#include <iostream>
#include <iterator>
#include <utility>

namespace
{
using namespace poly_borders;

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

bool NeedReplace(size_t firstSize, size_t secondSize, double areaDiff)
{
  if (firstSize != secondSize)
    return true;

  return !base::AlmostEqualAbs(areaDiff, 0.0, BordersData::kEqualityEpsilon);
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
    {
      copyTo.emplace_back(copyFrom[k].m_point);
      if (k == 0)
        break;
    }
  }
}

m2::RectD FindRectForPolygon(std::vector<MarkedPoint> const & points)
{
  m2::RectD rect;
  for (auto const & point : points)
    rect.Add(point.m_point);

  return rect;
}

void MakeSwapIfNeeds(size_t & a, size_t & b)
{
  if (a > b)
    std::swap(a, b);
}
}  // namespace

namespace poly_borders
{
// static
double const BordersData::kEqualityEpsilon = 1e-20;
// static
std::string const BordersData::kBorderExtension = ".poly";

void BordersData::Init(std::string const & bordersDir)
{
  base::HighResTimer timer;
  SCOPE_GUARD(ReadDataTimer,
              [&]() { LOG(LINFO, ("Reading took:", timer.ElapsedNano() / 1e6, "ms")); });

  size_t prevIndex = 0;
  LOG(LINFO, ("Borders path:", bordersDir));

  std::vector<std::string> files;
  Platform::GetFilesByExt(bordersDir, kBorderExtension, files);

  LOG(LINFO, ("Start read data from .poly files."));
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

      m_indexToName[prevIndex] = fileCopy;
      m_nameToIndex[fileCopy] = prevIndex++;
      for (auto const & point : region.Data())
        polygon.m_points.emplace_back(point);

      polygon.m_rect = region.GetRect();

      fileCopy = file + std::to_string(id);
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

  base::thread_pool::computational::ThreadPool threadPool(1);

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

    auto name = m_indexToName[i];
    std::vector<std::vector<m2::PointD>> points;
    size_t id = 1;
    for (;;)
    {
      std::vector<m2::PointD> toAppend;
      for (auto const & markedPoint : m_bordersPolygons[i].m_points)
        toAppend.emplace_back(markedPoint.m_point);

      points.emplace_back(std::move(toAppend));
      if (i + 1 < n && m_indexToName[i + 1] == name + std::to_string(id))
      {
        ++i;
        ++id;
      }
      else
      {
        break;
      }
    }

    std::string const polyPath = base::JoinPath(targetDir, name);
    DumpPolyFile(polyPath, name, points);
  }
}

// static
void BordersData::DumpPolyFile(std::string const & polyPath, std::string mwmName,
                               std::vector<std::vector<m2::PointD>> const & polygons)
{
  std::ofstream output(polyPath);
  output << std::setprecision(20);

  CHECK(strings::EndsWith(mwmName, kBorderExtension), ());
  mwmName.replace(mwmName.end() - kBorderExtension.size(), mwmName.end(), "");

  output << mwmName << std::endl;
  for (size_t i = 0; i < polygons.size(); ++i)
  {
    output << i + 1 << std::endl;
    for (auto const & point : polygons[i])
    {
      auto const latlon = MercatorBounds::ToLatLon(point);
      output << latlon.m_lon << " " << latlon.m_lat << std::endl;
    }

    output << "END" << std::endl;
  }

  output << "END" << std::endl;
}

void BordersData::Processor::operator()(size_t borderId)
{
  if (ShouldLog(borderId, m_data.m_bordersPolygons.size()))
    LOG(LINFO, ("Marking:", borderId + 1, "/", m_data.m_bordersPolygons.size()));

  auto & polygon = m_data.m_bordersPolygons[borderId];
  for (int pointId = 0; pointId < polygon.m_points.size(); ++pointId)
  {
    auto & point = polygon.m_points[pointId];
    m_data.MarkPoint(borderId, point, pointId);
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

    count += polygon.m_points.size() - withoutDuplicates.size();
    polygon.m_points = std::move(withoutDuplicates);
  }

  LOG(LINFO, ("Remove:", count, "duplicate points."));
}

void BordersData::MarkPoint(size_t curBorderId, MarkedPoint & point, int pointId)
{
  for (size_t i = 0; i < m_bordersPolygons.size(); ++i)
  {
    if (curBorderId == i)
      continue;

    if (point.m_marked)
      return;

    Polygon & anotherPolygon = m_bordersPolygons[i];

    if (anotherPolygon.m_rect.IsPointOutside(point.m_point))
      continue;

    for (size_t anotherPointId = 0; anotherPointId < anotherPolygon.m_points.size(); ++anotherPointId)
    {
      auto & anotherPoint = anotherPolygon.m_points[anotherPointId];

      if (base::AlmostEqualAbs(anotherPoint.m_point, point.m_point, kEqualityEpsilon))
      {
        anotherPoint.m_marked = true;
        point.m_marked = true;

        point.AddLink(i, anotherPointId);
        anotherPoint.AddLink(curBorderId, pointId);

        return;
      }
    }
  }
}

BordersData::BorderResult BordersData::GetFinalResult(Polygon const & polygon)
{
  size_t all = 0;
  size_t marked = 0;

  for (auto const & point : polygon.m_points)
  {
    if (point.m_marked)
      ++marked;

    ++all;
  }

  return BorderResult(marked, all);
}

void PrintWithSpaces(std::string const & str, size_t maxN)
{
  std::cout << str;
  if (maxN <= str.size())
    return;

  maxN -= str.size();
  for (int i = 0; i < maxN; ++i)
    std::cout << " ";
}

void BordersData::PrintDiff()
{
  using Info = std::tuple<double, std::string, size_t, size_t>;

  std::set<Info> info;

  size_t maxMwmNameLength = 0;
  static double constexpr kAreaEpsMetersSqr = 1e-4;
  for (size_t i = 0; i < m_bordersPolygons.size(); ++i)
  {
    auto const & mwmName = m_indexToName[i];
    auto const & result = GetFinalResult(m_bordersPolygons[i]);

    auto const & resultBefore = GetFinalResult(m_prevCopy[i]);

    size_t all = result.m_all;
    size_t allBefore = resultBefore.m_all;

    m_numberOfAddPoints += all - allBefore;

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

    PrintWithSpaces(name, maxMwmNameLength + 1);
    PrintWithSpaces("+" + std::to_string(all - allBefore) +" new points", 17);

    std::cout << " summary changed area: " << area << " m^2"
              << std::endl;
  }

  std::cout << "Number of added points: " << m_numberOfAddPoints << std::endl;
}

void BordersData::RemoveEmptySpaceBetweenBorders()
{
  LOG(LINFO, ("Start removing empty space between borders."));

  for (size_t curBorderId = 0; curBorderId < m_bordersPolygons.size(); ++curBorderId)
  {
    if (ShouldLog(curBorderId, m_bordersPolygons.size()))
      LOG(LINFO, ("Removing empty spaces:", curBorderId + 1, "/", m_bordersPolygons.size()));

    auto & curPolygon = m_bordersPolygons[curBorderId];

    for (size_t i = 0; i < curPolygon.m_points.size(); ++i)
    {
      // if point with index = i is frozen.
      if (curPolygon.IsFrozen(i, i))
        continue;

      auto & prevPoint = curPolygon.m_points[i];
      Link prevLink;
      if (!prevPoint.GetLink(curBorderId, prevLink))
        continue;

      size_t anotherBorderId = prevLink.m_borderId;
      auto & anotherPolygon = m_bordersPolygons[anotherBorderId];

      static size_t constexpr kMaxLookAhead = 5;
      for (size_t shift = 1; shift <= kMaxLookAhead; ++shift)
      {
        if (i + shift >= curPolygon.m_points.size())
          break;

        if (curPolygon.IsFrozen(i + shift, i + shift))
        {
          i = i + shift - 1;
          break;
        }

        auto & curPoint = curPolygon.m_points[i + shift];

        Link curLink;
        if (!curPoint.GetLink(curBorderId, curLink))
          continue;

        if (curLink.m_borderId != prevLink.m_borderId)
          continue;

        auto anotherPrevPointId = prevLink.m_pointId;
        auto anotherCurPointId = curLink.m_pointId;

        if (anotherPolygon.IsFrozen(anotherPrevPointId, anotherCurPointId))
          continue;

        auto const currentIdsLength = shift + 1;
        auto const anotherIdsLength = std::abs(static_cast<int32_t>(anotherCurPointId) -
                                               static_cast<int32_t>(anotherPrevPointId)) + 1;

        auto const & anotherPoints = anotherPolygon.m_points;

        auto const anotherSubpolygon =
            CopyPointsWithAnyDirection(anotherPoints, anotherPrevPointId, anotherCurPointId);

        auto const curSubpolygon =
            CopyPointsWithAnyDirection(curPolygon.m_points, i, i + shift);

        auto const anotherArea = FindPolygonArea(anotherSubpolygon);
        auto const currentArea = FindPolygonArea(curSubpolygon);
        auto const areaDiff = std::abs(anotherArea - currentArea);

        if (!NeedReplace(curSubpolygon.size(), anotherSubpolygon.size(), areaDiff))
          break;

        static double constexpr kMaxAreaDiff = 2000;
        if (areaDiff < kMaxAreaDiff)
        {
          bool const curLenIsMore = currentIdsLength > anotherIdsLength;

          size_t dstFrom = curLenIsMore ? anotherPrevPointId : i;
          size_t dstTo   = curLenIsMore ? anotherCurPointId  : i + shift;

          size_t srcFrom = curLenIsMore ? i         : anotherPrevPointId;
          size_t srcTo   = curLenIsMore ? i + shift : anotherCurPointId;

          size_t const areaChangeBorderId = curLenIsMore ? anotherBorderId : curBorderId;
          size_t const srcBorderId        = curLenIsMore ? curBorderId     : anotherBorderId;

          bool const reversed = ReplaceData::IsReversedIntervals(dstFrom, dstTo, srcFrom, srcTo);

          m_additionalAreaMetersSqr[areaChangeBorderId] += areaDiff;

          MakeSwapIfNeeds(dstFrom, dstTo);
          MakeSwapIfNeeds(srcFrom, srcTo);

          m_bordersPolygons[areaChangeBorderId].AddReplaceInfo(dstFrom, dstTo,
                                                               srcFrom, srcTo, srcBorderId,
                                                               reversed);

          m_bordersPolygons[srcBorderId].MakeFrozen(srcFrom, srcTo);
        }

        i = i + shift - 1;
        break;
      }
    }
  }

  DoReplace();
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
  auto id = m_nameToIndex.at(name);
  return m_bordersPolygons.at(id);
}
}  // namespace poly_borders

#include "testing/testing.hpp"

#include "poly_borders/poly_borders_tests/tools.hpp"

#include "poly_borders/borders_data.hpp"

#include "platform/platform_tests_support/scoped_dir.hpp"
#include "platform/platform_tests_support/scoped_file.hpp"
#include "platform/platform_tests_support/writable_dir_changer.hpp"

#include "platform/platform.hpp"

#include "geometry/point2d.hpp"

#include <set>

using namespace platform::tests_support;
using namespace platform;
using namespace poly_borders;
using namespace std;

namespace
{
std::string const kTestDir = "borders_poly_dir";
std::string const & kExt = BordersData::kBorderExtension;

auto constexpr kSmallShift = 1e-9;

auto constexpr kSmallPointShift = m2::PointD(kSmallShift, kSmallShift);

void Process(BordersData & bordersData, std::string const & bordersDir)
{
  bordersData.Init(bordersDir);
  bordersData.MarkPoints();
  bordersData.RemoveEmptySpaceBetweenBorders();
}

bool ConsistOf(Polygon const & polygon, std::vector<m2::PointD> const & points)
{
  CHECK_EQUAL(polygon.m_points.size(), points.size(), ());

  std::set<size_t> used;
  for (auto const & point : points)
  {
    for (size_t i = 0; i < polygon.m_points.size(); ++i)
    {
      static double constexpr kEps = 1e-5;
      if (base::AlmostEqualAbs(point, polygon.m_points[i].m_point, kEps) &&
          used.count(i) == 0)
      {
        used.emplace(i);
        break;
      }
    }
  }

  return used.size() == points.size();
}

// Dummy test.
UNIT_TEST(PolyBordersPostprocessor_RemoveEmptySpaces_1)
{
  ScopedDir const scopedDir(kTestDir);
  std::string const & bordersDir = scopedDir.GetFullPath();

  m2::PointD a(0.0, 0.0);
  m2::PointD b(1.0, 0.0);
  m2::PointD c(2.0, 0.0);
  m2::PointD d(3.0, 0.0);
  m2::PointD e(4.0, 0.0);

  std::vector<std::vector<m2::PointD>> polygons1 = {
    {a, b, c, d, e}
  };

  std::vector<std::vector<m2::PointD>> polygons2 = {
    {a, b, c, d, e}
  };

  vector<shared_ptr<ScopedFile>> files;
  files.emplace_back(CreatePolyBorderFileByPolygon(kTestDir, "First", polygons1));
  files.emplace_back(CreatePolyBorderFileByPolygon(kTestDir, "Second", polygons2));

  BordersData bordersData;
  Process(bordersData, bordersDir);

  auto const & bordersPolygon1 = bordersData.GetBordersPolygonByName("First" + kExt);
  TEST(ConsistOf(bordersPolygon1, {a, b, c, d, e}), ());

  auto const & bordersPolygon2 = bordersData.GetBordersPolygonByName("Second" + kExt);
  TEST(ConsistOf(bordersPolygon2, {a, b, c, d, e}), ());
}

UNIT_TEST(PolyBordersPostprocessor_RemoveEmptySpaces_2)
{
  ScopedDir const scopedDir(kTestDir);
  std::string const & bordersDir = scopedDir.GetFullPath();

  m2::PointD a(0.0, 0.0);
  m2::PointD b(1.0, 0.0);
  // We should make c.y small because in other case changed area
  // will be so great, that point |c| will not be added.
  m2::PointD c(2.0, kSmallShift);
  m2::PointD d(3.0, 0.0);
  m2::PointD e(4.0, 0.0);

  std::vector<std::vector<m2::PointD>> polygons1 = {
      {a, b, c, d, e}
  };

  // Point |c| absents in polygons2, algorithm should add it.
  std::vector<std::vector<m2::PointD>> polygons2 = {
      {a, b, d, e}
  };

  vector<shared_ptr<ScopedFile>> files;
  files.emplace_back(CreatePolyBorderFileByPolygon(kTestDir, "First", polygons1));
  files.emplace_back(CreatePolyBorderFileByPolygon(kTestDir, "Second", polygons2));

  BordersData bordersData;
  Process(bordersData, bordersDir);

  auto const & bordersPolygon1 = bordersData.GetBordersPolygonByName("First" + kExt);
  TEST(ConsistOf(bordersPolygon1, {a, b, c, d, e}), ());

  auto const & bordersPolygon2 = bordersData.GetBordersPolygonByName("Second" + kExt);
  TEST(ConsistOf(bordersPolygon2, {a, b, c, d, e}), ());
}

// Like |PolyBordersPostprocessor_RemoveEmptySpaces_2| but two points will be
// added instead of one.
UNIT_TEST(PolyBordersPostprocessor_RemoveEmptySpaces_3)
{
  ScopedDir const scopedDir(kTestDir);
  std::string const & bordersDir = scopedDir.GetFullPath();

  m2::PointD a(0.0, 0.0);
  m2::PointD b(1.0, 0.0);
  // We should make c.y (and d.y) small because in other case changed area
  // will be so great, that point |c| (|d|) will not be added.
  m2::PointD c(2.0, kSmallShift);
  m2::PointD d(2.5, kSmallShift);
  m2::PointD e(4.0, 0.0);
  m2::PointD f(5.0, 0.0);

  std::vector<std::vector<m2::PointD>> polygons1 = {
      {a, b, c, d, e, f}
  };

  // Point |c| is absent from polygons2, algorithm should add it.
  std::vector<std::vector<m2::PointD>> polygons2 = {
      {a, b, e, f}
  };

  vector<shared_ptr<ScopedFile>> files;
  files.emplace_back(CreatePolyBorderFileByPolygon(kTestDir, "First", polygons1));
  files.emplace_back(CreatePolyBorderFileByPolygon(kTestDir, "Second", polygons2));

  BordersData bordersData;
  Process(bordersData, bordersDir);

  auto const & bordersPolygon1 = bordersData.GetBordersPolygonByName("First" + kExt);
  TEST(ConsistOf(bordersPolygon1, {a, b, c, d, e, f}), ());

  auto const & bordersPolygon2 = bordersData.GetBordersPolygonByName("Second" + kExt);
  TEST(ConsistOf(bordersPolygon2, {a, b, c, d, e, f}), ());
}

// Do not add point |c| because changed area is too big.
UNIT_TEST(PolyBordersPostprocessor_RemoveEmptySpaces_4)
{
  ScopedDir const scopedDir(kTestDir);
  std::string const & bordersDir = scopedDir.GetFullPath();

  m2::PointD a(0.0, 0.0);
  m2::PointD b(1.0, 0.0);
  m2::PointD c(2.0, 1.0);
  m2::PointD d(4.0, 0.0);
  m2::PointD e(5.0, 0.0);

  std::vector<std::vector<m2::PointD>> polygons1 = {
      {a, b, c, d, e}
  };

  std::vector<std::vector<m2::PointD>> polygons2 = {
      {a, b, d, e}
  };

  vector<shared_ptr<ScopedFile>> files;
  files.emplace_back(CreatePolyBorderFileByPolygon(kTestDir, "First", polygons1));
  files.emplace_back(CreatePolyBorderFileByPolygon(kTestDir, "Second", polygons2));

  BordersData bordersData;
  Process(bordersData, bordersDir);

  auto const & bordersPolygon1 = bordersData.GetBordersPolygonByName("First" + kExt);
  TEST(ConsistOf(bordersPolygon1, {a, b, c, d, e}), ());

  auto const & bordersPolygon2 = bordersData.GetBordersPolygonByName("Second" + kExt);
  TEST(ConsistOf(bordersPolygon2, {a, b, d, e}), ());
}

// Replace {c2, d2} -> {c1, d1, e1}.
UNIT_TEST(PolyBordersPostprocessor_RemoveEmptySpaces_5)
{
  ScopedDir const scopedDir(kTestDir);
  std::string const & bordersDir = scopedDir.GetFullPath();

  m2::PointD a(0.0, 0.0);
  m2::PointD b(9.0, 0.0);

  m2::PointD c1(2.0, 3.0);
  m2::PointD d1(4.0, 4.0);
  m2::PointD e1(d1 + kSmallPointShift + kSmallPointShift);

  m2::PointD c2(c1 + kSmallPointShift);
  m2::PointD d2(d1 + kSmallPointShift);

  std::vector<std::vector<m2::PointD>> polygons1 = {
      {a, c1, d1, e1, b}
  };

  std::vector<std::vector<m2::PointD>> polygons2 = {
      {a, c2, d2, b}
  };

  vector<shared_ptr<ScopedFile>> files;
  files.emplace_back(CreatePolyBorderFileByPolygon(kTestDir, "First", polygons1));
  files.emplace_back(CreatePolyBorderFileByPolygon(kTestDir, "Second", polygons2));

  BordersData bordersData;
  Process(bordersData, bordersDir);

  auto const & bordersPolygon1 = bordersData.GetBordersPolygonByName("First" + kExt);
  TEST(ConsistOf(bordersPolygon1, {a, c1, d1, e1, b}), ());

  auto const & bordersPolygon2 = bordersData.GetBordersPolygonByName("Second" + kExt);
  TEST(ConsistOf(bordersPolygon2, {a, c1, d1, e1, b}), ());
}

// Remove duplicates.
UNIT_TEST(PolyBordersPostprocessor_RemoveEmptySpaces_6)
{
  ScopedDir const scopedDir(kTestDir);
  std::string const & bordersDir = scopedDir.GetFullPath();

  m2::PointD a(0.0, 0.0);
  m2::PointD b(1.0, 0.0);
  m2::PointD c(2.0, 1.0);
  m2::PointD d(4.0, 0.0);
  m2::PointD e(5.0, 0.0);

  std::vector<std::vector<m2::PointD>> polygons1 = {
      {a, b, c, d, d, d, e, e, e}
  };

  std::vector<std::vector<m2::PointD>> polygons2 = {
      {a, d, d, d, e}
  };

  vector<shared_ptr<ScopedFile>> files;
  files.emplace_back(CreatePolyBorderFileByPolygon(kTestDir, "First", polygons1));
  files.emplace_back(CreatePolyBorderFileByPolygon(kTestDir, "Second", polygons2));

  BordersData bordersData;
  Process(bordersData, bordersDir);

  auto const & bordersPolygon1 = bordersData.GetBordersPolygonByName("First" + kExt);
  TEST(ConsistOf(bordersPolygon1, {a, b, c, d, e}), ());

  auto const & bordersPolygon2 = bordersData.GetBordersPolygonByName("Second" + kExt);
  TEST(ConsistOf(bordersPolygon2, {a, d, e}), ());
}
}  // namespace

#include "generator/routing_city_boundaries_processor.hpp"

#include "generator/collector_routing_city_boundaries.hpp"
#include "generator/feature_builder.hpp"

#include "indexer/classificator.hpp"

#include "coding/file_reader.hpp"

#include "geometry/area_on_earth.hpp"
#include "geometry/distance_on_sphere.hpp"
#include "geometry/mercator.hpp"
#include "geometry/point2d.hpp"

#include "base/logging.hpp"

#include "3party/boost/boost/geometry.hpp"
#include "3party/boost/boost/geometry/geometries/point_xy.hpp"
#include "3party/boost/boost/geometry/geometries/polygon.hpp"

#include <cstdint>
#include <sstream>
#include <unordered_map>
#include <utility>

namespace
{
using LocalityData = generator::RoutingCityBoundariesWriter::LocalityData;
std::unordered_map<uint64_t, LocalityData> LoadNodeToLocalityData(std::string const & filename)
{
  FileReader reader(filename);
  ReaderSource<FileReader> source(reader);
  uint64_t n = 0;
  source.Read(&n, sizeof(n));

  std::unordered_map<uint64_t, LocalityData> result;
  for (size_t i = 0; i < n; ++i)
  {
    uint64_t nodeId = 0;
    source.Read(&nodeId, sizeof(nodeId));

    LocalityData localityData = LocalityData::Deserialize(source);
    result[nodeId] = localityData;
  }

  return result;
}

std::unordered_map<uint64_t, std::vector<feature::FeatureBuilder>> LoadNodeToBoundariesData(
    std::string const & filename)
{
  using MaxAccuracy = feature::serialization_policy::MaxAccuracy;
  FileReader reader(filename);
  ReaderSource<FileReader> source(reader);
  uint64_t n = 0;
  source.Read(&n, sizeof(n));

  std::unordered_map<uint64_t, std::vector<feature::FeatureBuilder>> result;
  for (size_t i = 0; i < n; ++i)
  {
    uint64_t nodeId = 0;
    source.Read(&nodeId, sizeof(nodeId));

    feature::FeatureBuilder featureBuilder;
    feature::ReadFromSourceRawFormat<MaxAccuracy>(source, featureBuilder);
    result[nodeId].emplace_back(std::move(featureBuilder));
  }

  return result;
}

// Returns Well-Known-Text, for latlon polygon it is:
// "POLYGON((lon1 lat1, lon2 lat2, ... ,lonN latN))
std::string GetWKTString(std::vector<m2::PointD> const & points)
{
  std::string str = "POLYGON((0";
  std::stringstream ss;
  ss << "POLYGON((";
  ss << std::setprecision(20);

  auto const writePoint = [&ss](auto const & point, bool isEnd) {
    auto const latlon = mercator::ToLatLon(point);
    ss << latlon.m_lon << " " << latlon.m_lat;
    if (!isEnd)
      ss << ", ";
  };

  for (auto const & point : points)
    writePoint(point, false /* isEnd */);

  writePoint(points.front(), true /* isEnd */);

  ss << "))";
  return ss.str();
}

double AreaOnEarth(std::vector<m2::PointD> const & points)
{
  namespace bg = boost::geometry;
  using LonLatCoords = bg::cs::spherical_equatorial<bg::degree>;
  bg::model::polygon<bg::model::point<double, 2, LonLatCoords>> sphericalPolygon;
  bg::read_wkt(GetWKTString(points), sphericalPolygon);

  bg::strategy::area::spherical<> areaCalculationStrategy(ms::kEarthRadiusMeters);

  double const area = bg::area(sphericalPolygon, areaCalculationStrategy);
  return fabs(area);
}

std::pair<feature::FeatureBuilder, double> GetBoundaryWithSmallestArea(
    std::vector<feature::FeatureBuilder> const & boundaries)
{
  size_t bestIndex = 0;
  double minArea = std::numeric_limits<double>::max();
  for (size_t i = 0; i < boundaries.size(); ++i)
  {
    auto const & geometry = boundaries[i].GetOuterGeometry();
    double const area = AreaOnEarth(geometry);

    if (minArea > area)
    {
      minArea = area;
      bestIndex = i;
    }
  }

  return {boundaries[bestIndex], minArea};
}

std::vector<m2::PointD> CreateCircleGeometry(m2::PointD const & center, double radiusMercator,
                                             double angleStepDegree)
{
  std::vector<m2::PointD> result;
  double const radStep = base::DegToRad(angleStepDegree);
  for (double angleRad = 0; angleRad <= 2 * math::pi; angleRad += radStep)
  {
    result.emplace_back(center.x + radiusMercator * cos(angleRad),
                        center.y + radiusMercator * sin(angleRad));
  }
  return result;
}

void TransformPointToCircle(feature::FeatureBuilder & feature, m2::PointD const & center,
                            double radiusMeters)
{
  auto circleGeometry = CreateCircleGeometry(
      center, mercator::MetersToMercator(radiusMeters), 1.0 /* angleStepDegree */);

  feature.SetArea();
  feature.ResetGeometry();
  feature.AddPolygon(circleGeometry);
}
}  // namespace

namespace generator
{
RoutingCityBoundariesProcessor::RoutingCityBoundariesProcessor(std::string filename)
  : m_filename(std::move(filename))
{
}

void RoutingCityBoundariesProcessor::ProcessDataFromCollector()
{
  auto const nodeOsmIdToLocalityData = LoadNodeToLocalityData(
      RoutingCityBoundariesWriter::GetNodeToLocalityDataFilename(m_filename));

  auto const nodeOsmIdToBoundaries = LoadNodeToBoundariesData(
      RoutingCityBoundariesWriter::GetNodeToBoundariesFilename(m_filename));

  using FeatureWriter = feature::FeatureBuilderWriter<feature::serialization_policy::MaxAccuracy>;
  FeatureWriter featuresWriter(RoutingCityBoundariesWriter::GetFeaturesBuilderFilename(m_filename),
                               FileWriter::Op::OP_APPEND);

  uint32_t pointToCircle = 0;
  uint32_t matchedBoundary = 0;

  for (auto const & item : nodeOsmIdToBoundaries)
  {
    uint64_t const nodeOsmId = item.first;
    auto const & boundaries = item.second;

    auto const it = nodeOsmIdToLocalityData.find(nodeOsmId);
    if (it == nodeOsmIdToLocalityData.cend())
      continue;

    auto const & localityData = it->second;
    if (localityData.m_population == 0)
      continue;

    double bestFeatureBuilderArea = 0.0;
    feature::FeatureBuilder bestFeatureBuilder;
    std::tie(bestFeatureBuilder, bestFeatureBuilderArea) = GetBoundaryWithSmallestArea(boundaries);

    double const radiusMeters =
        ftypes::GetRadiusByPopulationForRouting(localityData.m_population, localityData.m_place);
    double const areaUpperBound = ms::CircleAreaOnEarth(radiusMeters);

    if (bestFeatureBuilderArea > areaUpperBound)
    {
      ++pointToCircle;
      TransformPointToCircle(bestFeatureBuilder, localityData.m_position, radiusMeters);
    }
    else
    {
      ++matchedBoundary;
    }

    static std::unordered_map<ftypes::LocalityType, uint32_t> const kLocalityTypeToClassifType = {
        {ftypes::LocalityType::City, classif().GetTypeByPath({"place", "city"})},
        {ftypes::LocalityType::Town, classif().GetTypeByPath({"place", "town"})},
        {ftypes::LocalityType::Village, classif().GetTypeByPath({"place", "village"})},
    };

    CHECK_NOT_EQUAL(kLocalityTypeToClassifType.count(localityData.m_place), 0, ());
    bestFeatureBuilder.AddType(kLocalityTypeToClassifType.at(localityData.m_place));

    if (bestFeatureBuilder.PreSerialize())
      featuresWriter.Write(bestFeatureBuilder);
  }

  LOG(LINFO, (pointToCircle, "places were transformed to circle."));
  LOG(LINFO, (matchedBoundary, "boundaries were approved as city/town/village boundary."));
}

void RoutingCityBoundariesProcessor::DumpBoundaries(std::string const & dumpFilename)
{
  using MaxAccuracy = feature::serialization_policy::MaxAccuracy;

  auto const features = feature::ReadAllDatRawFormat<MaxAccuracy>(
      RoutingCityBoundariesWriter::GetFeaturesBuilderFilename(m_filename));

  std::vector<m2::RectD> boundariesRects;
  boundariesRects.reserve(features.size());

  for (auto const & feature : features)
    boundariesRects.emplace_back(feature.GetLimitRect());

  FileWriter writer(dumpFilename);
  rw::WriteVectorOfPOD(writer, boundariesRects);
}
}  // namespace generator

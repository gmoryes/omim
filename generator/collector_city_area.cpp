#include "generator/collector_city_area.hpp"

#include "generator/intermediate_data.hpp"
#include "generator/osm_element.hpp"
#include "generator/osm_element_helpers.hpp"

#include "indexer/classificator.hpp"
#include "indexer/ftypes_matcher.hpp"

#include "coding/internal/file_data.hpp"

#include "geometry/area_on_earth.hpp"
#include "geometry/mercator.hpp"

#include "base/assert.hpp"
#include "base/string_utils.hpp"

#include <algorithm>
#include <cctype>
#include <iterator>
#include <limits>

using namespace feature;
using namespace feature::serialization_policy;

namespace
{
boost::optional<uint64_t> GetPlaceNodeFromMembers(OsmElement const & element)
{
  uint64_t adminCentreRef = 0;
  uint64_t labelRef = 0;
  for (auto const & member : element.m_members)
  {
    if (member.m_type == OsmElement::EntityType::Node)
    {
      if (member.m_role == "admin_centre")
        adminCentreRef = member.m_ref;
      else if (member.m_role == "label")
        labelRef = member.m_ref;
    }
  }

  if (labelRef)
    return {labelRef};

  if (adminCentreRef)
    return {adminCentreRef};

  return {};
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

bool IsSuitablePlaceType(ftypes::LocalityType localityType)
{
  switch (localityType)
  {
  case ftypes::LocalityType::City:
  case ftypes::LocalityType::Town:
  case ftypes::LocalityType::Village: return true;
  default: return false;
  }
}

ftypes::LocalityType GetPlaceType(FeatureBuilder const & feature)
{
  TypesHolder h;
  for (auto const t : feature.GetTypes())
    h.Add(t);

  return ftypes::IsLocalityChecker::Instance().GetType(h);
}

// Note: temporary function.
// TODO (@gmoryes) implementation.
double AreaOnEarth(std::vector<m2::PointD> const & points)
{
  return std::numeric_limits<double>::max();
}

std::pair<FeatureBuilder, double> GetBoundaryWithSmallestArea(
    std::vector<FeatureBuilder> const & boundaries)
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

void TransformPointToCircle(FeatureBuilder & feature, m2::PointD const & center, double radiusMeters)
{
  CHECK(feature.IsPoint(), (feature));
  auto const circleGeometry = CreateCircleGeometry(center,
                                                   MercatorBounds::MetersToMercator(radiusMeters),
                                                   1.0 /* angleStepDegree */);

  feature.SetArea();
  feature.ResetGeometry();
  for (auto const & point : circleGeometry)
    feature.AddPoint(point);
}
}  // namespace

namespace generator
{
CityAreaCollector::CityAreaCollector(std::string const & filename)
    : CollectorInterface(filename),
      m_writer(std::make_unique<FeatureBuilderWriter<MaxAccuracy>>(GetTmpFilename())) {}

std::shared_ptr<CollectorInterface>
CityAreaCollector::Clone(std::shared_ptr<cache::IntermediateDataReader> const &) const
{
  return std::make_shared<CityAreaCollector>(GetFilename());
}

void CityAreaCollector::CollectFeature(FeatureBuilder const & feature, OsmElement const & osmElement)
{
  if (feature.IsArea() && IsSuitablePlaceType(GetPlaceType(feature)))
  {
    auto copy = feature;
    if (copy.PreSerialize())
      m_writer->Write(copy);
    return;
  }

  if (feature.IsArea())
  {
    auto const placeOsmIdOp = GetPlaceNodeFromMembers(osmElement);
    if (!placeOsmIdOp)
      return;

    auto const placeOsmId = *placeOsmIdOp;
    m_nodeOsmIdToBoundaries[placeOsmId].emplace_back(feature);
    return;
  }
  else if (feature.IsPoint())
  {
    auto const placeType = GetPlaceType(feature);
    if (!IsSuitablePlaceType(placeType))
      return;

    uint64_t const population = osm_element::GetPopulation(osmElement);
    if (population == 0)
      return;

    uint64_t nodeOsmId = osmElement.m_id;
    m2::PointD const center = MercatorBounds::FromLatLon(osmElement.m_lat, osmElement.m_lon);
    m_nodeOsmIdToLocalityData.emplace(nodeOsmId, LocalityData(population, placeType, center));
  }
}

void CityAreaCollector::Finish()
{
  m_writer.reset({});
}

void CityAreaCollector::Save()
{
  uint32_t pointToCircle = 0;
  uint32_t matchedBoundary = 0;
  // |m_writer| is closed after CityAreaCollector::Finish(), so open it with append mode.
  m_writer = std::make_unique<FeatureBuilderWriter<MaxAccuracy>>(GetTmpFilename(),
                                                                 FileWriter::Op::OP_APPEND);
  for (auto const & item : m_nodeOsmIdToBoundaries)
  {
    uint64_t const nodeOsmId = item.first;
    auto const & boundaries = item.second;

    auto const it = m_nodeOsmIdToLocalityData.find(nodeOsmId);
    if (it == m_nodeOsmIdToLocalityData.cend())
      continue;

    auto const & localityData = it->second;
    if (localityData.m_population == 0)
      continue;

    double bestFeatureBuilderArea = 0.0;
    FeatureBuilder bestFeatureBuilder;
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
      m_writer->Write(bestFeatureBuilder);
  }

  m_writer.reset({});
  CHECK(base::CopyFileX(GetTmpFilename(), GetFilename()), ());
  LOG(LINFO, (pointToCircle, "places were transformed to circle."));
  LOG(LINFO, (matchedBoundary, "boundaries were approved as city/town/village boundary."));
}

void CityAreaCollector::Merge(generator::CollectorInterface const & collector)
{
  collector.MergeInto(*this);
}

void CityAreaCollector::MergeInto(CityAreaCollector & collector) const
{
  for (auto const & item : m_nodeOsmIdToBoundaries)
  {
    uint64_t const osmId = item.first;
    auto const & featureBuilders = item.second;

    collector.m_nodeOsmIdToBoundaries.emplace(osmId, featureBuilders);
  }

  for (auto const & item : m_nodeOsmIdToLocalityData)
  {
    uint64_t const osmId = item.first;
    auto const & localityData = item.second;

    collector.m_nodeOsmIdToLocalityData.emplace(osmId, localityData);
  }

  CHECK(!m_writer || !collector.m_writer, ("Finish() has not been called."));
  base::AppendFileToFile(GetTmpFilename(), collector.GetTmpFilename());
}
}  // namespace generator

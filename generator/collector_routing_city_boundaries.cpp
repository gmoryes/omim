#include "generator/collector_routing_city_boundaries.hpp"

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
#include <iterator>

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
    return labelRef;

  if (adminCentreRef)
    return adminCentreRef;

  return {};
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
  return ftypes::IsLocalityChecker::Instance().GetType(feature.GetTypesHolder());
}
}  // namespace

namespace generator
{
// RoutingCityBoundariesCollector::LocalityData ----------------------------------------------------

void RoutingCityBoundariesCollector::LocalityData::Serialize(FileWriter & writer,
                                                             LocalityData const & localityData)
{
  writer.Write(&localityData.m_population, sizeof(localityData.m_population));

  auto const placeType = static_cast<uint32_t>(localityData.m_place);
  writer.Write(&placeType, sizeof(placeType));

  writer.Write(&localityData.m_position, sizeof(localityData.m_position));
}

RoutingCityBoundariesCollector::LocalityData
RoutingCityBoundariesCollector::LocalityData::Deserialize(ReaderSource<FileReader> & reader)
{
  LocalityData localityData;
  reader.Read(&localityData.m_population, sizeof(localityData.m_population));

  uint32_t placeType = 0;
  reader.Read(&placeType, sizeof(placeType));
  localityData.m_place = static_cast<ftypes::LocalityType>(placeType);

  reader.Read(&localityData.m_position, sizeof(localityData.m_position));

  return localityData;
}

// RoutingCityBoundariesCollector ------------------------------------------------------------------

RoutingCityBoundariesCollector::RoutingCityBoundariesCollector(
    std::string const & filename, std::shared_ptr<cache::IntermediateData> cache)
  : CollectorInterface(filename)
  , m_writer(std::make_unique<RoutingCityBoundariesWriter>(GetTmpFilename()))
  , m_cache(std::move(cache))
  , m_featureMakerSimple(m_cache)
{
}

std::shared_ptr<CollectorInterface> RoutingCityBoundariesCollector::Clone(
    std::shared_ptr<cache::IntermediateDataReader> const &) const
{
  return std::make_shared<RoutingCityBoundariesCollector>(GetFilename(), m_cache);
}

void RoutingCityBoundariesCollector::Collect(OsmElement const & osmElement)
{
  auto osmElementCopy = osmElement;
  feature::FeatureBuilder feature;
  m_featureMakerSimple.Add(osmElementCopy);

  while (m_featureMakerSimple.GetNextFeature(feature))
  {
    if (feature.GetParams().IsValid())
      Process(feature, osmElementCopy);
  }
}

void RoutingCityBoundariesCollector::Process(feature::FeatureBuilder & feature,
                                             OsmElement const & osmElement)
{
  if (feature.IsArea() && IsSuitablePlaceType(GetPlaceType(feature)))
  {
    if (feature.PreSerialize())
      m_writer->Process(feature);
    return;
  }

  if (feature.IsArea())
  {
    auto const placeOsmIdOp = GetPlaceNodeFromMembers(osmElement);
    if (!placeOsmIdOp)
      return;

    auto const placeOsmId = *placeOsmIdOp;

    if (feature.PreSerialize())
      m_writer->Process(placeOsmId, feature);
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
    m_writer->Process(nodeOsmId, LocalityData(population, placeType, center));
  }
}

void RoutingCityBoundariesCollector::Finish() { m_writer->Reset(); }

void RoutingCityBoundariesCollector::Save()
{
  m_writer->Save(GetFilename());
}

void RoutingCityBoundariesCollector::Merge(generator::CollectorInterface const & collector)
{
  collector.MergeInto(*this);
}

void RoutingCityBoundariesCollector::MergeInto(RoutingCityBoundariesCollector & collector) const
{
  m_writer->MergeInto(*collector.m_writer);
}

// RoutingCityBoundariesWriter ---------------------------------------------------------------------

// static
std::string RoutingCityBoundariesWriter::GetNodeToLocalityDataFilename(std::string const & filename)
{
  return filename + ".nodeId2locality";
}

// static
std::string RoutingCityBoundariesWriter::GetNodeToBoundariesFilename(std::string const & filename)
{
  return filename + ".nodeId2Boundaries";
}

// static
std::string RoutingCityBoundariesWriter::GetFeaturesBuilderFilename(std::string const & filename)
{
  return filename + ".features";
}

RoutingCityBoundariesWriter::RoutingCityBoundariesWriter(std::string const & filename)
  : m_nodeOsmIdToLocalityDataFilename(GetNodeToLocalityDataFilename(filename))
  , m_nodeOsmIdToBoundariesFilename(GetNodeToBoundariesFilename(filename))
  , m_featureBuilderFilename(GetFeaturesBuilderFilename(filename))
  , m_nodeOsmIdToLocalityDataWriter(std::make_unique<FileWriter>(m_nodeOsmIdToLocalityDataFilename))
  , m_nodeOsmIdToBoundariesWriter(std::make_unique<FileWriter>(m_nodeOsmIdToBoundariesFilename))
  , m_featureBuilderWriter(std::make_unique<FeatureWriter>(m_featureBuilderFilename))
{
}

void RoutingCityBoundariesWriter::Process(uint64_t nodeOsmId, LocalityData const & localityData)
{
  m_nodeOsmIdToLocalityDataWriter->Write(&nodeOsmId, sizeof(nodeOsmId));
  LocalityData::Serialize(*m_nodeOsmIdToLocalityDataWriter, localityData);

  ++m_nodeOsmIdToLocalityDataCount;
}

void RoutingCityBoundariesWriter::Process(uint64_t nodeOsmId,
                                          feature::FeatureBuilder const & feature)
{
  m_nodeOsmIdToBoundariesWriter->Write(&nodeOsmId, sizeof(nodeOsmId));
  FeatureWriter::Write(*m_nodeOsmIdToBoundariesWriter, feature);

  ++m_nodeOsmIdToBoundariesCount;
}

void RoutingCityBoundariesWriter::Process(feature::FeatureBuilder const & feature)
{
  m_featureBuilderWriter->Write(feature);
}

void RoutingCityBoundariesWriter::Reset()
{
  m_nodeOsmIdToLocalityDataWriter.reset({});
  m_nodeOsmIdToBoundariesWriter.reset({});
  m_featureBuilderWriter.reset({});
}

void RoutingCityBoundariesWriter::MergeInto(RoutingCityBoundariesWriter & writer)
{
  CHECK(!m_nodeOsmIdToLocalityDataWriter || !writer.m_nodeOsmIdToLocalityDataWriter,
        ("Finish() has not been called."));
  base::AppendFileToFile(m_nodeOsmIdToLocalityDataFilename,
                         writer.m_nodeOsmIdToLocalityDataFilename);

  CHECK(!m_nodeOsmIdToBoundariesWriter || !writer.m_nodeOsmIdToBoundariesWriter,
        ("Finish() has not been called."));
  base::AppendFileToFile(m_nodeOsmIdToBoundariesFilename,
                         writer.m_nodeOsmIdToBoundariesFilename);

  CHECK(!m_featureBuilderWriter || !writer.m_featureBuilderWriter,
        ("Finish() has not been called."));
  base::AppendFileToFile(m_featureBuilderFilename,
                         writer.m_featureBuilderFilename);

  writer.m_nodeOsmIdToLocalityDataCount += m_nodeOsmIdToLocalityDataCount;
  writer.m_nodeOsmIdToBoundariesCount += m_nodeOsmIdToBoundariesCount;
}

void RoutingCityBoundariesWriter::Save(std::string const & finalFileName)
{
  auto const nodeToLocalityFilename = GetNodeToLocalityDataFilename(finalFileName);
  auto const nodeToBoundariesFilename = GetNodeToBoundariesFilename(finalFileName);

  {
    FileWriter writer(nodeToLocalityFilename, FileWriter::Op::OP_WRITE_TRUNCATE);
    writer.Write(&m_nodeOsmIdToLocalityDataCount, sizeof(m_nodeOsmIdToLocalityDataCount));
  }
  {
    FileWriter writer(nodeToBoundariesFilename, FileWriter::Op::OP_WRITE_TRUNCATE);
    writer.Write(&m_nodeOsmIdToBoundariesCount, sizeof(m_nodeOsmIdToBoundariesCount));
  }

  base::AppendFileToFile(m_nodeOsmIdToLocalityDataFilename, nodeToLocalityFilename);
  base::AppendFileToFile(m_nodeOsmIdToBoundariesFilename, nodeToBoundariesFilename);

  CHECK(base::CopyFileX(m_featureBuilderFilename,
                        GetFeaturesBuilderFilename(finalFileName)), ());
}
}  // namespace generator

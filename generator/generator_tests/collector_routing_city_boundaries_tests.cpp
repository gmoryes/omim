#include "testing/testing.hpp"

#include "generator/collector_routing_city_boundaries.hpp"
#include "generator/generator_tests/common.hpp"
#include "generator/osm2type.hpp"
#include "generator/osm_element.hpp"
#include "generator/routing_city_boundaries_processor.hpp"

#include "indexer/classificator_loader.hpp"

#include "platform/platform.hpp"

#include "geometry/point2d.hpp"

#include "base/geo_object_id.hpp"
#include "base/scope_guard.hpp"

#include <algorithm>
#include <memory>
#include <vector>

using namespace generator_tests;
using namespace generator;
using namespace feature;

namespace
{
using BoundariesCollector = RoutingCityBoundariesCollector;

std::vector<m2::PointD> GetFeatureBuilderGeometry()
{
  static std::vector<m2::PointD> kPolygon = {{0, 0}, {0, 2}, {2, 2}, {2, 0}, {0, 0}};
  return kPolygon;
}

feature::FeatureBuilder MakeAreaFeatureBuilder(OsmElement element)
{
  feature::FeatureBuilder result;
  auto const filterType = [](uint32_t) { return true; };
  ftype::GetNameAndType(&element, result.GetParams(), filterType);
  result.SetOsmId(base::MakeOsmRelation(element.m_id));
  auto polygon = GetFeatureBuilderGeometry();
  result.AddPolygon(polygon);
  result.SetArea();
  return result;
}

feature::FeatureBuilder MakeNodeFeatureBuilder(OsmElement element)
{
  feature::FeatureBuilder result;
  auto const filterType = [](uint32_t) { return true; };
  ftype::GetNameAndType(&element, result.GetParams(), filterType);
  result.SetOsmId(base::MakeOsmNode(element.m_id));
  result.SetCenter(mercator::FromLatLon(element.m_lat, element.m_lon));
  return result;
}

OsmElement MakeAreaWithPlaceNode(uint64_t id, uint64_t placeId, std::string const & role)
{
  auto area = MakeOsmElement(id, {{"boundary", "administrative"}}, OsmElement::EntityType::Relation);
  area.m_members.emplace_back(placeId, OsmElement::EntityType::Node, role);
  return area;
}

bool HasRelationWithId(std::vector<feature::FeatureBuilder> const & fbs, uint64_t id) {
  return std::find_if(std::begin(fbs), std::end(fbs), [&](auto const & fb) {
    return fb.GetMostGenericOsmId() == base::MakeOsmRelation(id);
  }) != std::end(fbs);
};

auto const placeRelation1 = MakeOsmElement(1 /* id */, {{"place", "city"}}, OsmElement::EntityType::Relation);
auto const placeRelation2 = MakeOsmElement(2 /* id */, {{"place", "town"}}, OsmElement::EntityType::Relation);
auto const placeRelation3 = MakeOsmElement(3 /* id */, {{"place", "village"}}, OsmElement::EntityType::Relation);
auto const placeRelation4 = MakeOsmElement(4 /* id */, {{"place", "country"}}, OsmElement::EntityType::Relation);

auto const placeNode1 = MakeOsmElement(9 /* id */, {{"place", "city"}, {"population", "200.000"}}, OsmElement::EntityType::Node);
auto const placeNode2 = MakeOsmElement(10 /* id */, {{"place", "town"}, {"population", "10 000"}}, OsmElement::EntityType::Node);
auto const placeNode3 = MakeOsmElement(11 /* id */, {{"place", "village"}, {"population", "1000"}}, OsmElement::EntityType::Node);
auto const placeNode4 = MakeOsmElement(12 /* id */, {{"place", "country"}, {"population", "147000000"}}, OsmElement::EntityType::Node);

auto const relationWithLabel1 = MakeAreaWithPlaceNode(5 /* id */, 9 /* placeId */, "label" /* role */);
auto const relationWithLabel2 = MakeAreaWithPlaceNode(6 /* id */, 10 /* placeId */, "admin_centre" /* role */);
auto const relationWithLabel3 = MakeAreaWithPlaceNode(7 /* id */, 11 /* placeId */, "label" /* role */);
auto const relationWithLabel4 = MakeAreaWithPlaceNode(8 /* id */, 12 /* placeId */, "country" /* role */);

void Collect(BoundariesCollector & collector, std::vector<OsmElement> const & elements)
{
  for (auto const & element : elements)
  {
    auto featureBuilder = element.IsNode() ? MakeNodeFeatureBuilder(element)
                                           : MakeAreaFeatureBuilder(element);
    collector.Process(featureBuilder, element);
  }
}

void Collect(std::shared_ptr<CollectorInterface> & collector,
             std::vector<OsmElement> const & elements)
{
  auto boundariesCollector = dynamic_cast<BoundariesCollector*>(collector.get());
  Collect(*boundariesCollector, elements);
}

void Check(std::string const & filename)
{
  using Writer = RoutingCityBoundariesWriter;

  auto const featuresFileName = Writer::GetFeaturesBuilderFilename(filename);

  auto const fbs = ReadAllDatRawFormat<serialization_policy::MinSize>(featuresFileName);
  TEST(HasRelationWithId(fbs, 1), ());
  TEST(HasRelationWithId(fbs, 2), ());
  TEST(HasRelationWithId(fbs, 3), ());
  TEST(!HasRelationWithId(fbs, 4), ());

  auto const nodeToBoundaryFilename = Writer::GetNodeToBoundariesFilename(filename);
  auto nodeToBoundary = routing_city_boundaries::LoadNodeToBoundariesData(nodeToBoundaryFilename);
  TEST(nodeToBoundary.count(9), ());
  TEST(nodeToBoundary.count(10), ());
  TEST(nodeToBoundary.count(11), ());
  TEST(!nodeToBoundary.count(12), ());

  auto const truePolygon = GetFeatureBuilderGeometry();
  for (size_t id = 9; id <= 11; ++id)
  {
    auto const & geometryFromFile = nodeToBoundary[id].back().GetOuterGeometry();
    for (size_t i = 0; i < geometryFromFile.size(); ++i)
      TEST_ALMOST_EQUAL_ABS(geometryFromFile[i], truePolygon[i], 1e-6, ());
  }

  auto const nodeToLocalityFilename = Writer::GetNodeToLocalityDataFilename(filename);
  auto nodeToLocality = routing_city_boundaries::LoadNodeToLocalityData(nodeToLocalityFilename);
  TEST(nodeToLocality.count(9), ());
  TEST(nodeToLocality.count(10), ());
  TEST(nodeToLocality.count(11), ());
  TEST(!nodeToLocality.count(12), ());

  TEST_EQUAL(nodeToLocality[9].m_place, ftypes::LocalityType::City, ());
  TEST_EQUAL(nodeToLocality[9].m_population, 200000, ());

  TEST_EQUAL(nodeToLocality[10].m_place, ftypes::LocalityType::Town, ());
  TEST_EQUAL(nodeToLocality[10].m_population, 10000, ());

  TEST_EQUAL(nodeToLocality[11].m_place, ftypes::LocalityType::Village, ());
  TEST_EQUAL(nodeToLocality[11].m_population, 1000, ());
}
}  // namespace

UNIT_TEST(CollectorRoutingCityBoundaries_1)
{
  classificator::Load();
  auto const filename = generator_tests::GetFileName();
  SCOPE_GUARD(_, std::bind(Platform::RemoveFileIfExists, std::cref(filename)));

  std::shared_ptr<cache::IntermediateData> cache;
  auto c1 = std::make_shared<BoundariesCollector>(filename, cache);

  Collect(*c1, {placeRelation1, placeRelation2, placeRelation3, placeRelation4});
  Collect(*c1, {relationWithLabel1, relationWithLabel2, relationWithLabel3, relationWithLabel4});
  Collect(*c1, {placeNode1, placeNode2, placeNode3, placeNode4});

  c1->Finish();
  c1->Save();

  Check(filename);
}

UNIT_TEST(CollectorRoutingCityBoundaries_2)
{
  classificator::Load();
  auto const filename = generator_tests::GetFileName();
  SCOPE_GUARD(_, std::bind(Platform::RemoveFileIfExists, std::cref(filename)));

  std::shared_ptr<cache::IntermediateData> cache;
  auto c1 = std::make_shared<BoundariesCollector>(filename, cache);
  auto c2 = c1->Clone();

  Collect(*c1, {placeRelation1, placeRelation2});
  Collect(c2, {placeRelation3, placeRelation4});

  Collect(*c1, {relationWithLabel1, relationWithLabel2});
  Collect(c2, {relationWithLabel3, relationWithLabel4});

  Collect(*c1, {placeNode1, placeNode2});
  Collect(c2, {placeNode3, placeNode4});

  c1->Finish();
  c2->Finish();
  c1->Merge(*c2);
  c1->Save();

  Check(filename);
}

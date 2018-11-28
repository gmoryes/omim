#include "routing_common/car_model.hpp"

#include "base/macros.hpp"

#include "indexer/classificator.hpp"

#include <algorithm>
#include <vector>

using namespace std;
using namespace routing;

using InOutCitySpeedKMpH = VehicleModel::InOutCitySpeedKMpH;
using SpeedKMpH = VehicleModel::SpeedKMpH;

// See model specifics in different countries here:
//   https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Access-Restrictions

// See road types here:
//   https://wiki.openstreetmap.org/wiki/Key:highway
// Speed of road features located inside and outside cities and towns polygons in km per hour.
//                                                      in city            out city
InOutCitySpeedKMpH const             kSpeedMotorwayKMpH(SpeedKMpH(117.8), SpeedKMpH(123.4));
InOutCitySpeedKMpH const         kSpeedMotorwayLinkKMpH(SpeedKMpH(82.0),   SpeedKMpH(81.2));
InOutCitySpeedKMpH const                kSpeedTrunkKMpH(SpeedKMpH(83.4),   SpeedKMpH(100.2));
InOutCitySpeedKMpH const            kSpeedTrunkLinkKMpH(SpeedKMpH(73.0),   SpeedKMpH(77.2));
InOutCitySpeedKMpH const              kSpeedPrimaryKMpH(SpeedKMpH(63.1),   SpeedKMpH(75.2));
InOutCitySpeedKMpH const          kSpeedPrimaryLinkKMpH(SpeedKMpH(66.5),   SpeedKMpH(64.8));
InOutCitySpeedKMpH const            kSpeedSecondaryKMpH(SpeedKMpH(52.8),   SpeedKMpH(60.3));
InOutCitySpeedKMpH const        kSpeedSecondaryLinkKMpH(SpeedKMpH(50.2),   SpeedKMpH(60.0));
InOutCitySpeedKMpH const             kSpeedTertiaryKMpH(SpeedKMpH(45.5),   SpeedKMpH(50.5));
InOutCitySpeedKMpH const         kSpeedTertiaryLinkKMpH(SpeedKMpH(25.0),   SpeedKMpH(30.0));
InOutCitySpeedKMpH const          kSpeedResidentialKMpH(SpeedKMpH(20.0),   SpeedKMpH(25.0));
InOutCitySpeedKMpH const         kSpeedUnclassifiedKMpH(SpeedKMpH(51.3),   SpeedKMpH(66.0));
InOutCitySpeedKMpH const              kSpeedServiceKMpH(SpeedKMpH(15.0),   SpeedKMpH(15.0));
InOutCitySpeedKMpH const         kSpeedLivingStreetKMpH(SpeedKMpH(10.0),   SpeedKMpH(10.0));
InOutCitySpeedKMpH const                 kSpeedRoadKMpH(SpeedKMpH(10.0),   SpeedKMpH(10.0));
InOutCitySpeedKMpH const                kSpeedTrackKMpH(SpeedKMpH(5.0),    SpeedKMpH(5.0));
InOutCitySpeedKMpH const        kSpeedFerryMotorcarKMpH(SpeedKMpH(10.0),   SpeedKMpH(10.0));
InOutCitySpeedKMpH const kSpeedFerryMotorcarVehicleKMpH(SpeedKMpH(10.0),   SpeedKMpH(10.0));
InOutCitySpeedKMpH const  kSpeedRailMotorcarVehicleKMpH(SpeedKMpH(10.0),   SpeedKMpH(10.0));
InOutCitySpeedKMpH const         kSpeedShuttleTrainKMpH(SpeedKMpH(25.0),   SpeedKMpH(25.0));
InOutCitySpeedKMpH const                 kSpeedPierKMpH(SpeedKMpH(10.0),   SpeedKMpH(10.0));

double constexpr kSpeedOffroadKMpH = 10.0;

static VehicleModel::LimitsInitList const & CarLimitsDefault()
{
  static VehicleModel::LimitsInitList const g_carLimitsDefault =
  {
    // {{roadType, roadType}        Speed km per hour    passThroughAllowed}
    {{"highway", "motorway"},       kSpeedMotorwayKMpH,      true},
    {{"highway", "motorway_link"},  kSpeedMotorwayLinkKMpH,  true},
    {{"highway", "trunk"},          kSpeedTrunkKMpH,         true},
    {{"highway", "trunk_link"},     kSpeedTrunkLinkKMpH,     true},
    {{"highway", "primary"},        kSpeedPrimaryKMpH,       true},
    {{"highway", "primary_link"},   kSpeedPrimaryLinkKMpH,   true},
    {{"highway", "secondary"},      kSpeedSecondaryKMpH,     true},
    {{"highway", "secondary_link"}, kSpeedSecondaryLinkKMpH, true},
    {{"highway", "tertiary"},       kSpeedTertiaryKMpH,      true},
    {{"highway", "tertiary_link"},  kSpeedTertiaryLinkKMpH,  true},
    {{"highway", "residential"},    kSpeedResidentialKMpH,   true},
    {{"highway", "unclassified"},   kSpeedUnclassifiedKMpH,  true},
    {{"highway", "service"},        kSpeedServiceKMpH,       true},
    {{"highway", "living_street"},  kSpeedLivingStreetKMpH,  true},
    {{"highway", "road"},           kSpeedRoadKMpH,          true},
    {{"highway", "track"},          kSpeedTrackKMpH,         true}
    /// @todo: Add to classificator
    //{ {"highway", "shuttle_train"},  10 },
    //{ {"highway", "ferry"},          5  },
    //{ {"highway", "default"},        10 },
    /// @todo: Check type
    //{ {"highway", "construction"},   40 },
  };
  
  return g_carLimitsDefault;
}

static VehicleModel::LimitsInitList const & CarLimitsNoPassThroughLivingStreet()
{
  static VehicleModel::LimitsInitList const g_carLimitsNoPassThroughLivingStreet =
  {
    {{"highway", "motorway"},       kSpeedMotorwayKMpH,      true},
    {{"highway", "motorway_link"},  kSpeedMotorwayLinkKMpH,  true},
    {{"highway", "trunk"},          kSpeedTrunkKMpH,         true},
    {{"highway", "trunk_link"},     kSpeedTrunkLinkKMpH,     true},
    {{"highway", "primary"},        kSpeedPrimaryKMpH,       true},
    {{"highway", "primary_link"},   kSpeedPrimaryLinkKMpH,   true},
    {{"highway", "secondary"},      kSpeedSecondaryKMpH,     true},
    {{"highway", "secondary_link"}, kSpeedSecondaryLinkKMpH, true},
    {{"highway", "tertiary"},       kSpeedTertiaryKMpH,      true},
    {{"highway", "tertiary_link"},  kSpeedTertiaryLinkKMpH,  true},
    {{"highway", "residential"},    kSpeedResidentialKMpH,   true},
    {{"highway", "unclassified"},   kSpeedUnclassifiedKMpH,  true},
    {{"highway", "service"},        kSpeedServiceKMpH,       true},
    {{"highway", "living_street"},  kSpeedLivingStreetKMpH,  false},
    {{"highway", "road"},           kSpeedRoadKMpH,          true},
    {{"highway", "track"},          kSpeedTrackKMpH,         true}
  };
    
  return g_carLimitsNoPassThroughLivingStreet;
}

static VehicleModel::LimitsInitList const & CarLimitsNoPassThroughLivingStreetAndService()
{
  static VehicleModel::LimitsInitList const g_carLimitsNoPassThroughLivingStreetAndService =
  {
    {{"highway", "motorway"},       kSpeedMotorwayKMpH,      true},
    {{"highway", "motorway_link"},  kSpeedMotorwayLinkKMpH,  true},
    {{"highway", "trunk"},          kSpeedTrunkKMpH,         true},
    {{"highway", "trunk_link"},     kSpeedTrunkLinkKMpH,     true},
    {{"highway", "primary"},        kSpeedPrimaryKMpH,       true},
    {{"highway", "primary_link"},   kSpeedPrimaryLinkKMpH,   true},
    {{"highway", "secondary"},      kSpeedSecondaryKMpH,     true},
    {{"highway", "secondary_link"}, kSpeedSecondaryLinkKMpH, true},
    {{"highway", "tertiary"},       kSpeedTertiaryKMpH,      true},
    {{"highway", "tertiary_link"},  kSpeedTertiaryLinkKMpH,  true},
    {{"highway", "residential"},    kSpeedResidentialKMpH,   true},
    {{"highway", "unclassified"},   kSpeedUnclassifiedKMpH,  true},
    {{"highway", "service"},        kSpeedServiceKMpH,       false},
    {{"highway", "living_street"},  kSpeedLivingStreetKMpH,  false},
    {{"highway", "road"},           kSpeedRoadKMpH,          true},
    {{"highway", "track"},          kSpeedTrackKMpH,         true}
  };
  
  return g_carLimitsNoPassThroughLivingStreetAndService;
}

VehicleModel::LimitsInitList const g_carLimitsAustralia = CarLimitsDefault();

VehicleModel::LimitsInitList const g_carLimitsAustria = CarLimitsNoPassThroughLivingStreet();

VehicleModel::LimitsInitList const g_carLimitsBelarus = CarLimitsNoPassThroughLivingStreet();

VehicleModel::LimitsInitList const g_carLimitsBelgium = CarLimitsDefault();

VehicleModel::LimitsInitList const g_carLimitsBrazil = CarLimitsDefault();

VehicleModel::LimitsInitList const g_carLimitsDenmark =
{
  // No track
  {{"highway", "motorway"},       kSpeedMotorwayKMpH,      true},
  {{"highway", "motorway_link"},  kSpeedMotorwayLinkKMpH,  true},
  {{"highway", "trunk"},          kSpeedTrunkKMpH,         true},
  {{"highway", "trunk_link"},     kSpeedTrunkLinkKMpH,     true},
  {{"highway", "primary"},        kSpeedPrimaryKMpH,       true},
  {{"highway", "primary_link"},   kSpeedPrimaryLinkKMpH,   true},
  {{"highway", "secondary"},      kSpeedSecondaryKMpH,     true},
  {{"highway", "secondary_link"}, kSpeedSecondaryLinkKMpH, true},
  {{"highway", "tertiary"},       kSpeedTertiaryKMpH,      true},
  {{"highway", "tertiary_link"},  kSpeedTertiaryLinkKMpH,  true},
  {{"highway", "residential"},    kSpeedResidentialKMpH,   true},
  {{"highway", "unclassified"},   kSpeedUnclassifiedKMpH,  true},
  {{"highway", "service"},        kSpeedServiceKMpH,       true},
  {{"highway", "living_street"},  kSpeedLivingStreetKMpH,  true},
  {{"highway", "road"},           kSpeedRoadKMpH,          true}
};

VehicleModel::LimitsInitList const g_carLimitsFrance = CarLimitsDefault();

VehicleModel::LimitsInitList const g_carLimitsFinland = CarLimitsDefault();

VehicleModel::LimitsInitList const g_carLimitsGermany =
{
  // No pass through track
  {{"highway", "motorway"},       kSpeedMotorwayKMpH,      true},
  {{"highway", "motorway_link"},  kSpeedMotorwayLinkKMpH,  true},
  {{"highway", "trunk"},          kSpeedTrunkKMpH,         true},
  {{"highway", "trunk_link"},     kSpeedTrunkLinkKMpH,     true},
  {{"highway", "primary"},        kSpeedPrimaryKMpH,       true},
  {{"highway", "primary_link"},   kSpeedPrimaryLinkKMpH,   true},
  {{"highway", "secondary"},      kSpeedSecondaryKMpH,     true},
  {{"highway", "secondary_link"}, kSpeedSecondaryLinkKMpH, true},
  {{"highway", "tertiary"},       kSpeedTertiaryKMpH,      true},
  {{"highway", "tertiary_link"},  kSpeedTertiaryLinkKMpH,  true},
  {{"highway", "residential"},    kSpeedResidentialKMpH,   true},
  {{"highway", "unclassified"},   kSpeedUnclassifiedKMpH,  true},
  {{"highway", "service"},        kSpeedServiceKMpH,       true},
  {{"highway", "living_street"},  kSpeedLivingStreetKMpH,  true},
  {{"highway", "road"},           kSpeedRoadKMpH,          true},
  {{"highway", "track"},          kSpeedTrackKMpH,         false}
};

VehicleModel::LimitsInitList const g_carLimitsHungary = CarLimitsNoPassThroughLivingStreet();

VehicleModel::LimitsInitList const g_carLimitsIceland = CarLimitsDefault();

VehicleModel::LimitsInitList const g_carLimitsNetherlands = CarLimitsDefault();

VehicleModel::LimitsInitList const g_carLimitsNorway = CarLimitsDefault();

VehicleModel::LimitsInitList const g_carLimitsOman = CarLimitsDefault();

VehicleModel::LimitsInitList const g_carLimitsPoland = CarLimitsDefault();

VehicleModel::LimitsInitList const g_carLimitsRomania = CarLimitsNoPassThroughLivingStreet();

VehicleModel::LimitsInitList const g_carLimitsRussia = CarLimitsNoPassThroughLivingStreetAndService();

VehicleModel::LimitsInitList const g_carLimitsSlovakia = CarLimitsNoPassThroughLivingStreet();

VehicleModel::LimitsInitList const g_carLimitsSpain = CarLimitsDefault();

VehicleModel::LimitsInitList const g_carLimitsSwitzerland = CarLimitsDefault();

VehicleModel::LimitsInitList const g_carLimitsTurkey = CarLimitsDefault();

VehicleModel::LimitsInitList const g_carLimitsUkraine = CarLimitsNoPassThroughLivingStreetAndService();

VehicleModel::LimitsInitList const g_carLimitsUK = CarLimitsDefault();

VehicleModel::LimitsInitList const g_carLimitsUS = CarLimitsDefault();

static vector<VehicleModel::AdditionalRoadTags> const & AdditionalTags()
{
  static vector<VehicleModel::AdditionalRoadTags> const kAdditionalTags = {
    // {{highway tags}, {weightSpeed, etaSpeed}}
    {{"route", "ferry", "motorcar"}, kSpeedFerryMotorcarKMpH},
    {{"route", "ferry", "motor_vehicle"}, kSpeedFerryMotorcarVehicleKMpH},
    {{"railway", "rail", "motor_vehicle"}, kSpeedRailMotorcarVehicleKMpH},
    {{"route", "shuttle_train"}, kSpeedShuttleTrainKMpH},
    {{"route", "ferry"}, kSpeedFerryMotorcarKMpH},
    {{"man_made", "pier"}, kSpeedPierKMpH}
  };

  std::ofstream output("/sdcard/MapsWithMe/outputMisha", std::ofstream::app);
  output << "kAdditionalTags.size() = " << kAdditionalTags.size() << std::endl;
  for (auto const & tag : kAdditionalTags)
  {
    output << "[ ";
    for (auto const & val : tag.m_hwtag)
    {
      output << val << ", ";
    }
    output << "]" << std::endl;
  }
  
  return kAdditionalTags;
}

VehicleModel::SurfaceInitList const g_carSurface = {
  // {{surfaceType, surfaceType}, {weightFactor, etaFactor}}
  {{"psurface", "paved_good"}, {1.0, 1.0}},
  {{"psurface", "paved_bad"}, {0.5, 0.5}},
  {{"psurface", "unpaved_good"}, {0.8, 0.8}},
  {{"psurface", "unpaved_bad"}, {0.3, 0.3}}
};
 // namespace

namespace routing
{

CarModel::CarModel()
  : VehicleModel(classif(), CarLimitsDefault(), g_carSurface)
{
  InitAdditionalRoadTypes();
}

CarModel::CarModel(VehicleModel::LimitsInitList const & roadLimits)
  : VehicleModel(classif(), roadLimits, g_carSurface)
{
  InitAdditionalRoadTypes();
}

SpeedKMpH CarModel::GetSpeed(FeatureType & f, SpeedParams const & speedParams) const
{
  if (!speedParams.m_maxspeed.IsValid())
    return VehicleModel::GetSpeed(f, speedParams);

  // Note. It's the first rough attempt using maxspeed tag value for speed calculation.
  // It's used as a feature speed if it's valid and less then some value.
  // @TODO maxspeed tag value should be used more sophisticated.
  uint16_t const maxspeedBasedSpeedKmPH = speedParams.m_maxspeed.GetSpeedKmPH(speedParams.m_forward);
  auto const speedKmPH = min(static_cast<double>(maxspeedBasedSpeedKmPH), GetMaxWeightSpeed());
  return {speedKmPH /* weight */, speedKmPH /* eta */};
}

double CarModel::GetOffroadSpeed() const { return kSpeedOffroadKMpH; }
#include <fstream>
void CarModel::InitAdditionalRoadTypes()
{
  std::ofstream output("/sdcard/MapsWithMe/outputMisha", std::ofstream::app);
  output << "AdditionalTags().size() = " << AdditionalTags().size() << std::endl;
  for (auto const & tag : AdditionalTags())
  {
    output << "[ ";
    for (auto const & val : tag.m_hwtag)
    {
      output << val << ", ";
    }
    output << tag.m_speed.m_inCity.m_weight;
    output << "]" << std::endl;
  }
  SetAdditionalRoadTypes(classif(), AdditionalTags());
}

// static
CarModel const & CarModel::AllLimitsInstance()
{
  static CarModel const instance;
  return instance;
}

// static
routing::VehicleModel::LimitsInitList const & CarModel::GetLimits() { return CarLimitsDefault(); }

// static
vector<routing::VehicleModel::AdditionalRoadTags> const & CarModel::GetAdditionalTags()
{
  return AdditionalTags();
}

// static
VehicleModel::SurfaceInitList const & CarModel::GetSurfaces() { return g_carSurface; }

CarModelFactory::CarModelFactory(CountryParentNameGetterFn const & countryParentNameGetterFn)
  : VehicleModelFactory(countryParentNameGetterFn)
{
  // Names must be the same with country names from countries.txt
  m_models[""] = make_shared<CarModel>(CarLimitsDefault());
  m_models["Australia"] = make_shared<CarModel>(g_carLimitsAustralia);
  m_models["Austria"] = make_shared<CarModel>(g_carLimitsAustria);
  m_models["Belarus"] = make_shared<CarModel>(g_carLimitsBelarus);
  m_models["Belgium"] = make_shared<CarModel>(g_carLimitsBelgium);
  m_models["Brazil"] = make_shared<CarModel>(g_carLimitsBrazil);
  m_models["Denmark"] = make_shared<CarModel>(g_carLimitsDenmark);
  m_models["France"] = make_shared<CarModel>(g_carLimitsFrance);
  m_models["Finland"] = make_shared<CarModel>(g_carLimitsFinland);
  m_models["Germany"] = make_shared<CarModel>(g_carLimitsGermany);
  m_models["Hungary"] = make_shared<CarModel>(g_carLimitsHungary);
  m_models["Iceland"] = make_shared<CarModel>(g_carLimitsIceland);
  m_models["Netherlands"] = make_shared<CarModel>(g_carLimitsNetherlands);
  m_models["Norway"] = make_shared<CarModel>(g_carLimitsNorway);
  m_models["Oman"] = make_shared<CarModel>(g_carLimitsOman);
  m_models["Poland"] = make_shared<CarModel>(g_carLimitsPoland);
  m_models["Romania"] = make_shared<CarModel>(g_carLimitsRomania);
  m_models["Russian Federation"] = make_shared<CarModel>(g_carLimitsRussia);
  m_models["Slovakia"] = make_shared<CarModel>(g_carLimitsSlovakia);
  m_models["Spain"] = make_shared<CarModel>(g_carLimitsSpain);
  m_models["Switzerland"] = make_shared<CarModel>(g_carLimitsSwitzerland);
  m_models["Turkey"] = make_shared<CarModel>(g_carLimitsTurkey);
  m_models["Ukraine"] = make_shared<CarModel>(g_carLimitsUkraine);
  m_models["United Kingdom"] = make_shared<CarModel>(g_carLimitsUK);
  m_models["United States of America"] = make_shared<CarModel>(g_carLimitsUS);
}
}  // namespace routing

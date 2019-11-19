#pragma once

#include "routing/edge_estimator.hpp"
#include "routing/index_graph.hpp"
#include "routing/route.hpp"
#include "routing/speed_camera_ser_des.hpp"
#include "routing/vehicle_mask.hpp"

#include "routing_common/num_mwm_id.hpp"
#include "routing_common/vehicle_model.hpp"

#include <memory>
#include <vector>

class MwmValue;
class DataSource;

namespace routing
{
class AllIndexGraphs
{
public:
  static AllIndexGraphs & GetAllIndexGraphsInstance();

  void Init();
};

class IndexGraphLoader
{
public:
  virtual ~IndexGraphLoader() = default;

  virtual Geometry & GetGeometry(NumMwmId numMwmId) = 0;
  virtual IndexGraph & GetIndexGraph(NumMwmId mwmId) = 0;
  virtual void LoadAll() {}

  // Because several cameras can lie on one segment we return vector of them.
  virtual std::vector<RouteSegment::SpeedCamera> GetSpeedCameraInfo(Segment const & segment) = 0;
  virtual void Clear() = 0;

  static std::unique_ptr<IndexGraphLoader> Create(
      VehicleType vehicleType, bool loadAltitudes, std::shared_ptr<NumMwmIds> numMwmIds,
      std::shared_ptr<VehicleModelFactoryInterface> vehicleModelFactory,
      std::shared_ptr<EdgeEstimator> estimator, DataSource & dataSource,
      RoutingOptions routingOptions = RoutingOptions());
};

class IndexGraphLoaderImpl final : public IndexGraphLoader
{
public:
  IndexGraphLoaderImpl(VehicleType vehicleType, bool loadAltitudes,
                       std::shared_ptr<NumMwmIds> numMwmIds,
                       std::shared_ptr<VehicleModelFactoryInterface> vehicleModelFactory,
                       std::shared_ptr<EdgeEstimator> estimator, DataSource & dataSource,
                       RoutingOptions routingOptions = RoutingOptions());

  // IndexGraphLoader overrides:
  Geometry & GetGeometry(NumMwmId numMwmId) override;
  IndexGraph & GetIndexGraph(NumMwmId numMwmId) override;

  static void LoadGraph(std::shared_ptr<NumMwmIds> const & numMwmIds,
                        DataSource & dataSource,
                        std::shared_ptr<VehicleModelFactoryInterface> const & vehicleModelFactory,
                        std::shared_ptr<EdgeEstimator> const & estimator, bool loadAltitudes);

  struct GraphAttrs
  {
    std::shared_ptr<Geometry> m_geometry;
    std::unique_ptr<IndexGraph> m_indexGraph;
  };

  static thread_local std::unordered_map<NumMwmId, GraphAttrs> kGraphs;

  void LoadAll() override;
  std::vector<RouteSegment::SpeedCamera> GetSpeedCameraInfo(Segment const & segment) override;
  void Clear() override;

private:
  GraphAttrs & CreateGeometry(NumMwmId numMwmId);
  GraphAttrs & CreateIndexGraph(NumMwmId numMwmId, GraphAttrs & graph);

  VehicleType m_vehicleType;
  bool m_loadAltitudes;
  DataSource & m_dataSource;
  std::shared_ptr<NumMwmIds> m_numMwmIds;
  std::shared_ptr<VehicleModelFactoryInterface> m_vehicleModelFactory;
  std::shared_ptr<EdgeEstimator> m_estimator;
  bool m_loadAll = false;

  std::unordered_map<NumMwmId, GraphAttrs> m_graphs;

  std::unordered_map<NumMwmId, std::map<SegmentCoord, std::vector<RouteSegment::SpeedCamera>>>
      m_cachedCameras;
  decltype(m_cachedCameras)::iterator ReceiveSpeedCamsFromMwm(NumMwmId numMwmId);

  RoutingOptions m_avoidRoutingOptions = RoutingOptions();
};

void DeserializeIndexGraph(MwmValue const & mwmValue, VehicleType vehicleType, IndexGraph & graph);
}  // namespace routing

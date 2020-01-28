#pragma once

#include "poly_borders/help_structures.hpp"

#include "base/control_flow.hpp"

#include <map>
#include <string>
#include <vector>

namespace poly_borders
{
class BordersData
{
public:
  inline static double const kEqualityEpsilon = 1e-20;
  inline static std::string const kBorderExtension = ".poly";

  void Init(std::string const & bordersDir);
  /// \brief Runs |MarkPoint(borderId, pointId)| for each borderId and it's pointId. See to
  /// |MarkPoint| for more details.
  void MarkPoints();

  void RemoveEmptySpaceBetweenBorders();

  void DumpPolyFiles(std::string const & targetDir);
  Polygon const & GetBordersPolygonByName(std::string const & name) const;
  void PrintDiff();

private:
  struct Processor
  {
    explicit Processor(BordersData & data) : m_data(data) {}
    void operator()(size_t borderId);

    BordersData & m_data;
  };

  /// \brief Some polygons can have sequentially same points - duplicates. This method removes such
  /// points and leaves only unique.
  void RemoveDuplicatePoints();

  /// \brief Finds point on other polygons equals to point passed on argument. If such point is
  /// found, the method will create link "some border (with id = anotherBorderId) has the same point
  /// (with id = anotherPointId").
  void MarkPoint(size_t curBorderId, size_t curPointId);

  /// \brief Checks whether we could replace points from segment: [curLeftPointId, curRightPointId]
  /// of |curBorderId| to another points from another border in order to get rid of empty space
  /// between curBorder and anotherBorder.
  base::ControlFlow TryToReplace(size_t curBorderId, size_t & curLeftPointId,
                                 size_t curRightPointId);

  bool HasLinkAt(size_t curBorderId, size_t pointId);

  /// \brief Replace points using |Polygon::ReplaceData| that filled after
  /// |RemoveEmptySpaceBetweenBorders()|.
  void DoReplace();

  size_t m_removedPointsCount = 0;
  size_t m_dublicatedPointsCount = 0;
  std::map<size_t, double> m_additionalAreaMetersSqr;

  std::map<std::string, size_t> m_polyFileNameToIndex;
  std::map<size_t, std::string> m_indexToPolyFileName;
  std::vector<Polygon> m_bordersPolygons;
  std::vector<Polygon> m_prevCopy;
};
}  // namespace poly_borders

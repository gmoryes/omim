#pragma once

#include "poly_borders/help_structures.hpp"

#include <map>
#include <string>
#include <vector>

namespace poly_borders
{
class BordersData
{
public:
  static double const kEqualityEpsilon;
  static std::string const kBorderExtension;

  void Init(std::string const & bordersDir);
  void MarkPoints();

  void RemoveEmptySpaceBetweenBorders();

  void DumpPolyFiles(std::string const & targetDir);

  std::vector<Polygon> const & GetBordersPolygon() const { return m_bordersPolygons; }
  Polygon const & GetBordersPolygonByName(std::string const & name) const;

  void PrintDiff();

private:
  struct Processor
  {
    explicit Processor(BordersData & data) : m_data(data) {}
    void operator()(size_t borderId);

    BordersData & m_data;
  };

  void RemoveDuplicatePoints();

  void MarkPoint(size_t curBorderId, MarkedPoint & point, int pointId);

  size_t m_numberOfAddPoints = 0;
  std::map<size_t, double> m_additionalAreaMetersSqr;
  void DoReplace();

  std::map<std::string, size_t> m_nameToIndex;
  std::map<size_t, std::string> m_indexToName;
  std::vector<Polygon> m_bordersPolygons;
  std::vector<Polygon> m_prevCopy;
};
}  // namespace poly_borders

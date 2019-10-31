#pragma once

#include <string>

namespace generator
{
class RoutingCityBoundariesProcessor
{
public:
  explicit RoutingCityBoundariesProcessor(std::string filename);

  void ProcessDataFromCollector();
  void DumpBoundaries(std::string const & dumpFilename);

private:
  std::string m_filename;
};
}  // namespace generator

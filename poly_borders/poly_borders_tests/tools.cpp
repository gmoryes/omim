#include "poly_borders/poly_borders_tests/tools.hpp"

#include "poly_borders/borders_data.hpp"

#include "platform/platform_tests_support/scoped_file.hpp"

#include "geometry/point2d.hpp"

#include "base/file_name_utils.hpp"

#include <string>
#include <vector>

using namespace platform::tests_support;

namespace poly_borders
{
std::shared_ptr<ScopedFile>
CreatePolyBorderFileByPolygon(std::string const & relativeDirPath,
                              std::string const & name,
                              std::vector<std::vector<m2::PointD>> const & polygons)
{
  std::string const fullName = name + BordersData::kBorderExtension;
  std::string path = base::JoinPath(relativeDirPath, fullName);

  auto file = std::make_shared<ScopedFile>(path, ScopedFile::Mode::Create);

  BordersData::DumpPolyFile(file->GetFullPath(), fullName, polygons);

  return file;
}
}  // namespace poly_borders

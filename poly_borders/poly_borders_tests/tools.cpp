#include "poly_borders/poly_borders_tests/tools.hpp"

#include "poly_borders/borders_data.hpp"

#include "generator/borders.hpp"

#include "platform/platform.hpp"

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
  std::string path = base::JoinPath(relativeDirPath, name + BordersData::kBorderExtension);

  auto file = std::make_shared<ScopedFile>(path, ScopedFile::Mode::Create);

  auto const targetDir = base::GetDirectory(file->GetFullPath());
  borders::DumpBorderToPolyFile(targetDir, name, polygons);

  return file;
}
}  // namespace poly_borders

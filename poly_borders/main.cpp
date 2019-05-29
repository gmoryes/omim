#include "poly_borders/src/borders_data.hpp"
#include "poly_borders/src/help_structures.hpp"

#include "platform/platform.hpp"

#include "base/assert.hpp"
#include "base/exception.hpp"
#include "base/logging.hpp"

#include <exception>

#include "3party/gflags/src/gflags/gflags.h"

DEFINE_string(borders_path, "", "Path to directory with *.poly files.");
DEFINE_string(output_path, "", "Path to target directory where will be placed *.poly files.");

using namespace poly_borders;

int main(int argc, char ** argv)
{
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::SetUsageMessage("\n\n\tThe tool is used to process *.poly borders files. We use such files\n"
                          "\tfor \"planet to mwms separation\" in generator. The problem is that we have\n"
                          "\tempty spaces between neighbouring borders. This tool create new borders\n"
                          "\tbased on input data by adding new points to borders in this way that the\n"
                          "\tchanged area of each border will not too much.\n"
                          "\tArguments:\n"
                          "\t\t--borders_path=/path/to/directory/with/borders\n"
                          "\t\t--output_path=/path/to/directory/where/new/borders/will/be/placed\n");

  if (FLAGS_borders_path.empty() || FLAGS_output_path.empty())
  {
    google::ShowUsageWithFlags("poly_borders_postprocessor");
    return 0;
  }

  CHECK(Platform::IsDirectory(FLAGS_borders_path), ("Can not find directory:", FLAGS_borders_path));
  CHECK(Platform::IsDirectory(FLAGS_output_path), ("Can not find directory:", FLAGS_output_path));

  try
  {
    BordersData data;
    data.Init(FLAGS_borders_path);
    data.MarkPoints();
    data.RemoveEmptySpaceBetweenBorders();
    data.MarkPoints();
    data.PrintDiff();
    data.DumpPolyFiles(FLAGS_output_path);
  }
  catch (RootException const & e)
  {
    LOG(LINFO, ("Core exception:", e.Msg()));
  }
  catch (std::exception const & e)
  {
    LOG(LINFO, ("Std exception:", e.what()));
  }
  catch (...)
  {
    LOG(LINFO, ("Unknown exception."));
  }

  return 0;
}

#pragma once

#include "base/macros.hpp"

#include <fstream>
#include <string>

namespace base
{
class AllocatorProfilerLogsProducer
{
public:
  AllocatorProfilerLogsProducer();

  static AllocatorProfilerLogsProducer & Instance();

  std::vector<std::string> GetLastBatchOfLogs(std::string const & blockName);

private:
  std::string m_logFile;
  std::ifstream m_logFileInput;
};

class AllocatorProfiler
{
public:
  AllocatorProfiler(std::string const & blockName, AllocatorProfilerLogsProducer & logsProducer);
  ~AllocatorProfiler();

  AllocatorProfiler(AllocatorProfiler &&) = delete;
  AllocatorProfiler(AllocatorProfiler const &) = delete;

private:

  size_t Find(std::vector<std::string> const & strings, std::vector<std::string> const & lines, bool first);
  static double RetrieveMemoryUsage(std::string const & line);

  std::string const & m_BlockName;
  AllocatorProfilerLogsProducer & m_logsProducer;
};
}  // namespace base
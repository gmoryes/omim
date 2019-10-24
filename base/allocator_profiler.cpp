#include "base/allocator_profiler.hpp"

#include "base/assert.hpp"
#include "base/logging.hpp"
#include "base/string_utils.hpp"

#include <cctype>
#include <limits>

namespace base
{
// static
AllocatorProfilerLogsProducer & AllocatorProfilerLogsProducer::Instance()
{
  static AllocatorProfilerLogsProducer instance;
  return instance;
}

AllocatorProfilerLogsProducer::AllocatorProfilerLogsProducer()
{
  CHECK(std::getenv("LOG_FILE"), ("Can not find LOG_FILE env"));
  if (m_logFile.empty())
    m_logFile = std::string(std::getenv("LOG_FILE"));

  m_logFileInput.open(m_logFile);
  CHECK(m_logFileInput.good(), ("Can not open:", m_logFile));
}

std::vector<std::string> AllocatorProfilerLogsProducer::GetLastBatchOfLogs(
    std::string const & blockName)
{
  std::string line;
  auto const end = [&line, &blockName]() {
    bool const hasEnd = line.find("End profile") != std::string::npos;
    bool const hasBlockName = line.find(blockName) != std::string::npos;

    return hasEnd && hasBlockName;
  };

  std::vector<std::string> lines;
  do
  {
    std::getline(m_logFileInput, line);
    LOG_FORCE(LINFO, ("read:", line, "end:", end()));
    lines.emplace_back(std::move(line));
  } while (!end());

  LOG_FORCE(LINFO, ("return:", lines.size()));
  return lines;
}

AllocatorProfiler::AllocatorProfiler(std::string const & blockName,
                                     AllocatorProfilerLogsProducer & logsProducer)
  : m_BlockName(blockName), m_logsProducer(logsProducer)
{
  LOG_FORCE(LINFO, ("Start profile:", m_BlockName));
}

size_t AllocatorProfiler::Find(std::vector<std::string> const & strings,
                               std::vector<std::string> const & lines, bool first)
{
  if (lines.empty())
    return 0;

  auto const found = [&strings](std::string const & line)
  {
    for (auto const & str : strings)
    {
      if (line.find(str) == std::string::npos)
        return false;
    }
    return true;
  };

  size_t resultIndex = first ? 0 : lines.size() - 1;
  int32_t step = first ? 1 : -1;
  for (; resultIndex < lines.size(); resultIndex += step)
  {
    if (found(lines[resultIndex]))
      return resultIndex;
  }

  return lines.size();
}

// static
double AllocatorProfiler::RetrieveMemoryUsage(std::string const & line)
{
  // Dumping heap profile to /tmp/mybin.hprof.0150.heap (2449 MB allocated cumulatively, 102 MB currently in use)
  auto const readNumber = [&line](size_t & i) {
    uint64_t n = 0.0;
    while (i < line.size() && isdigit(line[i]))
    {
      n *= 10;
      n += line[i] - '0';

      ++i;
    }

    return static_cast<double>(n);
  };

  auto const tryRead = [&line](size_t pos, std::string const & templ) {
    for (size_t i = pos; i < line.size(); ++i)
    {
      if (line[i] != templ[i - pos])
        return false;
    }
    return true;
  };

  for (size_t i = 0; i < line.size(); ++i)
  {
    if (isdigit(line[i]))
    {
      double n = readNumber(i);
      if (tryRead(i, " MB currently in use"))
        return n;
    }
  }

  return std::numeric_limits<double>::max();
}

AllocatorProfiler::~AllocatorProfiler()
{
  LOG_FORCE(LINFO, ("End profile:", m_BlockName));
  std::vector<std::string> lines = m_logsProducer.GetLastBatchOfLogs(m_BlockName);

  size_t startIndex = Find({"Start profile", m_BlockName}, lines, true /* first */);
  size_t endIndex = Find({"End profile", m_BlockName}, lines, false /* first */);
  CHECK_NOT_EQUAL(startIndex, lines.size(), ());
  CHECK_NOT_EQUAL(endIndex, lines.size(), ());
  CHECK_LESS(startIndex, endIndex, ());

  size_t const baseMemoryPoint = Find({"currently in use"}, lines, true /* first */);
  if (baseMemoryPoint == lines.size())
  {
    LOG_FORCE(LINFO, (m_BlockName, "usage:", 0));
    return;
  }

  double const start = RetrieveMemoryUsage(lines[baseMemoryPoint]);
  LOG_FORCE(LINFO, ("startIndex:", startIndex, "endIndex:", endIndex, "base:", start));
  double end = start;
  for (size_t i = startIndex; i < endIndex; ++i)
  {
    if (lines[i].find("currently in use") == std::string::npos)
      continue;

    end =  RetrieveMemoryUsage(lines[i]);
    CHECK_NOT_EQUAL(end, std::numeric_limits<double>::max(), (lines[i]));
  }

  LOG_FORCE(LINFO, (m_BlockName, "usage:", end - start, "MB"));
}
} // namespace base
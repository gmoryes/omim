#pragma once

#include "shaders/program_pool.hpp"

#include "drape/vulkan/vulkan_device_holder.hpp"
#include "drape/vulkan/vulkan_gpu_program.hpp"
#include "drape/pointers.hpp"

#include <vulkan_wrapper.h>
#include <vulkan/vulkan.h>

#include <array>

namespace gpu
{
namespace vulkan
{
class VulkanProgramPool : public ProgramPool
{
public:
  explicit VulkanProgramPool(dp::vulkan::DeviceHolderPtr deviceHolder);
  ~VulkanProgramPool() override;

  drape_ptr<dp::GpuProgram> Get(Program program) override;

private:
  dp::vulkan::DeviceHolderPtr m_deviceHolder;
  std::array<drape_ptr<dp::vulkan::VulkanGpuProgram>,
             static_cast<size_t>(Program::ProgramsCount)> m_programs;
};
}  // namespace vulkan
}  // namespace gpu

#include "fdcan_bus.hpp"
#include "../impl/fdcan_bus_impl.hpp"

namespace HAL::FDCAN
{

// 全局函数实现
IFdcanBus &get_fdcan_bus_instance()
{
    return FdcanBus::instance();
}

} // namespace HAL::FDCAN

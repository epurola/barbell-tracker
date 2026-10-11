#pragma once 

#include <cstdint>

namespace vbt::runtime
{
    constexpr uint64_t acquisitionDeadlineUs(uint64_t epochUs, uint32_t sequence, uint64_t periodUs) noexcept
    {
        return epochUs + static_cast<uint64_t>(sequence) * periodUs;
    }

    // If multiple timer notifications accumulated before the task ran,
    // all but one represent missed opportunities to service a deadline.
    constexpr uint32_t missedNotifications(uint32_t notificationCount) noexcept
    {
        return notificationCount > 0 ? notificationCount -1 : 0;
    }
    
}
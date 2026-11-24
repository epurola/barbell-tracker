#pragma once

#include <cstdint>

#include "contracts/ModeTypes.h"

namespace vbt::runtime
{
    struct __attribute__((packed)) GatePacket
    {
        uint8_t type; // 2 = MSG_GATE_TRIGGER
    };

    class GateReceiver
    {
    public:
        static constexpr uint8_t WIFI_CHANNEL = 1;
        static constexpr uint8_t MSG_GATE_TRIGGER = 2;
        static constexpr uint64_t MIN_GATE_INTERVAL_US = 200000; // 200 ms dedupe for 3-burst packets

        void begin();
        void end();
        void reset();
        vbt::GateSplitMetrics snapshot() const;
        bool isRunning() const { return running_; }

    private:
        bool running_{false};
    };

    GateReceiver& gateReceiver();
}

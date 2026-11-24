#pragma once

#include <Arduino.h>
#include <Wire.h>

#include <cstdint>

#include "contracts/ImuTypes.h"

namespace vbt::imu
{
    class Lsm6ds3
    {
        public:
            static constexpr uint8_t DEFAULT_ADDRESS = 0x6A;
            static constexpr uint8_t EXPECTED_WHO_AM_I = 0x69;

            explicit Lsm6ds3(uint8_t address = DEFAULT_ADDRESS) : address_(address) {}

            bool begin(TwoWire& wire);
            bool readSample(vbt::RawImuSample& sample, uint64_t timestampUs, uint32_t sequence);
            void sleep();
            bool isInitialized() const { return initialized_; }
            uint8_t whoAmI() const { return whoAmI_; }


        private:
            bool readRegister(uint8_t reg, uint8_t& value);
            bool writeRegister(uint8_t reg, uint8_t value);
            bool readRegisters(uint8_t firstRegister, uint8_t* destination, size_t length);
            TwoWire* wire_{nullptr};
            uint8_t address_{DEFAULT_ADDRESS};
            uint8_t whoAmI_{0};
            bool initialized_{false};

    };
}
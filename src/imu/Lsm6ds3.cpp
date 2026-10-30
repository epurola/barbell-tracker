#include "imu/Lsm6ds3.h"

#include <cmath>

namespace
{
    constexpr uint8_t REG_WHO_AM_I = 0x0f;
    constexpr uint8_t REG_CTRL1_XL = 0x10;
    constexpr uint8_t REG_CTLR2_G  = 0x11;
    constexpr uint8_t REG_CTLR3_C  = 0x12;
    constexpr uint8_t REG_STATUS   = 0x1E;
    constexpr uint8_t REG_OUTX_L_G = 0x22;

    constexpr uint8_t STATUS_XLDA = 1u << 0;
    constexpr uint8_t STATUS_GDA  = 1u << 1;

    // LSM6DS3 configured ranges:
    // Accelerometer: +/-4 g, 0.122 mg/LSB.
    // Gyroscope:     +/-2000 degrees/s, 70 mdps/LSB.
    constexpr float ACCEL_MPS2_PER_LSB = 0.000122f * 9.80655f;
    constexpr float GYRO_RAPDPS_PER_LSB = 0.070f * 0.017453292519943295f;
  

    int16_t decodeInt16(uint8_t low, uint8_t high)
    {
        const uint16_t bits = static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8);
        return static_cast<int16_t>(bits);
    }
}

namespace vbt::imu
{
    bool Lsm6ds3::begin(TwoWire& wire)
    {
        initialized_ = false;
        wire_ = &wire;

        uint8_t identity = 0;
        if(!readRegister(REG_WHO_AM_I, identity)) return false;

        whoAmI_ = identity;
        if(whoAmI_ != EXPECTED_WHO_AM_I) return false;

        // Enable block-data update and register auto-increment
        if(!writeRegister(REG_CTLR3_C, 0x44)) return false;

        // Accelerometer:
        // ODR = 833 Hz, full scale = +/-4 g
        if(!writeRegister(REG_CTRL1_XL, 0x78)) return false;

        // Gyroscope:
        // ODR = 833 Hz, full scale = +/-2000 degrees/s
        if(!writeRegister(REG_CTLR2_G, 0x7C)) return false;

        delay(10);

        // Verify that both sensors were configured as requested
        uint8_t accelConfig = 0;
        uint8_t gyroConfig = 0;

        if(!readRegister(REG_CTRL1_XL, accelConfig) || !readRegister(REG_CTLR2_G, gyroConfig)) return false;
        if(accelConfig != 0x78 || gyroConfig != 0x7C) return false;

        initialized_ = true;
        return true;
    }

    bool Lsm6ds3::readSample(vbt::RawImuSample& sample, uint64_t timestampUs, uint32_t sequence)
    {
        if(!initialized_ || wire_ == nullptr) return false;

        // Require fresh data from both sensors.
        uint8_t status = 0;
        if(!readRegister(REG_STATUS, status)) return false;

        if((status & (STATUS_XLDA | STATUS_GDA)) != (STATUS_XLDA | STATUS_GDA)) return false;

        // Register order:
        // gyro X/Y/Z, followed by accelerometer X/Y/Z.'

        uint8_t data[12]{};
        if(!readRegisters(REG_OUTX_L_G, data, sizeof(data))) return false;

        const int16_t gyroX = decodeInt16(data[0], data[1]);
        const int16_t gyroY = decodeInt16(data[2], data[3]);
        const int16_t gyroZ = decodeInt16(data[4], data[5]);

        const int16_t accelX = decodeInt16(data[6], data[7]);
        const int16_t accelY = decodeInt16(data[8], data[9]);
        const int16_t accelZ = decodeInt16(data[10], data[11]);

        sample.timestamp_us = timestampUs;
        sample.sequence = sequence;

         // Convert to SI units. These are sensor-scaled measurements,
         // not yet bias-corrected or calibrated.
         sample.accelerometer = vbt::Vector3(accelX * ACCEL_MPS2_PER_LSB, accelY * ACCEL_MPS2_PER_LSB, accelZ * ACCEL_MPS2_PER_LSB);
         sample.gyroscope = vbt::Vector3(gyroX * GYRO_RAPDPS_PER_LSB, gyroY * GYRO_RAPDPS_PER_LSB, gyroZ * GYRO_RAPDPS_PER_LSB);
         sample.status = vbt::SampleStatus::Valid | 
                         vbt::SampleStatus::AccelorometerOk |
                         vbt::SampleStatus::GyroscopeOk | 
                         vbt::SampleStatus::TimestampValid |
                         vbt::SampleStatus::SequenceValid;
        return true;
    }

    bool Lsm6ds3::readRegister(uint8_t reg, uint8_t& value)
    {
        if(wire_ == nullptr) return false;

        wire_->beginTransmission(address_);
        wire_->write(reg);

        if(wire_->endTransmission(false) != 0) return false;

        const size_t received = wire_->requestFrom(address_, static_cast<size_t>(1), true);

        if(received != 1 || !wire_->available()) return false;

        value = static_cast<uint8_t>(wire_->read());
        return true;
    }

    bool Lsm6ds3::writeRegister(uint8_t reg, uint8_t value)
    {
        if(wire_ == nullptr) return false;

        wire_->beginTransmission(address_);
        wire_->write(reg);
        wire_->write(value);

        return wire_->endTransmission() == 0;
    }

    bool Lsm6ds3::readRegisters(uint8_t firstRegister, uint8_t* destination, size_t length)
    {
        if(wire_ == nullptr || destination == nullptr || length == 0) return false;

        wire_->beginTransmission(address_);
        wire_->write(firstRegister);

        if(wire_->endTransmission(false) != 0) return false;

        const size_t received = wire_->requestFrom(address_, length, true);

        if(received != length)
        {
            while(wire_->available()) wire_->read();
            return false;
        }

        for (size_t i = 0; i < length; ++i)
        {
            if(!wire_->available()) return false;
            
            destination[i] = static_cast<uint8_t>(wire_->read());
        }
        return true;

    }

}

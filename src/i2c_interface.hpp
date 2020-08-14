#pragma once

#include "hw_interface_helpers.hpp"

#include <chrono>
#include <vector>
#include <functional>
#include <thread>
#include <optional>

namespace i2c_interface
{

  class I2CInterface {
  public:
    I2CInterface(const uint_fast32_t &bits_per_second = 100000) :
      bits_per_second_(bits_per_second) {
      delay_ns_ = std::chrono::nanoseconds(500000000) / bits_per_second;
    }

    virtual void write(const uint_fast16_t &address, const std::vector<uint8_t> &data) const = 0;
    virtual std::vector<uint8_t> read(const uint_fast16_t &address, const uint_fast16_t &length) const = 0;
    virtual std::vector<uint8_t> read_eeprom(const uint_fast16_t &device_address,
                                             const uint_fast16_t &memory_address,
                                             const uint_fast16_t &length) const = 0;

  protected:
    void delay(void) const;
    uint_fast32_t bits_per_second_;
    std::chrono::nanoseconds delay_ns_;
  };

  void write(const hw_interface::stream_id &stream_id, const uint32_t &address,
             const uint32_t &value, const uint8_t &length,
             const hw_interface::access_width &access_width,
             std::ostream &out);

  std::optional<uint32_t> read(const hw_interface::stream_id &stream_id,
                               const uint32_t &address, const uint8_t &length,
                               const hw_interface::access_width &access_width,
                               std::ostream &out);

  enum i2c_interfaces {
                       FE = 0,
                       REF = 1,
  };

  extern const std::array<std::reference_wrapper<const I2CInterface>, 2> i2c_bitbangers;

} // namespace i2c_interface

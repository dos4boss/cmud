#pragma once

#include "hw_interface_helpers.hpp"

#include <chrono>
#include <inttypes.h>
#include <vector>
#include <functional>
#include <thread>
#include <type_traits>
#include <optional>

template <typename T,
          bool big_endian = true,
          typename std::enable_if<std::is_arithmetic<T>::value>::type * = nullptr>
std::vector<uint8_t> to_byte_vector(T const &value) {
  std::vector<uint8_t> bytes;

  for (size_t i = 0; i < sizeof(value); i++) {
    uint8_t byte = value >> (i * 8);
    if constexpr (big_endian) {
      bytes.insert(bytes.begin(), byte);
    }
    else {
      bytes.insert(bytes.end(), byte);
    }
  }

  return bytes;
}

template <typename T,
          bool big_endian = true,
          typename std::enable_if<std::is_arithmetic<T>::value>::type * = nullptr>
T from_byte_vector(const std::vector<uint8_t> &bytes) {
  if(sizeof(T) != bytes.size())
    throw std::runtime_error("Byte length is not matching size requested of type");
  T result{0};
  for (size_t i = 0; i < sizeof(T); i++) {
    if constexpr (big_endian) {
        result |= bytes[i];
    } else {
      result |= bytes[sizeof(T) - i - 1];
    }
    if (i != sizeof(T) - 1)
      result <<= 8;
  }
  return result;
}

namespace i2c_interface
{

  class I2CInterface {
  public:
    I2CInterface(const uint_fast32_t &bits_per_second = 100000) :
      bits_per_second_(bits_per_second) {
      delay_ns_ = std::chrono::nanoseconds(500000000) / bits_per_second;
    }

    virtual void write(const uint_fast16_t &address, const std::vector<uint8_t> &data) = 0;
    virtual std::vector<uint8_t> read(const uint_fast16_t &address, const uint_fast16_t &length) = 0;

  protected:
    void delay(void);
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

  // if no readback of SCL line is supported, it can't react to clock
  // stretching
  struct DefaultSCLGetter {
    bool operator()(void) { return 1; }
  };

  template <class SDA_Setter, class SDA_Getter, class SCL_Setter,
            class SCL_Getter = DefaultSCLGetter, bool ArbitrationCheck = false>
  class I2CBitBang : I2CInterface {
  public:
    I2CBitBang(const uint_fast32_t &bits_per_second = 100000) {}

    void write(const uint_fast16_t &address,
               const std::vector<uint8_t> &data) override {
      if (!write_byte(true, false, address)) {
        bool nacks = false;
        for (const auto &byte : data) {
          if (!nacks)
            nacks = write_byte(false, false, byte);
          else
            break;
        }
        stop_cond();
        if (!nacks)
          return;
      } else
        stop_cond();
      throw std::runtime_error("I2C: Writing failed because of missing ACKs");
    }

    std::vector<uint8_t> read(const uint_fast16_t &address,
                              const uint_fast16_t &length) override {
      std::vector<uint8_t> result;
      result.reserve(length);

      if (!write_byte(true, false, address | 1)) {
        for (uint_fast16_t k; k < length; ++k) {
          const bool is_last = k == length - 1;
          result.push_back(read_byte(is_last, is_last));
        }
        return result;
      } else
        stop_cond();
      throw std::runtime_error("I2C: Reading failed because of missing ACK.");
    }

  private:
    void start_cond(void) {
      if (started_) {
        // if started, do a restart condition
        SDA_Setter{}(1);
        delay();
        SCL_Setter{}(1);

        // handle clock stretching if supported
        while (SCL_Getter{}() == 0) {
          std::this_thread::sleep_for(delay_ns_ / 10);
        }

        delay();
      }

      if constexpr (ArbitrationCheck) {
        if (SDA_Getter{}() == 0)
          throw std::runtime_error("I2C: Arbitration lost during start condition");
      }

      SDA_Setter{}(0);
      delay();
      SCL_Setter{}(0);
      started_ = true;
    }

    void stop_cond(void) {
      SDA_Setter{}(0);
      delay();
      SCL_Setter{}(1);

      // handle clock stretching if supported
      while (SCL_Getter{}() == 0) {
        std::this_thread::sleep_for(delay_ns_ / 10);
      }

      delay();
      SDA_Setter{}(1);
      delay();

      if constexpr (ArbitrationCheck) {
        if (SDA_Getter{}() == 0)
          throw std::runtime_error("I2C: Arbitration lost during stop condition");
      }

      started_ = false;
    }

    void write_bit(bool bit) {
      SDA_Setter{}(bit);
      delay();
      SCL_Setter{}(1);
      delay();

      // handle clock stretching if supported
      while (SCL_Getter{}() == 0) {
        std::this_thread::sleep_for(delay_ns_ / 10);
      }

      if constexpr (ArbitrationCheck) {
        if (bit && (SDA_Getter{}() == 0))
          throw std::runtime_error("I2C: Arbitration lost during bit writing");
      }
      SCL_Setter{}(0);
    }

    bool read_bit(void) {
      SDA_Setter{}(1);
      delay();
      SCL_Setter{}(1);

      // handle clock stretching if supported
      while (SCL_Getter{}() == 0) {
        std::this_thread::sleep_for(delay_ns_ / 10);
      }

      delay();
      auto result = SDA_Getter{}();
      SCL_Setter{}(0);
      return result;
    }

    bool write_byte(bool send_start, bool send_stop, uint_fast8_t byte) {
      bool nack;

      if (send_start)
        start_cond();

      for (uint_fast8_t bit = 0; bit < 8; ++bit) {
        write_bit((byte & 0x80) != 0);
        byte <<= 1;
      }

      nack = read_bit();

      if (send_stop)
        stop_cond();

      return nack;
    }

    uint_fast8_t read_byte(bool nack, bool send_stop) {
      uint_fast8_t byte = 0;

      for (uint_fast8_t bit = 0; bit < 8; ++bit) {
        byte = (byte << 1) | read_bit();
      }

      write_bit(nack);

      if (send_stop)
        stop_cond();

      return byte;
    }

    bool started_ = false;
  };

} // namespace i2c_interface

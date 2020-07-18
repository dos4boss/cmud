#pragma once

#include <chrono>
#include <inttypes.h>
#include <vector>
#include <functional>
#include <thread>

#include <iostream>

namespace i2c_interface
{

  class I2CInterface {
  public:
    I2CInterface(const uint_fast32_t &bits_per_second = 100000) :
      bits_per_second_(bits_per_second) {
      delay_ns_ = std::chrono::seconds(1) / bits_per_second;
      std::cout << "delay ns " << delay_ns_.count() << std::endl;
    }

    virtual void write(const uint_fast16_t &address, const std::vector<uint8_t> &data) = 0;
    virtual std::vector<uint8_t> read(const uint_fast16_t &address, const uint_fast16_t &length) = 0;

  protected:
    void delay(void);
    uint_fast32_t bits_per_second_;
    std::chrono::nanoseconds delay_ns_;
  };


  struct Accessor {
    std::function<void(bool)> setter;
    std::function<bool(void)> getter;
  };

  template <Accessor &SDA_Accessor, Accessor &SCL_Accessor>
  class I2CBitBang : I2CInterface {
  public:
    I2CBitBang(const uint_fast32_t &bits_per_second = 100000) {}

    void write(const uint_fast16_t &address, const std::vector<uint8_t> &data) override {
      if(!write_byte(true, false, address)) {
        bool nacks = false;
        for(const auto & byte : data) {
          if(!nacks)
            nacks = write_byte(byte);
          else
            break;
        }
        stop_cond();
        if(!nacks)
          return;
      }
      else
        stop_cond();
      throw std::runtime_error("I2C: Writing failed because of missing ACKs");
    }

    std::vector<uint8_t> read(const uint_fast16_t &address, const uint_fast16_t &length) override {
      std::vector<uint8_t> result;
      result.reserve(length);

      if (!write_byte(true, false, address | 1)) {
        for(uint_fast16_t k; k < length; ++k) {
          const bool is_last = k == length - 1;
          result.push_back(read_byte(is_last, is_last));
        }
        return result;
      }
      else
        stop_cond();
      throw std::runtime_error("I2C: Reading failed because of ");
    }

  private :

    void start_cond(void) {
      if (started_) {
        // if started, do a restart condition
        SDA_Accessor.setter(1);
        delay();
        SCL_Accessor.setter(1);

        // handle clock stretching if the accessor supports it
        if constexpr (SCL_Accessor.getter) {
            while (SCL_Accessor.getter() == 0) {
              std::this_thread::sleep_for(delay_ns_ / 10);
            }
          }

        delay();
      }

      if (SDA_Accessor.getter() == 0)
        throw std::runtime_error("I2C: Arbitration lost during start condition");

      SDA_Accessor.setter(0);
      delay();
      SCL_Accessor.setter(0);
      started_ = true;
    }

    void stop_cond(void) {
      SDA_Accessor.setter(0);
      delay();
      SCL_Accessor.setter(1);

      // handle clock stretching if the accessor supports it
      if constexpr (SCL_Accessor.getter) {
          while (SCL_Accessor.getter() == 0) {
            std::this_thread::sleep_for(delay_ns_ / 10);
          }
        }

      delay();
      SDA_Accessor.setter(1);
      delay();

      if (SDA_Accessor.getter() == 0)
        throw std::runtime_error("I2C: Arbitration lost during stop condition");

      started_ = false;
    }

    void write_bit(bool bit) {
      SDA_Accessor.setter(bit);
      delay();
      SCL_Accessor.setter(1);
      delay();
      // handle clock stretching if the accessor supports it
      if constexpr(SCL_Accessor.getter) {
          while (SCL_Accessor.getter() == 0) {
            std::this_thread::sleep_for(delay_ns_ / 10);
          }
        }

      if (bit && (SDA_Accessor.getter() == 0))
        throw std::runtime_error("I2C: Arbitration lost during bit writing");

      SCL_Accessor.setter(0);
    }

    bool read_bit(void) {
      SDA_Accessor.setter(1);
      delay();
      SCL_Accessor.setter(1);
      // handle clock stretching if the accessor supports it
      if constexpr(SCL_Accessor.getter) {
          while (SCL_Accessor.getter() == 0) {
            std::this_thread::sleep_for(delay_ns_ / 10);
          }
        }
      delay();
      auto result = SDA_Accessor.getter();
      SCL_Accessor.setter(0);
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

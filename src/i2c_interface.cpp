#include "logger.hpp"
#include "i2c_interface.hpp"
#include "gsl/gsl-lite.hpp"
#include "utilities.hpp"

#include <iostream>

namespace i2c_interface {
  MAKE_LOCAL_LOGGER("i2c_interface");

  void I2CInterface::delay(void) const {
    std::this_thread::sleep_for(delay_ns_);
  }

  void write(const hw_interface::stream_id &stream_id, const uint32_t &address,
             const uint32_t &value, const uint8_t &length, const hw_interface::access_width &access_width,
             std::ostream &out) {
    switch(stream_id) {
    case hw_interface::HSD_STRM_FE_D_21:
    case hw_interface::HSD_STRM_FE_D_22:
    case hw_interface::HSD_STRM_FE_D_23:
    case hw_interface::HSD_STRM_FE_D_25:
    case hw_interface::HSD_STRM_FE_D_30:
    case hw_interface::HSD_STRM_FE_D_43: {
      std::vector<uint8_t> data;
      switch (access_width) {
      case hw_interface::access_width::BYTE: {
        if (length != 1) {
          LOGGER_ERROR("Byte access is usually only done for 1 byte long streams.");
          return;
        }
        const auto sized_value = gsl::narrow<uint8_t>(value);
        LOGGER_INFO("Writing byte: 0x{:02X} @ 0x{:04X}", sized_value, address);
        data = to_byte_vector(sized_value);
        break;
      }

      case hw_interface::access_width::WORD: {
        if (length != 2) {
          LOGGER_ERROR("Word access is usually only done for 2 byte long streams.");
          return;
        }
        const auto sized_value = gsl::narrow<uint16_t>(value);
        LOGGER_INFO("Writing byte: 0x{:04X} @ 0x{:04X}", sized_value, address);
        data = to_byte_vector(sized_value);
        break;
      }

      case hw_interface::access_width::DWORD: {
        if (length != 4) {
          LOGGER_ERROR("Dword access is usually only done for 4 byte long streams.");
          return;
        }
        const auto sized_value = gsl::narrow<uint32_t>(value);
        LOGGER_INFO("Writing byte: 0x{:08X} @ 0x{:04X}", sized_value, address);
        data = to_byte_vector(sized_value);
        break;
      }

      default:
        LOGGER_ERROR("Access mode currently not supported.");
        break;
      }

      i2c_bitbangers[i2c_interfaces::FE].get().write(address, data);
      break;
    }
    default:
      LOGGER_ERROR("Requested stream (0x{:x}) currently not supported for I2C access.", stream_id);
    }
  }

  std::optional<uint32_t> read(const hw_interface::stream_id &stream_id,
                               const uint32_t &address, const uint8_t &length,
                               const hw_interface::access_width &access_width,
                               std::ostream &out) {
    logger::RAIIFlush raii_flush(out);

    switch (stream_id) {
    case hw_interface::HSD_STRM_FE_D_21:
    case hw_interface::HSD_STRM_FE_D_22:
    case hw_interface::HSD_STRM_FE_D_23:
    case hw_interface::HSD_STRM_FE_D_25:
    case hw_interface::HSD_STRM_FE_D_30:
    case hw_interface::HSD_STRM_FE_D_43: {



      switch (access_width) {
      case hw_interface::access_width::BYTE: {
        if (length != 1) {
          LOGGER_ERROR("Byte access is usually only done for 1 byte long streams.");
          return EXIT_FAILURE;
        }
        const auto read_bytes = i2c_bitbangers[i2c_interfaces::FE].get().read(address, 1);
        const auto read_value = from_byte_vector<uint8_t>(read_bytes);
        LOGGER_INFO("Read byte: 0x{:02X} @ 0x{:04X}", read_value, address);
        return read_value;
      }

      case hw_interface::access_width::WORD: {
        if (length != 2) {
          LOGGER_ERROR("Word access is usually only done for 2 byte long streams.");
          return EXIT_FAILURE;
        }
        const auto read_bytes = i2c_bitbangers[i2c_interfaces::FE].get().read(address, 2);
        const auto read_value = from_byte_vector<uint16_t>(read_bytes);
        LOGGER_INFO("Read word: 0x{:04X} @ 0x{:04X}", read_value, address);
        return read_value;
      }

      case hw_interface::access_width::DWORD: {
        if (length != 4) {
          LOGGER_ERROR("Dword access is usually only done for 4 byte long streams.");
          return EXIT_FAILURE;
        }
        const auto read_bytes = i2c_bitbangers[i2c_interfaces::FE].get().read(address, 4);
        const auto read_value = from_byte_vector<uint32_t>(read_bytes);
        LOGGER_INFO("Read word: 0x{:08X} @ 0x{:04X}", read_value, address);
        return read_value;
      }

      default:
        LOGGER_ERROR("Access mode currently not supported.");
        return std::nullopt;
      }
    }

    default:
      LOGGER_ERROR("Requested stream (0x{:x}) currently not supported for I2C access.", stream_id);
      return std::nullopt;
    }
  }

  // if no readback of SCL line is supported, it can't react to clock
  // stretching
  struct DefaultSCLGetter {
    bool operator()(void) { return 1; }
  };

  enum i2c_direction : bool {WRITE = 0,
                             READ = 1};

  enum i2c_ack : bool {ACK = 0,
                       NACK = 1};

  template <class SDA_Setter, class SDA_Getter, class SCL_Setter,
            class SCL_Getter = DefaultSCLGetter, bool ArbitrationCheck = false>
  class I2CBitBang : public I2CInterface {
  public:
    I2CBitBang(const uint_fast32_t &bits_per_second = 100000) :  I2CInterface(bits_per_second) {}

    void write(const uint_fast16_t &address,
               const std::vector<uint8_t> &data) const override {
      if (write_byte(true, false, address | i2c_direction::WRITE) == i2c_ack::ACK) {
        i2c_ack ack = i2c_ack::ACK;
        for (const auto &byte : data) {
          if (ack == i2c_ack::ACK)
            ack = write_byte(false, false, byte);
          else
            break;
        }
        stop_cond();
        if (ack == i2c_ack::ACK)
          return;
      } else
        stop_cond();
      throw std::runtime_error("I2C: Writing failed because of missing ACKs");
    }

    std::vector<uint8_t> read(const uint_fast16_t &address,
                              const uint_fast16_t &length) const override {
      std::vector<uint8_t> result;
      result.reserve(length);

      if (!write_byte(true, false, address | i2c_direction::READ)) {
        for (uint_fast16_t k = 0; k < length; ++k) {
          const bool is_last = k == length - 1;
          result.push_back(read_byte(is_last ? i2c_ack::NACK : i2c_ack::ACK, is_last));
        }
        return result;
      } else
        stop_cond();
      throw std::runtime_error("I2C: Reading failed because of missing ACK.");
    }

    std::vector<uint8_t> read_eeprom(const uint_fast16_t &device_address,
                                     const uint_fast16_t &memory_address,
                                     const uint_fast16_t &length) const override {
      std::vector<uint8_t> result;
      result.reserve(length);
      if (write_byte(true, false, device_address | i2c_direction::WRITE) == i2c_ack::NACK)
        goto NACK;
      if (write_byte(false, false, memory_address) == i2c_ack::NACK)
        goto NACK;
      if (write_byte(true, false, device_address | i2c_direction::READ) == i2c_ack::NACK)
        goto NACK;
      for (size_t k = 0; k < length - 1; ++k)
        result.push_back(read_byte(i2c_ack::ACK, false));
      result.push_back(read_byte(i2c_ack::NACK, true));
      return result;

    NACK:
      throw std::runtime_error("I2C: Reading EEPROM failed because of missing ACK.");
    }

  protected:
    void start_cond(void) const {
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

    void stop_cond(void) const {
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

    void write_bit(bool bit) const {
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

    bool read_bit(void) const {
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

    i2c_ack write_byte(bool send_start, bool send_stop, uint_fast8_t byte) const {
      i2c_ack ack;

      if (send_start)
        start_cond();

      for (uint_fast8_t bit = 0; bit < 8; ++bit) {
        write_bit((byte & 0x80) != 0);
        byte <<= 1;
      }

      ack = read_bit() ? i2c_ack::NACK : i2c_ack::ACK;

      if (send_stop)
        stop_cond();

      return ack;
    }

    uint_fast8_t read_byte(i2c_ack ack, bool send_stop) const {
      uint_fast8_t byte = 0;

      for (uint_fast8_t bit = 0; bit < 8; ++bit) {
        byte = (byte << 1) | read_bit();
      }

      write_bit((ack == i2c_ack::ACK) ? 0 : 1);

      if (send_stop)
        stop_cond();

      return byte;
    }

  private:
    mutable bool started_ = false;
  };


  struct SDA_Setter_FE {
    void operator()(bool bit) {
      hw_interface::switch_status sw_status;
      if(bit)
        sw_status = hw_interface::HSD_STA_HIGH;
      else
        sw_status = hw_interface::HSD_STA_LOW;

      logger::NullStream null_stream;
      hw_interface::write_switch(hw_interface::HSD_SW_DIG_I2C_FE_DATA_WR,
                                 sw_status,
                                 null_stream);
    }
  };

  struct SDA_Getter_FE {
    bool operator()(void) {
      logger::NullStream null_stream;
      const auto sw_status_opt = hw_interface::read_switch_status(hw_interface::HSD_SW_DIG_I2C_FE_DATA_RD,
                                                                  null_stream);
      return sw_status_opt.value() == hw_interface::HSD_STA_HIGH;
    }
  };

  struct SCL_Setter_FE {
    void operator()(bool bit) {
      hw_interface::switch_status sw_status;
      if (bit)
        sw_status = hw_interface::HSD_STA_HIGH;
      else
        sw_status = hw_interface::HSD_STA_LOW;

      logger::NullStream null_stream;
      hw_interface::write_switch(hw_interface::HSD_SW_DIG_I2C_FE_CLK,
                                 sw_status,
                                 null_stream);
    }
  };

  template <class SDA_Setter, class SDA_Getter, class SCL_Setter,
            class SCL_Getter = DefaultSCLGetter, bool ArbitrationCheck = false>
  class I2CBitBang24XXEEPROM : public I2CBitBang<SDA_Setter, SDA_Getter, SCL_Setter, SCL_Getter, ArbitrationCheck> {
  public:
    I2CBitBang24XXEEPROM(const uint_fast32_t &bits_per_second = 100000)
      : I2CBitBang<SDA_Setter, SDA_Getter, SCL_Setter, SCL_Getter, ArbitrationCheck>(bits_per_second) {}

  };

    const I2CBitBang24XXEEPROM<SDA_Setter_FE, SDA_Getter_FE, SCL_Setter_FE>
        i2c_bitbanger_fe;

    const std::array<std::reference_wrapper<const I2CInterface>, 1>
        i2c_bitbangers = {i2c_bitbanger_fe};

} // namespace i2c_interface

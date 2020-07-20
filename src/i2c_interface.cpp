#include "logger.hpp"
#include "i2c_interface.hpp"
#include "gsl/gsl-lite.hpp"

#include <iostream>

namespace i2c_interface {
  MAKE_LOCAL_LOGGER("i2c_interface");

  void I2CInterface::delay(void) {
    std::this_thread::sleep_for(delay_ns_);
  }

  struct SDA_Setter_FE {
    void operator()(bool bit) {
      hw_interface::switch_status sw_status;
      if(bit)
        sw_status = hw_interface::HSD_STA_HIGH;
      else
        sw_status = hw_interface::HSD_STA_LOW;

      hw_interface::write_switch(hw_interface::HSD_SW_DIG_I2C_FE_DATA_WR,
                                 sw_status,
                                 std::cout);
    }
  };

  struct SDA_Getter_FE {
    bool operator()(void) {
      const auto sw_status_opt = hw_interface::read_switch_status(hw_interface::HSD_SW_DIG_I2C_FE_DATA_RD,
                                                                  std::cout);
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

      hw_interface::write_switch(hw_interface::HSD_SW_DIG_I2C_FE_CLK,
                                 sw_status,
                                 std::cout);
    }
  };

  static i2c_interface::I2CBitBang<SDA_Setter_FE, SDA_Getter_FE, SCL_Setter_FE> i2c_bitbanger_fe;

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

      i2c_bitbanger_fe.write(address, data);
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
        const auto read_bytes = i2c_bitbanger_fe.read(address, 1);
        const auto read_value = from_byte_vector<uint8_t>(read_bytes);
        LOGGER_INFO("Read byte: 0x{:02X} @ 0x{:04X}", read_value, address);
        return read_value;
      }

      case hw_interface::access_width::WORD: {
        if (length != 2) {
          LOGGER_ERROR("Word access is usually only done for 2 byte long streams.");
          return EXIT_FAILURE;
        }
        const auto read_bytes = i2c_bitbanger_fe.read(address, 2);
        const auto read_value = from_byte_vector<uint16_t>(read_bytes);
        LOGGER_INFO("Read word: 0x{:04X} @ 0x{:04X}", read_value, address);
        return read_value;
      }

      case hw_interface::access_width::DWORD: {
        if (length != 4) {
          LOGGER_ERROR("Dword access is usually only done for 4 byte long streams.");
          return EXIT_FAILURE;
        }
        const auto read_bytes = i2c_bitbanger_fe.read(address, 4);
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

} // namespace i2c_interface

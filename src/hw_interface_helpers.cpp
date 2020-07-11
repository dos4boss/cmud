#include "logger.hpp"
#include "hw_interface_helpers.hpp"
#include "hw_interface.hpp"
#include "gsl/gsl-lite.hpp"

#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <climits>
#include <string>
#include <cstring>
#include <unordered_map>

#include <sys/io.h>

namespace hw_interface {
  MAKE_LOCAL_LOGGER("hw_interface");

  template <typename R> static constexpr R bitmask(unsigned int const onecount) {
    return static_cast<R>(-(onecount != 0)) &
      (static_cast<R>(-1) >> ((sizeof(R) * CHAR_BIT) - onecount));
  }

  bool is_valid_switch(const std::string &switch_name) {
    auto result = std::find_if(switch_infos.begin(), switch_infos.end(),
                               [switch_name](const switch_info &sw_info) {
                                 return sw_info.name == switch_name;
                               });
    return result != switch_infos.end();
  }

  uint32_t get_bitstream_value(const uint32_t &data, const uint8_t bit_length, const uint8_t bit_offset) {
    auto bit_mask = bitmask<uint32_t>(bit_length) << bit_offset;
    return (data & bit_mask) >> bit_offset;
  }

  void set_bitstream_value(uint32_t &data, const uint8_t bit_length,
                           const uint8_t bit_offset, const uint32_t value) {
    auto bit_mask = bitmask<uint32_t>(bit_length);
    if(value > bit_mask)
      throw std::runtime_error("Value is too big to be set...");

    auto shifted_bitmask = bit_mask << bit_offset;
    data = (data & ~shifted_bitmask) | ((value & bit_mask) << bit_offset);
  }


  void init(void) {
    std::unordered_map<decltype(stream_info::address), decltype(stream_info::length)> ioport_map;
    for (const auto &stream_info : stream_infos) {
      *(stream_info.data) = stream_info.default_data;
      if(stream_info.interface_mode == interface_mode::IO_PORTS) {
        const auto search = ioport_map.find(stream_info.address);
        if (search != ioport_map.end() && search->second == stream_info.length)
          continue;
        const auto return_value = ioperm(stream_info.address, stream_info.length, 1);
        ioport_map[stream_info.address] = stream_info.length;
        if (return_value != 0) {
          if(errno == EINVAL) {
            LOGGER_ERROR("Failed to get access to I/O Ports 0x{:04X} - 0x{:04X} ({})",
                         stream_info.address,
                         stream_info.address + stream_info.length,
                         std::strerror(errno));
            std::cout << logger::LoggerSink::get();
          }
          else if(errno == ENOMEM) {
            LOGGER_CRITICAL("Out of memory during I/O Ports 0x{:04X} - "
                            "0x{:04X} access request",
                            stream_info.address,
                            stream_info.address + stream_info.length);
            std::cout << logger::LoggerSink::get();
            exit(EXIT_FAILURE);
          } else if (errno == EPERM) {
            LOGGER_CRITICAL("Permission denied on access request. Make sure to run as root.");
            std::cout << logger::LoggerSink::get();
            exit(EXIT_FAILURE);
          }
          else {
            LOGGER_CRITICAL("Unknown failure during I/O Port access request ({})",
                            std::strerror(errno));
            std::cout << logger::LoggerSink::get();
            exit(EXIT_FAILURE);
          }
        }
      }
    }
  }

  uint32_t read_io(const uint32_t address, uint32_t &value, const uint8_t length,
                   enum access_width access_width, std::ostream &out) {
    switch (access_width) {
    case access_width::BYTE: {
      if (length != 1) {
        LOGGER_ERROR("Byte access is usually only done for 1 byte long streams.");
        return EXIT_FAILURE;
      }
      const auto read_value = inb(address);
      LOGGER_INFO("Read byte: 0x{:02X} @ 0x{:04X}", read_value, address);
      value = read_value;
      break;
    }

    case access_width::WORD: {
      if (length != 2) {
        LOGGER_ERROR("Word access is usually only done for 2 byte long streams.");
        return EXIT_FAILURE;
      }
      const auto read_value = inw(address);
      LOGGER_INFO("Read word: 0x{:04X} @ 0x{:04X}", read_value, address);
      value = read_value;
      break;
    }

    case access_width::DWORD: {
      if (length != 4) {
        LOGGER_ERROR("Dword access is usually only done for 4 byte long streams.");
        return EXIT_FAILURE;
      }
      const auto read_value = inl(address);
      LOGGER_INFO("Read word: 0x{:08X} @ 0x{:04X}", read_value, address);
      value = read_value;
      break;
    }

    default:
      LOGGER_ERROR("Access mode currently not supported.");
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }

  uint32_t flush_switch_in(enum switch_id switch_id, std::ostream &out) {
    if (switch_id >= switch_infos.size()) {
      LOGGER_ERROR("Invalid switch id ({:} >= {:})", switch_id, switch_infos.size());
      return EXIT_FAILURE;
    }
    const auto &stream_info = stream_infos[switch_infos[switch_id].stream_id];
    switch (stream_info.interface_mode) {
    case interface_mode::IO_PORTS:
      return read_io(stream_info.address, *(stream_info.data), stream_info.length,
                     stream_info.access_width, out);
      break;
    default:
      LOGGER_ERROR("Interface mode not yet supported.");
      return EXIT_FAILURE;
    }
  }


  std::optional<uint32_t> read_switch(const switch_id &switch_id, std::ostream &out) {
    logger::RAIIFlush raii_flusher(out);

    const auto &switch_info = switch_infos[switch_id];
    if (switch_info.switch_id != switch_id) {
      LOGGER_ERROR("Switch id is mismatching idx (idx: {:}, switch_id: {:})",
                   switch_id, switch_info.switch_id);
      return std::nullopt;
    }

    const auto flush_result = flush_switch_in(switch_info.switch_id, out);
    if(flush_result != EXIT_SUCCESS) {
      LOGGER_ERROR("Failed to read switch from hardware.");
      return std::nullopt;
    }

    auto bitstream_value =
      get_bitstream_value(*stream_infos[switch_info.stream_id].data,
                          switch_info.bit_length,
                          switch_info.bit_position);

    if(switch_info.type == switch_type::CONTINUOUS) {
      out << "Continuous Value: " << bitstream_value << std::endl;
      return bitstream_value;
    }
    else {
      auto switch_trans = std::find_if(switch_info.translation->begin(),
                                       switch_info.translation->end(),
                                       [&bitstream_value](const switch_translation& trans) {
                                         return trans.bitstream_value == bitstream_value;
                                       });
      if(switch_trans == switch_info.translation->end()) {
        LOGGER_ERROR("Invalid switch value in bitstream");
        return std::nullopt;
      }
      else {
        out << "Discrete Value: " << switch_trans->name << std::endl;
        return switch_trans->id;
      }
    }
  }



  std::optional<uint32_t> read_switch(const std::string &switch_name, std::ostream &out) {
    logger::RAIIFlush raii_flush(out);
    auto result = std::find_if(switch_infos.begin(), switch_infos.end(),
                               [switch_name](const switch_info &sw_info) {
                                 return sw_info.name == switch_name;
                               });
    if (result == switch_infos.end()) {
      LOGGER_ERROR("Switch '{}' is invalid", switch_name);
      return std::nullopt;
    }

    return read_switch(result->switch_id, out);
  }

  void write_io(const uint32_t address, const uint32_t value, const uint8_t length, enum access_width access_width, std::ostream &out) {
    switch(access_width) {
    case access_width::BYTE: {
      if(length != 1) {
        LOGGER_ERROR("Byte access is usually only done for 1 byte long streams.");
        return;
      }
      const auto sized_value = gsl::narrow<uint8_t>(value);
      LOGGER_INFO("Writing byte: 0x{:02X} @ 0x{:04X}", sized_value, address);
      outb(sized_value,  address);
      break;
    }

    case access_width::WORD: {
      if (length != 2) {
        LOGGER_ERROR("Word access is usually only done for 2 byte long streams.");
        return;
      }
      const auto sized_value = gsl::narrow<uint16_t>(value);
      LOGGER_INFO("Writing byte: 0x{:04X} @ 0x{:04X}", sized_value, address);
      outw(sized_value, address);
      break;
    }

    case access_width::DWORD: {
      if (length != 4) {
        LOGGER_ERROR("Dword access is usually only done for 4 byte long streams.");
        return;
      }
      const auto sized_value = gsl::narrow<uint32_t>(value);
      LOGGER_INFO("Writing byte: 0x{:08X} @ 0x{:04X}", sized_value, address);
      outl(sized_value, address);
      break;
    }

    default:
      LOGGER_ERROR("Access mode currently not supported.");
    }
  }

  void flush_switch_out(enum switch_id switch_id, std::ostream &out) {
    if(switch_id >= switch_infos.size()) {
      LOGGER_ERROR("Invalid switch id ({:} >= {:})", switch_id, switch_infos.size());
      return;
    }
    const auto &stream_info = stream_infos[switch_infos[switch_id].stream_id];
    switch(stream_info.interface_mode) {
    case interface_mode::IO_PORTS:
      write_io(stream_info.address, *(stream_info.data), stream_info.length, stream_info.access_width, out);
      break;
    default:
      LOGGER_ERROR("Interface mode not yet supported.");
    }
  }

  void write_switch(const std::string &switch_name, const std::string &switch_state, std::ostream &out) {
    logger::RAIIFlush raii_flush(out);

    auto result = std::find_if(switch_infos.begin(), switch_infos.end(),
                               [switch_name](const switch_info &sw_info) {
                                 return sw_info.name == switch_name;
                               });
    if (result == switch_infos.end()) {
      LOGGER_ERROR("Switch is invalid");
      return;
    }

    if(result->type == switch_type::CONTINUOUS) {
      unsigned long cont_value;
      try {
        cont_value = std::stoul(switch_state, 0, 0);
      }
      catch(const std::invalid_argument &e){
        LOGGER_ERROR("Could not create a valid value for input '{}' ({}).", switch_state, e.what());
        return;
      } catch (const std::out_of_range &e) {
        LOGGER_ERROR("Input value '{}' does not fit in data size ({}).", switch_state, e.what());
        return;
      }

      if((result->continuous_min > cont_value) || (result->continuous_max < cont_value)) {
        LOGGER_ERROR("Input value has to be between {0} (0x{0:X}) and {1} (0x{1:X})",
                     result->continuous_min, result->continuous_min);
        return;
      }
      set_bitstream_value(*stream_infos[result->stream_id].data,
                          result->bit_length, result->bit_position,
                          cont_value);
    }
    else if (result->type == switch_type::DISCRETE) {
      auto switch_trans =
          std::find_if(result->translation->begin(), result->translation->end(),
                       [&switch_state](const switch_translation &trans) {
                         return trans.name == switch_state;
                       });
      if (switch_trans == result->translation->end()) {
        LOGGER_ERROR("Invalid switch state for selected switch.\n"
                     "Valid values would be:");
        for(const auto & switch_translation : *result->translation)
          LOGGER_ERROR("\t{}", switch_translation.name);
      }
      else
        set_bitstream_value(*stream_infos[result->stream_id].data,
                            result->bit_length, result->bit_position,
                            switch_trans->bitstream_value);
    }
    flush_switch_out(result->switch_id, out);
  }

}; // namespace hw_interface

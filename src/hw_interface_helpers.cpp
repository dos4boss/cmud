#include "hw_interface_helpers.hpp"
#include "hw_interface.hpp"
#include "output_helpers.hpp"
#include "../ansi-color-kit/ansicolorkit.h"
#include "gsl/gsl-lite.hpp"

#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <climits>
#include <string>
#include <cstring>

#include <sys/io.h>

namespace hw_interface {

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
    for(const auto & stream_info : stream_infos) {
      *(stream_info.data) = stream_info.default_data;
      if(stream_info.interface_mode == interface_mode::IO_PORTS) {
        auto return_value = ioperm(stream_info.address, stream_info.length, 1);
        if(return_value != 0) {
          std::cout << "ret_val " << return_value << std::strerror(errno) << std::endl;
          if(errno == EINVAL)
            std::cout << ansi::foreground::RED << "[I/O Ports] Address "
                      << stream_info.address << " failed" << std::endl;
          else if(errno == ENOMEM) {
            std::cout << ansi::foreground::RED
                      << "[I/O Ports] Out of Memory during access request for address "
                      << stream_info.address << std::endl;
            exit(EXIT_FAILURE);
          } else if (errno == EPERM) {
            std::cout << ansi::foreground::RED
                      << "[I/O Ports] Permission denied on access request. Make "
              "sure to run as root. "
                      << std::endl;
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
        out << ansi::foreground::RED
            << "Byte access is usually only done for 1 byte long streams."
            << std::endl;
        return EXIT_FAILURE;
      }
      const auto read_value = inb(address);
      out << ansi::foreground::CYAN << "[I/O Ports] Read byte: 0x"
          << std::hex << +read_value << " @ 0x" << address << std::endl;
      value = read_value;
      break;
    }

    case access_width::WORD: {
      if (length != 2) {
        out << ansi::foreground::RED
            << "Word access is usually only done for 2 byte long streams."
            << std::endl;
        return EXIT_FAILURE;
      }
      const auto read_value = inw(address);
      out << ansi::foreground::CYAN << "[I/O Ports] Read byte: 0x" << std::hex
          << read_value << " @ 0x" << address << std::endl;
      value = read_value;
      break;
    }

    case access_width::DWORD: {
      if (length != 2) {
        out << ansi::foreground::RED
            << "Word access is usually only done for 2 byte long streams."
            << std::endl;
        return EXIT_FAILURE;
      }
      const auto read_value = inl(address);
      out << ansi::foreground::CYAN << "[I/O Ports] Read byte: 0x" << std::hex
          << read_value << " @ 0x" << address << std::endl;
      value = read_value;
      break;
    }

    default:
      out << ansi::foreground::RED
          << "[I/O Ports] Access mode currently not supported." << std::endl;
    }
    return EXIT_SUCCESS;
  }

  uint32_t flush_switch_in(enum switch_id switch_id, std::ostream &out) {
    if (switch_id >= switch_infos.size()) {
      out << ansi::foreground::RED << "Invalid switch id" << std::endl;
      return EXIT_FAILURE;
    }
    const auto &stream_info = stream_infos[switch_infos[switch_id].stream_id];
    switch (stream_info.interface_mode) {
    case interface_mode::IO_PORTS:
      return read_io(stream_info.address, *(stream_info.data), stream_info.length,
                     stream_info.access_width, out);
      break;
    default:
      out << ansi::foreground::RED << "Interface mode not supported"
          << std::endl;
      return EXIT_FAILURE;
    }
  }


  std::optional<uint32_t> read_switch(const switch_id &switch_id, std::ostream &out) {
    RAIIStreamReconstructor stream_recon(out);
    const auto &switch_info = switch_infos[switch_id];
    if (switch_info.switch_id != switch_id) {
      out << ansi::foreground::RED << "Invalid switch_id."
          << std::endl;
      return std::nullopt;
    }

    const auto flush_result = flush_switch_in(switch_info.switch_id, out);
    if(flush_result != EXIT_SUCCESS) {
      out << ansi::foreground::RED << "Failed to read switch from hardware." << std::endl;
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
        out << ansi::foreground::RED << "Invalid switch value in bitstream"
            << std::endl;
        return std::nullopt;
      }
      else {
        out << "Discrete Value: " << switch_trans->name << std::endl;
        return switch_trans->id;
      }
    }
  }


  std::optional<uint32_t> read_switch(const std::string &switch_name, std::ostream &out) {
    RAIIStreamReconstructor stream_recon(out);

    auto result = std::find_if(switch_infos.begin(), switch_infos.end(),
                               [switch_name](const switch_info &sw_info) {
                                 return sw_info.name == switch_name;
                               });
    if (result == switch_infos.end()) {
      out << ansi::foreground::RED << "Switch is invalid" << std::endl;
      return std::nullopt;
    }

    return read_switch(result->switch_id, out);
  }

  void write_io(const uint32_t address, const uint32_t value, const uint8_t length, enum access_width access_width, std::ostream &out) {
    switch(access_width) {
    case access_width::BYTE: {
      if(length != 1) {
        out << ansi::foreground::RED << "Byte access is usually only done for 1 byte long streams." << std::endl;
        return;
      }
      const auto sized_value = gsl::narrow<uint8_t>(value);
      out << ansi::foreground::CYAN << "[I/O Ports] Writing byte: 0x" << std::hex
          << +sized_value << " @ 0x" << address << std::endl;
      outb(sized_value,  address);
      break;
    }

    case access_width::WORD: {
      if (length != 2) {
        out << ansi::foreground::RED
            << "Word access is usually only done for 2 byte long streams."
            << std::endl;
        return;
      }
      const auto sized_value = gsl::narrow<uint16_t>(value);
      out << ansi::foreground::CYAN << "[I/O Ports] Writing byte: 0x"
          << std::hex << sized_value << " @ 0x" << address << std::endl;
      outw(sized_value, address);
      break;
    }

    case access_width::DWORD: {
      if (length != 4) {
        out << ansi::foreground::RED
            << "Dword access is usually only done for 4 byte long streams."
            << std::endl;
        return;
      }
      const auto sized_value = gsl::narrow<uint32_t>(value);
      out << ansi::foreground::CYAN << "[I/O Ports] Writing byte: 0x"
          << std::hex << sized_value << " @ 0x" << address << std::endl;
      outl(sized_value, address);
      break;
    }

    default:
      out << ansi::foreground::RED << "[I/O Ports] Access mode currently not supported." << std::endl;
    }
  }

  void flush_switch_out(enum switch_id switch_id, std::ostream &out) {
    if(switch_id >= switch_infos.size()) {
      out << ansi::foreground::RED << "Invalid switch id" << std::endl;
      return;
    }
    const auto &stream_info = stream_infos[switch_infos[switch_id].stream_id];
    switch(stream_info.interface_mode) {
    case interface_mode::IO_PORTS:
      write_io(stream_info.address, *(stream_info.data), stream_info.length, stream_info.access_width, out);
      break;
    default:
      out << ansi::foreground::RED << "Interface mode not supported" << std::endl;
    }
  }

  void write_switch(const std::string &switch_name, const std::string &switch_state, std::ostream &out) {
    RAIIStreamReconstructor stream_recon(out);

    auto result = std::find_if(switch_infos.begin(), switch_infos.end(),
                               [switch_name](const switch_info &sw_info) {
                                 return sw_info.name == switch_name;
                               });
    if (result == switch_infos.end()) {
      out << "Switch is invalid" << std::endl;
      return;
    }

    if(result->type == switch_type::CONTINUOUS) {
      unsigned long cont_value;
      try {
        cont_value = std::stoul(switch_state, 0, 0);
      }
      catch(const std::invalid_argument &e){
        out << ansi::foreground::RED << "Could not create a valid value for input '"
            << switch_state << "' (" << e.what() << ")." << std::endl;
        return;
      } catch (const std::out_of_range &e) {
        out << ansi::foreground::RED << "Input value '" << switch_state
            << "' does not fit in data size (" << e.what() << ")." << std::endl;
        return;
      }

      if((result->continuous_min > cont_value) || (result->continuous_max < cont_value)) {
        out << ansi::foreground::RED << "Input value has to be between "
            << result->continuous_min << " (0x" << std::hex
            << result->continuous_min << ") and " << std::dec
            << result->continuous_max << " (0x" << std::hex
            << result->continuous_max << ")" << ansi::RESET << std::endl;
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
        out << ansi::foreground::RED
            << "Invalid switch state for selected switch."
            << std::endl;
        out << "Valid values would be:" << std::endl;
        for(const auto & switch_translation : *result->translation)
          out << "\t" << switch_translation.name << std::endl;
      }
      else
        set_bitstream_value(*stream_infos[result->stream_id].data,
                            result->bit_length, result->bit_position,
                            switch_trans->bitstream_value);
    }
    flush_switch_out(result->switch_id, out);
  }

}; // namespace hw_interface

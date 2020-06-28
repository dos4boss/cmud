#include "hw_interface_helpers.hpp"
#include "hw_interface.hpp"
#include "output_helpers.hpp"
#include "../ansi-color-kit/ansicolorkit.h"

#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <climits>
#include <string>

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
    for(const auto & stream_info : stream_infos)
      *(stream_info.data) = stream_info.default_data;
  }

  void read_switch(const std::string &switch_name, std::ostream &out) {
    RAIIStreamReconstructor stream_recon(out);

    auto result = std::find_if(switch_infos.begin(), switch_infos.end(),
                               [switch_name](const switch_info &sw_info) {
                                 return sw_info.name == switch_name;
                               });
    if (result == switch_infos.end()) {
      out << ansi::foreground::RED << "Switch is invalid" << std::endl;
      return;
    }

    if ((stream_infos[result->stream_id].interface_mode !=
         interface_mode::IO_PORTS) &&
        (stream_infos[result->stream_id].interface_mode !=
         interface_mode::MMIO)) {
      out << ansi::foreground::RED
          << "Currently only I/O ports and MMIO is supported" << std::endl;
      return;
    }

    auto bitstream_value =
      get_bitstream_value(*stream_infos[result->stream_id].data,
                          result->bit_length,
                          result->bit_position);

    if(result->type == switch_type::CONTINUOUS) {
      out << "Continuous Value: " << bitstream_value << std::endl;
    }
    else {
      auto switch_trans = std::find_if(result->translation->begin(),
                                       result->translation->end(),
                                       [&bitstream_value](const switch_translation& trans) {
                                         return trans.bitstream_value == bitstream_value;
                                       });
      if(switch_trans == result->translation->end())
        out << ansi::foreground::RED << "Invalid switch value in bitstream"
            << std::endl;
      else
        out << "Discrete Value: " << switch_trans->name << std::endl;
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

      out << "Only discrete switches can be set with string value " << cont_value << std::endl;
      return;
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

  }

}; // namespace hw_interface

#include "hw_interface_helpers.hpp"
#include "hw_interface.hpp"

#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <climits>

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

  void read_switch(const std::string &switch_name, std::ostream &out) {
    auto result = std::find_if(switch_infos.begin(), switch_infos.end(),
                               [switch_name](const switch_info &sw_info) {
                                 return sw_info.name == switch_name;
                               });
    if (result == switch_infos.end()) {
      out << "Switch is invalid" << std::endl;
      return;
    }

    if ((stream_infos[result->stream_id].interface_mode !=
         interface_mode::IO_PORTS) &&
        (stream_infos[result->stream_id].interface_mode !=
         interface_mode::MMIO)) {
      out << "Currently only I/O ports and MMIO is supported" << std::endl;
      return;
    }

    auto bitstream_value =
      get_bitstream_value(stream_infos[result->stream_id].data,
                          result->bit_length,
                          result->bit_position);

    if(result->type == switch_type::CONTINUOUS) {
      out << "Continuous Value: " << bitstream_value << std::endl;
    }
    else {
      auto switch_trans = std::find_if(result->translation->begin(),
                                       result->translation->end(),
                                       [bitstream_value](const switch_translation& trans) {
                                         return trans.bitstream_value == bitstream_value;
                                       });
      if(switch_trans == result->translation->end())
        out << "Invalid switch value in bitstream" << std::endl;
      else
        out << "Discrete Value: " << switch_trans->name << std::endl;
    }
  }

}; // namespace hw_interface

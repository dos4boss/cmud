#pragma once

#include <optional>
#include <iostream>
#include "hw_interface.hpp"


namespace io_port_interface {

  void register_access(const decltype(hw_interface::stream_info::address) &address,
                       const decltype(hw_interface::stream_info::length) &length);

  void write(const uint32_t &address, const uint32_t &value,
             const uint8_t &length, const hw_interface::access_width &access_width,
             std::ostream &out);

  std::optional<uint32_t> read(const uint32_t &address, const uint8_t &length,
                               const hw_interface::access_width &access_width,
                               std::ostream &out);

} // namespace io_port_interface

#pragma once

#include <iostream>

template <typename T,
          typename iterator,
          bool big_endian = true,
          typename std::enable_if<std::is_arithmetic<T>::value>::type * = nullptr>
T from_byte_iterator(iterator &byte_iterator) {
  T result{0};
  for (size_t i = 0; i < sizeof(T); i++) {
    if (byte_iterator == iterator())
      throw std::runtime_error("Iterator at end..");
    if constexpr (big_endian) {
      result <<= 8;
      result |= uint8_t(*byte_iterator++);
    } else {
      result |= T(*byte_iterator++) << (i * 8);
    }
  }
  return result;
}

namespace fpga_interface {

  enum fpga {AT_DSP = 0,
             LINK,
             MOD_DEMOD,
             INVALID};

  enum fpga_mode {PROGRAMM = 0,
                  RUN};

  void set_mode(const fpga &fpga, const fpga_mode &fpga_mode, std::ostream &out);
  bool get_init(const fpga &fpga, std::ostream &out);
  bool get_done(const fpga &fpga, std::ostream &out);
  bool get_busy(const fpga &fpga, std::ostream &out);

  bool load_fpga(const fpga &fpga, const std::string &file_path, std::ostream &out);
  bool load_fpga(const std::string &fpga_str, const std::string &file_path, std::ostream &out);

} // namespace fpga_interface

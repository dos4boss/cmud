#pragma once

#include <iostream>

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

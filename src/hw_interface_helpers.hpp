#pragma once

#include <string>

namespace hw_interface {

  void init(void);
  bool is_valid_switch(const std::string &switch_name);
  void read_switch(const std::string &switch_name, std::ostream &out);
  void write_switch(const std::string &switch_name,
                    const std::string &switch_state, std::ostream &out);

}; // namespace hw_interface

#pragma once

#include <string>

namespace hw_interface {

  bool is_valid_switch(const std::string &switch_name);
  void read_switch(const std::string &switch_name, std::ostream &out);

}; // namespace hw_interface

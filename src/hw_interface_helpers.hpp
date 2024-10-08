#pragma once
#include "hw_interface.hpp"

#include <string>
#include <optional>

namespace hw_interface {

  void init(void);
  bool is_valid_switch(const std::string &switch_name);
  std::optional<uint32_t> read_switch(const switch_id &switch_id, std::ostream &out);
  std::optional<uint32_t> read_switch(const std::string &switch_name, std::ostream &out);
  std::optional<switch_status> read_switch_status(const switch_id &switch_id, std::ostream &out);

  void write_switch(const std::string &switch_name, const std::string &switch_state, std::ostream &out);
  void write_switch(const switch_id &switch_id, const switch_status &switch_state, std::ostream &out);
  void write_switch(const switch_id &switch_id, const uint32_t &value, std::ostream &out);

}; // namespace hw_interface

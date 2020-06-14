#include "hw_interface_helpers.hpp"
#include "hw_interface.hpp"

#include <algorithm>

namespace hw_interface {

bool is_valid_switch(std::string switch_name) {
  auto result = std::find_if(switch_infos.begin(), switch_infos.end(),
                             [switch_name](const switch_info &sw_info) {
                               return sw_info.name == switch_name;
                             });
  return result != switch_infos.end();
}

}; // namespace hw_interface

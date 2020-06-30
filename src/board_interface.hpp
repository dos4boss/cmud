#pragma once
#include <iostream>

namespace board_interface {

  bool check_if_module_present(const std::string& board_id, std::ostream &out, const unsigned &board_idx=0);

};

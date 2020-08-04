#pragma once
#include <iostream>
#include <vector>
#include <array>

#include "hw_interface_helpers.hpp"
#include "mmio_interface.hpp"

namespace board_interface {

  bool check_if_board_present(const std::string& board_id, std::ostream &out, const unsigned &board_idx=0);
  void check_for_present_boards(std::ostream &out);

  enum board_idx {
    REF = 0,
    FE,
    DIG,
    RXTX_0,
    RXTX_1,
    COR_0,
    COR_1,
    AUDIO,
    DSP_56303,
    AUC_0,
    AUC_1,
    ADC_0,
    ADC_1,
    DDC_0,
    DDC_1,
    DDC400_0,
    DDC400_1,
    WDDC400_0,
    WDDC400_1,
    COR_BIN,
    DIG_OLD,
    COPRO_0,
    COPRO_1,
    WCDMATX_0,
    WCDMATX_1,
    WCDMARX_0,
    WCDMARX_1,
    FECTX_0,
    FECTX_1,
    FECRX_0,
    FECRX_1,
    IQIF,
    AUXTX,
    RXIF3_0,
    RXIF3_1,
    USU_0,
    USU_1,
    PowerQuiccII_0,
    PowerQuiccII_1,
    PowerQuiccIII_0,
    PowerQuiccIII_1,
    Bluetooth_0,
    Bluetooth_1,
    MB
  };

  class Board {
  public:
    Board(const board_idx &idx, char const *name, const uint_fast8_t &number)
      : idx_(idx), name_(name), number_(number) {}
    virtual bool is_present(std::ostream &out) const = 0;
    virtual std::vector<uint8_t> read_eeprom(std::ostream &out) = 0;

    board_idx idx_;
    uint_fast8_t number_;
    char const *name_;
  };

  extern const std::array<std::reference_wrapper<const Board>, 14> boards;
};

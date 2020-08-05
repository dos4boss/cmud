#include "logger.hpp"
#include "board_interface.hpp"
#include "hw_interface_helpers.hpp"
#include "mmio_interface.hpp"

namespace board_interface {
  MAKE_LOCAL_LOGGER("board_interface");

  bool check_if_board_present(const std::string &board_id, std::ostream &out, const unsigned &board_idx) {
    logger::RAIIFlush raii_flush(out);

    const auto iter = std::find_if(boards.begin(), boards.end(),
                                   [&](const std::reference_wrapper<const Board> &board) {
                                     return (board.get().name_ == board_id) && (board.get().number_ == board_idx);
                                   });

    if(iter == boards.end()) {
      LOGGER_ERROR("Board with name '{}' and board number {} does not exist.", board_id, board_idx);
      return false;
    }

    return iter->get().is_present(out);
  }

  void check_for_present_boards(std::ostream &out) {
    logger::RAIIFlush raii_flush(out);

    for (const std::reference_wrapper<const Board> &board : boards) {
      const auto is_present = board.get().is_present(out);
      out << "Board " << board.get().name_ << " (" << +board.get().number_ << ") is "
          << (is_present ? "present" : "not present") << std::endl;
    }
  }

  using presence_switch_check =
    std::pair<hw_interface::switch_id, hw_interface::switch_status>;

  template<size_t switch_number>
  class BoardSwitchPresenceCheck : public virtual Board {
  public:
    BoardSwitchPresenceCheck(const board_idx &idx, char const *name, const uint_fast8_t &number,
                             const std::array<presence_switch_check, switch_number> &switch_checks)
      : Board(idx, name, number),
        presence_switch_checks_(switch_checks) {}

    bool is_present(std::ostream &out) const override {
      bool result = true;
      for(const auto &presence_switch : presence_switch_checks_) {
        if (auto switch_val = hw_interface::read_switch_status(presence_switch.first, out))
          result &= switch_val.value() == presence_switch.second;
      }
      return result;
    }

  protected:
    const std::array<presence_switch_check, switch_number> presence_switch_checks_;
  };

  class BoardPresenceAndReadViaCor : public virtual Board {
  public:
    BoardPresenceAndReadViaCor(const board_idx &idx, char const *name, const uint_fast8_t &number,
                               const mmio_interface::MMIOInterface &mmio_interface)
        : Board(idx, name, number), mmio_interface_(mmio_interface) {}

    bool is_present(std::ostream &out) const override {
      std::vector<uint8_t> check_data = {0x55, 0xaa};
      auto restore_data = mmio_interface_.read(0x7fa, 2, out);
      mmio_interface_.write(0x7fa, check_data, out);
      auto readback_data = mmio_interface_.read(0x7fa, 2, out);
      mmio_interface_.write(0x7fa, restore_data, out);
      return check_data == readback_data;
    }

    std::vector<uint8_t> read_eeprom(const uint_fast16_t &memory_address,
                                     const uint_fast16_t &length,
                                     std::ostream &out) const override { return {}; /* TODO */ }
  protected:
    const mmio_interface::MMIOInterface &mmio_interface_;
  };

  class BoardEEPROM24XX : public virtual Board {
  public:
    BoardEEPROM24XX(const board_idx &idx, char const *name,
                    const uint_fast8_t &number, const uint_fast8_t &i2c_address,
                    const i2c_interface::i2c_interfaces &i2c_interface)
      : Board(idx, name, number), i2c_address_(i2c_address), i2c_interface_(i2c_interface) {}

    std::vector<uint8_t> read_eeprom(const uint_fast16_t &memory_address,
                                     const uint_fast16_t &length,
                                     std::ostream &out) const override {
      return i2c_interface::i2c_bitbangers[i2c_interface_].get().read_eeprom(i2c_address_, memory_address, length);
    }

  protected:
    const uint_fast8_t i2c_address_;
    const i2c_interface::i2c_interfaces i2c_interface_;
  };

  template <size_t switch_number>
  class BoardSwitchPresenceCheckEEPROM24XX : public BoardSwitchPresenceCheck<switch_number>, public BoardEEPROM24XX {
  public:
    BoardSwitchPresenceCheckEEPROM24XX(const board_idx &idx, char const *name, const uint_fast8_t &number,
                                       const std::array<presence_switch_check, switch_number> &switch_checks,
                                       const uint_fast8_t &i2c_address, const i2c_interface::i2c_interfaces &i2c_interface)
        : Board(idx, name, number), BoardSwitchPresenceCheck<switch_number>(
                                        idx, name, number, switch_checks),
          BoardEEPROM24XX(idx, name, number, i2c_address, i2c_interface) {}
  };

  const BoardSwitchPresenceCheckEEPROM24XX<1> adc_0{ADC_0,
                                                    "ADC",
                                                    0,
                                                    {std::make_pair(hw_interface::HSD_SW_DIG_ADC_1_OPT_POLL_0,
                                                                    hw_interface::HSD_STA_PRESENT)},
                                                    0xAC,
                                                    i2c_interface::i2c_interfaces::FE};

  const BoardSwitchPresenceCheckEEPROM24XX<1> adc_1{ADC_1,
                                                    "ADC",
                                                    1,
                                                    {std::make_pair(hw_interface::HSD_SW_DIG_ADC_2_OPT_POLL_0,
                                                                    hw_interface::HSD_STA_PRESENT)},
                                                    0xAE,
                                                    i2c_interface::i2c_interfaces::FE};

  const BoardSwitchPresenceCheckEEPROM24XX<1> auc_0{AUC_0,
                                                    "AUC",
                                                    0,
                                                    {std::make_pair(hw_interface::HSD_SW_DIG_AUC_1_OPT_POLL_0,
                                                                    hw_interface::HSD_STA_PRESENT)},
                                                    0x80,
                                                    i2c_interface::i2c_interfaces::FE};

  const BoardSwitchPresenceCheckEEPROM24XX<1> auc_1{AUC_1,
                                                    "AUC",
                                                    1,
                                                    {std::make_pair(hw_interface::HSD_SW_DIG_AUC_2_OPT_POLL_0,
                                                                    hw_interface::HSD_STA_PRESENT)},
                                                    0xD0,
                                                    i2c_interface::i2c_interfaces::FE};

  const BoardSwitchPresenceCheckEEPROM24XX<3> ddc_0{DDC_0,
                                                    "DDC",
                                                    0,
                                                    {
                                                     std::make_pair(hw_interface::HSD_SW_DIG_DDC_1_OPT_POLL_0,
                                                                    hw_interface::HSD_STA_PRESENT),
                                                     std::make_pair(hw_interface::HSD_SW_DIG_DDC_1_OPT_POLL_1,
                                                                    hw_interface::HSD_STA_NOT_PRESENT),
                                                     std::make_pair(hw_interface::HSD_SW_DIG_DDC_1_OPT_POLL_2,
                                                                    hw_interface::HSD_STA_NOT_PRESENT),
                                                    },
                                                    0xA4,
                                                    i2c_interface::i2c_interfaces::FE};

  const BoardSwitchPresenceCheckEEPROM24XX<3> ddc_1{DDC_1,
                                                    "DDC",
                                                    1,
                                                    {
                                                     std::make_pair(hw_interface::HSD_SW_DIG_DDC_2_OPT_POLL_0,
                                                                    hw_interface::HSD_STA_PRESENT),
                                                     std::make_pair(hw_interface::HSD_SW_DIG_DDC_2_OPT_POLL_1,
                                                                    hw_interface::HSD_STA_NOT_PRESENT),
                                                     std::make_pair(hw_interface::HSD_SW_DIG_DDC_2_OPT_POLL_2,
                                                                    hw_interface::HSD_STA_NOT_PRESENT),
                                                    },
                                                    0xA6,
                                                    i2c_interface::i2c_interfaces::FE};

  const BoardSwitchPresenceCheckEEPROM24XX<3> ddc400_0{DDC400_0,
                                                       "DDC400",
                                                       0,
                                                       {
                                                        std::make_pair(hw_interface::HSD_SW_DIG_DDC_1_OPT_POLL_0,
                                                                       hw_interface::HSD_STA_PRESENT),
                                                        std::make_pair(hw_interface::HSD_SW_DIG_DDC_1_OPT_POLL_1,
                                                                       hw_interface::HSD_STA_PRESENT),
                                                        std::make_pair(hw_interface::HSD_SW_DIG_DDC_1_OPT_POLL_2,
                                                                       hw_interface::HSD_STA_NOT_PRESENT),
                                                       },
                                                       0xA4,
                                                       i2c_interface::i2c_interfaces::FE};

  const BoardSwitchPresenceCheckEEPROM24XX<3> ddc400_1{DDC400_1,
                                                       "DDC400",
                                                       1,
                                                       {
                                                        std::make_pair(hw_interface::HSD_SW_DIG_DDC_2_OPT_POLL_0,
                                                                       hw_interface::HSD_STA_PRESENT),
                                                        std::make_pair(hw_interface::HSD_SW_DIG_DDC_2_OPT_POLL_1,
                                                                       hw_interface::HSD_STA_PRESENT),
                                                        std::make_pair(hw_interface::HSD_SW_DIG_DDC_2_OPT_POLL_2,
                                                                       hw_interface::HSD_STA_NOT_PRESENT),
                                                       },
                                                       0xA6,
                                                       i2c_interface::i2c_interfaces::FE};

  const BoardSwitchPresenceCheckEEPROM24XX<3> wddc400_0{WDDC400_0,
                                                       "WDDC400",
                                                        0,
                                                        {
                                                         std::make_pair(hw_interface::HSD_SW_DIG_DDC_1_OPT_POLL_0,
                                                                        hw_interface::HSD_STA_PRESENT),
                                                         std::make_pair(hw_interface::HSD_SW_DIG_DDC_1_OPT_POLL_1,
                                                                        hw_interface::HSD_STA_NOT_PRESENT),
                                                         std::make_pair(hw_interface::HSD_SW_DIG_DDC_1_OPT_POLL_2,
                                                                        hw_interface::HSD_STA_PRESENT),
                                                        },
                                                        0xA4,
                                                        i2c_interface::i2c_interfaces::FE};

  const BoardSwitchPresenceCheckEEPROM24XX<3> wddc400_1{WDDC400_1,
                                                       "WDDC400",
                                                        1,
                                                        {
                                                         std::make_pair(hw_interface::HSD_SW_DIG_DDC_2_OPT_POLL_0,
                                                                        hw_interface::HSD_STA_PRESENT),
                                                         std::make_pair(hw_interface::HSD_SW_DIG_DDC_2_OPT_POLL_1,
                                                                        hw_interface::HSD_STA_NOT_PRESENT),
                                                         std::make_pair(hw_interface::HSD_SW_DIG_DDC_2_OPT_POLL_2,
                                                                        hw_interface::HSD_STA_PRESENT),
                                                        },
                                                        0xA6,
                                                        i2c_interface::i2c_interfaces::FE};

  const mmio_interface::MMIOInterface cor1_interface{0xD4000},
      cor2_interface{0xD5000};

  const BoardPresenceAndReadViaCor rxtx_0{RXTX_0, "RXTX", 0, cor1_interface};
  const BoardPresenceAndReadViaCor rxtx_1{RXTX_1, "RXTX", 1, cor2_interface};
  const BoardPresenceAndReadViaCor cor_0{COR_0, "COR", 0, cor1_interface};
  const BoardPresenceAndReadViaCor cor_1{COR_1, "COR", 1, cor2_interface};

  const std::array<std::reference_wrapper<const Board>, 14> boards = {
                                                                  //    {REF, "REF", 0},
                                                                  adc_0, adc_1, auc_0, auc_1,
                                                                  ddc_0, ddc_1, ddc400_0, ddc400_1, wddc400_0, wddc400_1,
                                                                  rxtx_0, rxtx_1, cor_0, cor_1
  };


}; // namespace board_interface

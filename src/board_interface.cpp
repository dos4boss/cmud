#include "logger.hpp"
#include "board_interface.hpp"
#include "hw_interface_helpers.hpp"
#include "mmio_interface.hpp"

#include <algorithm>

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
      logger::RAIIFlush raii_flusher(out);

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
                               const mmio_interface::CorrectionProcessorInterface &corpro_interface,
                               const uint16_t eeprom_interaction_msg)
      : Board(idx, name, number), corpro_interface_(corpro_interface),
        eeprom_interaction_msg_(eeprom_interaction_msg) {}

    bool is_present(std::ostream &out) const override {
      logger::RAIIFlush raii_flusher(out);
      return corpro_interface_.is_present(out);
    }

    std::vector<uint8_t> read_eeprom(const uint_fast16_t &memory_address,
                                     const uint_fast16_t &length,
                                     std::ostream &out) const override {
      logger::RAIIFlush raii_flusher(out);

      if (!is_present(out)) {
        LOGGER_ERROR("Requested board is not present");
        return {};
      }

      uint16_t remaining_length = std::min(length, uint_fast16_t(65000));
      uint_fast16_t next_memory_address = memory_address;

      std::vector<uint8_t> result(length);

      while (remaining_length > 0) {
        uint16_t next_segment_length = std::min(remaining_length, uint16_t(2032));

        const std::vector<uint16_t> data_in = {eeprom_interaction_msg_,
                                               uint16_t((next_memory_address >> 16) & 0xFFFF),
                                               uint16_t(next_memory_address & 0xFFFF),
                                               next_segment_length};

        auto [err, data] = corpro_interface_.interact(0x1B, std::chrono::seconds(10), data_in, next_segment_length >> 1, out);
        if (err != mmio_interface::error_code::CommandSuccessful) {
          LOGGER_ERROR("Correction processor communication failed ({})", mmio_interface::error_code_to_string(err));
          return {};
        }
        std::memcpy(result.data() + next_memory_address, data.data(), next_segment_length);

        next_memory_address += next_segment_length;
        remaining_length -= next_segment_length;
      }
      return result;
    }

  protected:
    const mmio_interface::CorrectionProcessorInterface &corpro_interface_;
    const uint16_t eeprom_interaction_msg_;
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
      (void)out;
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



  class I2CSwitcherUSURAII {
  public:
    using switch_with_value = std::pair<hw_interface::switch_id, hw_interface::switch_status>;

    I2CSwitcherUSURAII(const Board &board, std::ostream &out) : board_(board), out_(out) {
      if (board_.number_ > 1)
        throw std::runtime_error("Board number has to be 0 or 1");

      const std::array<std::array<switch_with_value, 3>, 2> general_switches = {
          {{std::make_pair(hw_interface::HSD_SW_USU1_I2C_SDA, hw_interface::HSD_STA_HIGH),
            std::make_pair(hw_interface::HSD_SW_USU1_I2C_SCL, hw_interface::HSD_STA_HIGH),
            std::make_pair(hw_interface::HSD_SW_USU1_I2C_SOURCE, hw_interface::HSD_STA_USU_I2C_SOURCE_FPGA)},
           {std::make_pair(hw_interface::HSD_SW_USU2_I2C_SDA, hw_interface::HSD_STA_HIGH),
            std::make_pair(hw_interface::HSD_SW_USU2_I2C_SCL, hw_interface::HSD_STA_HIGH),
            std::make_pair(hw_interface::HSD_SW_USU2_I2C_SOURCE, hw_interface::HSD_STA_USU_I2C_SOURCE_FPGA)}}};

      for (const auto &sw_with_val : general_switches[board_.number_])
        hw_interface::write_switch(sw_with_val.first, sw_with_val.second, out_);

      if ((board_.idx_ == board_idx::Bluetooth_0) || (board_.idx_ == board_idx::Bluetooth_1) ||
          (board_.idx_ == board_idx::USU_0) || (board_.idx_ == board_idx::USU_1)) {
        const std::array<std::array<switch_with_value, 2>, 2> bt_usu_switches = {
          {{std::make_pair(hw_interface::HSD_SW_USU1_I2C_PPC_ENABLE, hw_interface::HSD_STA_DISABLE),
            std::make_pair(hw_interface::HSD_SW_USU1_I2C_SP_ENABLE, hw_interface::HSD_STA_DISABLE)},
           {std::make_pair(hw_interface::HSD_SW_USU2_I2C_PPC_ENABLE, hw_interface::HSD_STA_DISABLE),
            std::make_pair(hw_interface::HSD_SW_USU2_I2C_SP_ENABLE, hw_interface::HSD_STA_DISABLE)}}};

        for (const auto &sw_with_val : bt_usu_switches[board_.number_])
          hw_interface::write_switch(sw_with_val.first, sw_with_val.second, out_);
      }

      if ((board_.idx_ == board_idx::PowerQuiccII_0) || (board_.idx_ == board_idx::PowerQuiccII_1) ||
          (board_.idx_ == board_idx::PowerQuiccIII_0) || (board_.idx_ == board_idx::PowerQuiccIII_1)) {

        const std::array<std::array<switch_with_value, 2>, 2> powerquicc_setup_switches = {
          {{std::make_pair(hw_interface::HSD_SW_USU1_PPC_CLK_ENABLE, hw_interface::HSD_STA_ENABLE),
            std::make_pair(hw_interface::HSD_SW_USU1_PPC_RESET, hw_interface::HSD_STA_ENABLE)},
           {std::make_pair(hw_interface::HSD_SW_USU2_PPC_CLK_ENABLE, hw_interface::HSD_STA_ENABLE),
            std::make_pair(hw_interface::HSD_SW_USU2_PPC_RESET, hw_interface::HSD_STA_ENABLE)}}};

        for (const auto &sw_with_val : powerquicc_setup_switches[board_.number_])
          hw_interface::write_switch(sw_with_val.first, sw_with_val.second, out_);

        std::this_thread::sleep_for(std::chrono::milliseconds(800));

        const std::array<std::array<switch_with_value, 2>, 2> powerquicc_switches = {
          {{std::make_pair(hw_interface::HSD_SW_USU1_I2C_PPC_ENABLE, hw_interface::HSD_STA_ENABLE),
            std::make_pair(hw_interface::HSD_SW_USU1_I2C_SP_ENABLE, hw_interface::HSD_STA_DISABLE)},
           {std::make_pair(hw_interface::HSD_SW_USU2_I2C_PPC_ENABLE, hw_interface::HSD_STA_ENABLE),
            std::make_pair(hw_interface::HSD_SW_USU2_I2C_SP_ENABLE, hw_interface::HSD_STA_DISABLE)}}};

        for (const auto &sw_with_val : powerquicc_switches[board_.number_])
          hw_interface::write_switch(sw_with_val.first, sw_with_val.second, out_);
      }

    }

    ~I2CSwitcherUSURAII(void) {
      const std::array<std::array<switch_with_value, 3>, 2> general_switches = {
          {{std::make_pair(hw_interface::HSD_SW_USU1_I2C_PPC_ENABLE, hw_interface::HSD_STA_DISABLE),
            std::make_pair(hw_interface::HSD_SW_USU1_I2C_SP_ENABLE, hw_interface::HSD_STA_DISABLE),
            std::make_pair(hw_interface::HSD_SW_USU1_I2C_SOURCE, hw_interface::HSD_STA_USU_I2C_SOURCE_MC)},
           {std::make_pair(hw_interface::HSD_SW_USU2_I2C_PPC_ENABLE, hw_interface::HSD_STA_DISABLE),
            std::make_pair(hw_interface::HSD_SW_USU2_I2C_SP_ENABLE, hw_interface::HSD_STA_DISABLE),
            std::make_pair(hw_interface::HSD_SW_USU2_I2C_SOURCE, hw_interface::HSD_STA_USU_I2C_SOURCE_MC)}}};

      for (const auto &sw_with_val : general_switches[board_.number_])
        hw_interface::write_switch(sw_with_val.first, sw_with_val.second, out_);
    }

  private:
    const Board &board_;
    std::ostream &out_;
  };


  class I2CSwitcherAudioRAII {
  public:
    I2CSwitcherAudioRAII(const Board &board, std::ostream &out) : board_(board), out_(out) {
      const auto switch_status = hw_interface::read_switch_status(hw_interface::HSD_SW_AUDIO_I2C_BUSY_READ, out_);
      if (switch_status.value() == hw_interface::HSD_STA_AUDIO_I2C_BUSY)
        throw std::runtime_error("Audio I2C still busy");

      hw_interface::write_switch(hw_interface::HSD_SW_AUDIO_I2C_BUSY_WRITE, hw_interface::HSD_STA_AUDIO_I2C_BUSY, out_);
    }

    ~I2CSwitcherAudioRAII(void) {
      hw_interface::write_switch(hw_interface::HSD_SW_AUDIO_I2C_BUSY_WRITE, hw_interface::HSD_STA_AUDIO_I2C_NOT_BUSY, out_);
    }

  private:
    const Board &board_;
    std::ostream &out_;
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

  const mmio_interface::CorrectionProcessorInterface cor1_interface{0},
             cor2_interface{1};

  const BoardPresenceAndReadViaCor rxtx_0{RXTX_0, "RXTX", 0, cor1_interface, 0x03};
  const BoardPresenceAndReadViaCor rxtx_1{RXTX_1, "RXTX", 1, cor2_interface, 0x03};
  const BoardPresenceAndReadViaCor cor_0{COR_0, "COR", 0, cor1_interface, 0x08};
  const BoardPresenceAndReadViaCor cor_1{COR_1, "COR", 1, cor2_interface, 0x08};
  const BoardPresenceAndReadViaCor rxif3_0{RXIF3_0, "RXIF3", 0, cor1_interface, 0x12};
  const BoardPresenceAndReadViaCor rxif3_1{RXIF3_1, "RXIF3", 0, cor2_interface, 0x12};

  const std::array<std::reference_wrapper<const Board>, 14> boards = {
                                                                  //    {REF, "REF", 0},
                                                                  adc_0, adc_1, auc_0, auc_1,
                                                                  ddc_0, ddc_1, ddc400_0, ddc400_1, wddc400_0, wddc400_1,
                                                                  rxtx_0, rxtx_1, cor_0, cor_1
  };


}; // namespace board_interface

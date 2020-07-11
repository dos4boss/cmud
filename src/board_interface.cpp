#include "logger.hpp"
#include "board_interface.hpp"
#include "hw_interface_helpers.hpp"
#include "correction_processor_interface.hpp"

namespace board_interface {
  MAKE_LOCAL_LOGGER("board_interface");

  enum ddc_type {DDC,
                 DDC400,
                 WDDC400,
                 NOT_PRESENT};

  ddc_type get_ddc_type(const unsigned &board_idx, std::ostream &out) {
    hw_interface::switch_id switch_0, switch_1, switch_2;
    if (board_idx == 0) {
      switch_0 = hw_interface::HSD_SW_DIG_DDC_1_OPT_POLL_0;
      switch_1 = hw_interface::HSD_SW_DIG_DDC_1_OPT_POLL_1;
      switch_2 = hw_interface::HSD_SW_DIG_DDC_1_OPT_POLL_2;
    }
    else if (board_idx == 1) {
      switch_0 = hw_interface::HSD_SW_DIG_DDC_2_OPT_POLL_0;
      switch_1 = hw_interface::HSD_SW_DIG_DDC_2_OPT_POLL_1;
      switch_2 = hw_interface::HSD_SW_DIG_DDC_2_OPT_POLL_2;
    }
    auto switch_val_0 = hw_interface::read_switch(switch_0, out);
    auto switch_val_1 = hw_interface::read_switch(switch_1, out);
    auto switch_val_2 = hw_interface::read_switch(switch_2, out);

    if (!switch_val_0.has_value() || !switch_val_1.has_value() ||
        !switch_val_2.has_value())
      return NOT_PRESENT;

    if ((switch_val_0.value() == hw_interface::HSD_STA_PRESENT) &&
        (switch_val_1.value() != hw_interface::HSD_STA_PRESENT) &&
        (switch_val_2.value() != hw_interface::HSD_STA_PRESENT))
      return DDC;

    if ((switch_val_0.value() == hw_interface::HSD_STA_PRESENT) &&
        (switch_val_1.value() == hw_interface::HSD_STA_PRESENT) &&
        (switch_val_2.value() != hw_interface::HSD_STA_PRESENT))
      return DDC400;

    if ((switch_val_0.value() == hw_interface::HSD_STA_PRESENT) &&
        (switch_val_1.value() != hw_interface::HSD_STA_PRESENT) &&
        (switch_val_2.value() == hw_interface::HSD_STA_PRESENT))
      return WDDC400;

    return NOT_PRESENT;
  }

  bool check_if_module_present_single_switch_board_idx(const hw_interface::switch_id &switch_0,
                                                       const hw_interface::switch_id &switch_1,
                                                       const unsigned &board_idx,
                                                       const hw_interface::switch_status &success_status,
                                                       std::ostream &out) {
    if (board_idx == 0) {
      if (auto switch_val = hw_interface::read_switch(switch_0, out))
        return switch_val.value() == success_status;
    }
    else if (board_idx == 1) {
      if (auto switch_val = hw_interface::read_switch(switch_1, out))
        return switch_val.value() == success_status;
    }
    else
      LOGGER_ERROR("Invalid board idx");
    return false;
  }

  bool check_if_module_present(const std::string &board_id, std::ostream &out,
                               const unsigned &board_idx) {
    if (board_id == "ADC") {
      return check_if_module_present_single_switch_board_idx(hw_interface::HSD_SW_DIG_ADC_1_OPT_POLL_0,
                                                             hw_interface::HSD_SW_DIG_ADC_2_OPT_POLL_0,
                                                             board_idx,
                                                             hw_interface::switch_status::HSD_STA_PRESENT,
                                                             out);
    }
    else if (board_id == "DDC") {
      return get_ddc_type(board_idx, out) == DDC;
    }
    else if (board_id == "DDC400") {
      return get_ddc_type(board_idx, out) == DDC400;
    }
    else if (board_id == "WDDC400") {
      return get_ddc_type(board_idx, out) == WDDC400;
    }
    else if (board_id == "AUC") {
      return check_if_module_present_single_switch_board_idx(hw_interface::HSD_SW_DIG_AUC_1_OPT_POLL_0,
                                                             hw_interface::HSD_SW_DIG_AUC_2_OPT_POLL_0,
                                                             board_idx,
                                                             hw_interface::switch_status::HSD_STA_PRESENT,
                                                             out);
    }
    else if (board_id == "DUC") {
      return check_if_module_present_single_switch_board_idx(hw_interface::HSD_SW_DIG_DUC1_PRESENT,
                                                             hw_interface::HSD_SW_DIG_DUC2_PRESENT,
                                                             board_idx,
                                                             hw_interface::switch_status::HSD_STA_DIG_DUC_PRESENT,
                                                             out);
    }
    else if (board_id == "RXTX") {
      if (board_idx == 0) {
        correction_processor_interface::CorrectionProcessorInterface cor_pro(0xD4000);
        return cor_pro.is_present();
      }
      else if (board_idx == 1) {
        correction_processor_interface::CorrectionProcessorInterface cor_pro(0xD5000);
        return cor_pro.is_present();
      }
    }
    return false;
  }

}; // namespace board_interface

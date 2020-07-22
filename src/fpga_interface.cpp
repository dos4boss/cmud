#include "fpga_interface.hpp"
#include "logger.hpp"
#include "hw_interface_helpers.hpp"
#include <thread>
#include <chrono>

namespace fpga_interface {

  MAKE_LOCAL_LOGGER("fpga_interface");

  void set_mode(const fpga &fpga, const fpga_mode &fpga_mode, std::ostream &out) {
    logger::RAIIFlush raii_flush(out);

    hw_interface::switch_status switch_state;

    if (fpga_mode == fpga_mode::PROGRAMM)
      switch_state = hw_interface::HSD_STA_DIG_PRG;
    else if (fpga_mode == fpga_mode::RUN)
      switch_state = hw_interface::HSD_STA_DIG_RUN;
    else {
      LOGGER_ERROR("fpga_mode unknown");
      return;
    }

    hw_interface::switch_id switch_id;
    if (fpga == fpga::AT_DSP)
      switch_id = hw_interface::HSD_SW_DIG_XIL_AT_DSP;
    else if (fpga == fpga::LINK)
      switch_id = hw_interface::HSD_SW_DIG_XIL_LINK;
    else if (fpga == fpga::MOD_DEMOD)
      switch_id = hw_interface::HSD_SW_DIG_XIL_MOD_DEMOD;
    else {
      LOGGER_ERROR("fpga unknown");
      return;
    }

    hw_interface::write_switch(switch_id, switch_state, out);
  }

  bool get_init(const fpga &fpga, std::ostream &out) {
    logger::RAIIFlush raii_flush(out);

    std::optional<hw_interface::switch_status> switch_state;

    if (fpga == fpga::AT_DSP)
      switch_state = hw_interface::read_switch_status(hw_interface::HSD_SW_DIG_XIL_AT_DSP_INIT, out);
    else if (fpga == fpga::LINK)
      switch_state = hw_interface::read_switch_status(hw_interface::HSD_SW_DIG_XIL_LINK_INIT, out);
    else if (fpga == fpga::MOD_DEMOD)
      switch_state = hw_interface::read_switch_status(hw_interface::HSD_SW_DIG_XIL_MOD_DEMOD_INIT, out);
    else {
      LOGGER_ERROR("fpga unknown");
      return false;
    }

    return switch_state.value() == hw_interface::HSD_STA_DIG_INIT;
  }

  bool get_done(const fpga &fpga, std::ostream &out) {
    logger::RAIIFlush raii_flush(out);

    std::optional<hw_interface::switch_status> switch_state;

    if (fpga == fpga::AT_DSP)
      switch_state = hw_interface::read_switch_status(hw_interface::HSD_SW_DIG_XIL_AT_DSP_DONE, out);
    else if (fpga == fpga::LINK)
      switch_state = hw_interface::read_switch_status(hw_interface::HSD_SW_DIG_XIL_LINK_DONE, out);
    else if (fpga == fpga::MOD_DEMOD) {
      return (hw_interface::read_switch_status(hw_interface::HSD_SW_DIG_XIL_MOD_DONE, out).value() == hw_interface::HSD_STA_DIG_DONE) &&
        (hw_interface::read_switch_status(hw_interface::HSD_SW_DIG_XIL_MOD_DONE, out).value() == hw_interface::HSD_STA_DIG_DONE);
    } else {
      LOGGER_ERROR("fpga unknown");
      return false;
    }

    return switch_state.value() == hw_interface::HSD_STA_DIG_DONE;
  }

  bool init(const fpga &fpga, std::ostream &out) {
    logger::RAIIFlush raii_flush(out);

    set_mode(fpga, fpga_mode::PROGRAMM, out);
    size_t loop_counter = 0;
    bool is_done, is_init;
    do {
      is_done = get_done(fpga, out);
      is_init = get_init(fpga, out);

      if (loop_counter >= 5) {
        LOGGER_ERROR("Failed to reset");
        return false;
      }

      loop_counter++;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while(!(is_init && !is_done));

    set_mode(fpga, fpga_mode::RUN, out);

    loop_counter = 0;
    do {
      is_init = get_init(fpga, out);

      if (loop_counter >= 10) {
        LOGGER_ERROR("Failed to initiate reprogramming");
        return false;
      }

      loop_counter++;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    } while(is_init);

    return true;
  }

} // namespace fpga_interface

#include "fpga_interface.hpp"
#include "logger.hpp"
#include "hw_interface_helpers.hpp"
#include <thread>
#include <chrono>
#include <fstream>
#include <iterator>

namespace fpga_interface {

  MAKE_LOCAL_LOGGER("fpga_interface");

  std::string fpga_to_string(const fpga &fpga) {
    if (fpga == fpga::AT_DSP)
      return "AT_DSP";
    else if (fpga == fpga::LINK)
      return "LINK";
    else if (fpga == fpga::MOD_DEMOD)
      return "MOD_DEMOD";
    else {
      LOGGER_ERROR("fpga unknown");
      return "";
    }
  }

  fpga string_to_fpga(const std::string &fpga) {
    if (fpga == "AT_DSP")
      return fpga::AT_DSP;
    else if (fpga == "LINK")
      return fpga::LINK;
    else if (fpga == "MOD_DEMOD")
      return fpga::MOD_DEMOD;
    else {
      LOGGER_ERROR("fpga unknown");
      return fpga::INVALID;
    }
  }

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

    hw_interface::switch_id switch_id;

    if (fpga == fpga::AT_DSP)
      switch_id = hw_interface::HSD_SW_DIG_XIL_AT_DSP_INIT;
    else if (fpga == fpga::LINK)
      switch_id = hw_interface::HSD_SW_DIG_XIL_LINK_INIT;
    else if (fpga == fpga::MOD_DEMOD)
      switch_id = hw_interface::HSD_SW_DIG_XIL_MOD_DEMOD_INIT;
    else {
      LOGGER_ERROR("fpga unknown");
      return false;
    }

    return hw_interface::read_switch_status(switch_id, out).value() ==
      hw_interface::HSD_STA_DIG_INIT;
  }

  bool get_busy(const fpga &fpga, std::ostream &out) {
    logger::RAIIFlush raii_flush(out);

    hw_interface::switch_id switch_id;

    if (fpga == fpga::AT_DSP)
      switch_id = hw_interface::HSD_SW_DIG_XIL_AT_DSP_BUSY;
    else if (fpga == fpga::LINK)
      switch_id = hw_interface::HSD_SW_DIG_XIL_LINK_BUSY;
    else if (fpga == fpga::MOD_DEMOD)
      switch_id = hw_interface::HSD_SW_DIG_MOD_DEMOD_XIL_DATA_BUSY;
    else {
      LOGGER_ERROR("fpga unknown");
      return false;
    }

    return hw_interface::read_switch_status(switch_id, out).value() ==
           hw_interface::HSD_STA_DIG_BUSY;
  }

  bool get_done(const fpga &fpga, std::ostream &out) {
    logger::RAIIFlush raii_flush(out);

    const auto check_for_done =
        [&out](const hw_interface::switch_id &switch_id) {
          return hw_interface::read_switch_status(switch_id, out).value() ==
                 hw_interface::HSD_STA_DIG_DONE;
        };

    if (fpga == fpga::AT_DSP)
      return check_for_done(hw_interface::HSD_SW_DIG_XIL_AT_DSP_DONE);
    else if (fpga == fpga::LINK)
      return check_for_done(hw_interface::HSD_SW_DIG_XIL_LINK_DONE);
    else if (fpga == fpga::MOD_DEMOD) {
      return check_for_done(hw_interface::HSD_SW_DIG_XIL_MOD_DONE) &&
             check_for_done(hw_interface::HSD_SW_DIG_XIL_DEMOD_DONE);
    } else {
      LOGGER_ERROR("fpga unknown");
      return false;
    }
  }

  bool init(const fpga &fpga, std::ostream &out) {
    logger::RAIIFlush raii_flush(out);

    set_mode(fpga, fpga_mode::PROGRAMM, out);

    auto timeout = std::chrono::high_resolution_clock::now() +
                   std::chrono::milliseconds(5);

    while (!get_init(fpga, out) || get_done(fpga, out)) {
      if (std::chrono::high_resolution_clock::now() > timeout) {
        LOGGER_ERROR("Failed to reset {} FPGA", fpga_to_string(fpga));
        return false;
      }
    }

    set_mode(fpga, fpga_mode::RUN, out);

    timeout = std::chrono::high_resolution_clock::now() +
              std::chrono::milliseconds(20);

    while (std::chrono::high_resolution_clock::now() < timeout) {
      if (!get_init(fpga, out)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        return true;
      }
    }

    LOGGER_ERROR("Failed to initiate reprogramming {} FPGA",
                 fpga_to_string(fpga));
    return false;
  }

  inline bool ready_for_next_byte(const fpga &fpga, std::ostream &out) {
    return !get_busy(fpga, out) && !get_init(fpga, out);
  }

  bool wait_for_ready(const fpga &fpga, std::ostream &out) {
    logger::RAIIFlush raii_flush(out);

    const auto timeout = std::chrono::high_resolution_clock::now() +
      std::chrono::milliseconds(10);

    while (true) {
      if (ready_for_next_byte(fpga, out))
        return true;

      while (!get_busy(fpga, out)) {
        if (std::chrono::high_resolution_clock::now() > timeout) {
          LOGGER_ERROR("Failed during wait for ready of {} FPGA",
                       fpga_to_string(fpga));
          return false;
        }
      }

      while(get_init(fpga, out)) {
        if (std::chrono::high_resolution_clock::now() > timeout) {
          LOGGER_ERROR("Init error while loading {} FPGA",
                       fpga_to_string(fpga));
          return false;
        }
      }
    }
  }

  bool load_fpga(const fpga &fpga, const std::string &file_path, std::ostream &out) {
    logger::RAIIFlush raii_flush(out);

    std::ifstream binary_file(file_path, std::ios::binary);
    std::istreambuf_iterator binary_file_iter(binary_file);

    if (binary_file.fail()) {
      LOGGER_ERROR("Could not open file {}", file_path);
      return false;
    }

    hw_interface::switch_id switch_id;
    if (fpga == fpga::AT_DSP)
      switch_id = hw_interface::HSD_SW_DIG_LOAD_XIL_AT_DSP;
    else if (fpga == fpga::LINK)
      switch_id = hw_interface::HSD_SW_DIG_LOAD_XIL_LINK;
    //else if (fpga == fpga::MOD_DEMOD)
    //  switch_id = hw_interface::HSD_SW_DIG_XIL_MOD_DEMOD; //TODO
    else {
      LOGGER_ERROR("fpga unknown");
      return false;
    }

    hw_interface::access_width access_width;
    access_width = hw_interface::stream_infos[hw_interface::switch_infos[switch_id].stream_id].access_width;

    if (!init(fpga, out))
      return false;

    while (binary_file_iter != std::istreambuf_iterator<char>()) {
      if(wait_for_ready(fpga, out))
        return false;

      uint32_t value = 0;
      if (access_width == hw_interface::access_width::BYTE)
        value = from_byte_iterator<uint8_t>(binary_file_iter);
      else if (access_width == hw_interface::access_width::WORD)
        value = from_byte_iterator<uint16_t>(binary_file_iter);
      else if (access_width == hw_interface::access_width::DWORD)
        value = from_byte_iterator<uint32_t>(binary_file_iter);

      hw_interface::write_switch(switch_id, value, out);
    }

    if (get_init(fpga, out)) {
      LOGGER_ERROR("Error after loading {} FPGA (init)", fpga_to_string(fpga));
      return false;
    }

    const auto timeout = std::chrono::high_resolution_clock::now() +
                         std::chrono::milliseconds(2);

    while (!get_done(fpga, out)) {
      if (std::chrono::high_resolution_clock::now() > timeout) {
        LOGGER_ERROR("Error after loading {} FPGA (done)", fpga_to_string(fpga));
        return false;
      }
    }

    return true;
  }

  bool load_fpga(const std::string &fpga_str, const std::string &file_path, std::ostream &out) {
    logger::RAIIFlush raii_flush(out);

    const auto fpga = string_to_fpga(fpga_str);
    return load_fpga(fpga, file_path, out);
  }

} // namespace fpga_interface

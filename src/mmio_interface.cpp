#include "logger.hpp"
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

#include <stdexcept>

#include "mmio_interface.hpp"

namespace mmio_interface {
  MAKE_LOCAL_LOGGER("mmio");

  MMIOInterface::MMIOInterface(const unsigned &isa_base_address_) {
    _isa_base_address = isa_base_address_;

    _memfd = open("/dev/mem", O_RDWR | O_SYNC);
    if (_memfd < 0)
      throw std::runtime_error("Failed to open '/dev/mem'. Must be run as root.");
    _isa_base_ptr = (uint8_t *)mmap(0, _isa_size, PROT_READ | PROT_WRITE,
                                    MAP_SHARED, _memfd, _isa_base_address);
    if (_isa_base_ptr == MAP_FAILED)
      throw std::runtime_error("Memory mapping failed.");
  }

  MMIOInterface::~MMIOInterface(void) {
    close(_memfd);
  }

  template <typename T>
  std::vector<T> MMIOInterface::read(const unsigned &offset,
                                     const unsigned &number_elements,
                                     std::ostream &out) const {
    static_assert(std::is_integral<T>::value, "Integral required");
    logger::RAIIFlush raii_flush(out);

    if (offset + (number_elements * sizeof(T)) >= _isa_size)
      throw std::runtime_error("Offset too large.");

    std::vector<T> result(number_elements);
    for(unsigned idx = 0; idx < result.size(); ++idx) {
      result[idx] = *(T*)(_isa_base_ptr + offset + (idx * sizeof(T)));
      LOGGER_INFO("Reading 0x{:0{}X} @ 0x{:08X}", result[idx], 2 * sizeof(T),
                  _isa_base_address + offset + (idx * sizeof(T)));
    }
    return result;
  }

  template <typename T>
  void MMIOInterface::write(const unsigned &offset,
                            const std::vector<T> data,
                            std::ostream &out) const {
    static_assert(std::is_integral<T>::value, "Integral required");
    logger::RAIIFlush raii_flush(out);

    if (offset + data.size() * sizeof(T) >= _isa_size)
      throw std::runtime_error("Offset too large.");

    size_t idx = 0;
    for(const auto &datum : data) {
      LOGGER_INFO("Writing 0x{:0{}X} @ 0x{:08X}", datum, 2 * sizeof(T), _isa_base_address + offset + idx);
      *(T*)(_isa_base_ptr + offset + idx) = datum;
      idx += sizeof(T);
    }
  }


  bool CorrectionProcessorInterface::is_present(std::ostream &out) const {
      std::vector<uint8_t> check_data = {0x55, 0xaa};
      auto restore_data = read<uint8_t>(0x7fa, 2, out);
      write(0x7fa, check_data, out);
      auto readback_data = read<uint8_t>(0x7fa, 2, out);
      write(0x7fa, restore_data, out);
      return check_data == readback_data;
  }


  std::string error_code_to_string(const error_code &err) {
    switch(err) {
    case error_code::CommandSuccessful: return "Command successful";
    case error_code::GeneralError: return "General error";
    case error_code::TableNotFound: return "Table not found";
    case error_code::OutOfMemory: return "Out of memory";
    case error_code::ChecksumError: return "Checksum error";
    case error_code::DataExceedsMemoryLimits: return "Data exceeds memory limits";
    case error_code::PermissionDenied: return "Permission denied";
    case error_code::TooManyOpenTables: return "Too many open tables";
    case error_code::EPROMBufferOverflow: return "EPROM Buffer overflow";
    case error_code::TableBlockNotValid: return "Table block not valid";
    case error_code::IndexOutOfRange: return "Index out of range";
    case error_code::UnknownCommand: return "Unknown Command";
    case error_code::TableIdNotValid: return "Table ID not valid";
    case error_code::NoSettingTable: return "No setting table";
    case error_code::NoDeviationTable: return "No deviation table";
    case error_code::HardwareNotSupported: return "Hardware not supported";
    case error_code::InvalidSwitchSetting: return "Invalid switch setting";
    case error_code::InvalidSignalDirection: return "Invalid signal direction";
    case error_code::InvalidArgument: return "Invalid argument";
    case error_code::InvalidFrequencyBand: return "Invalid frequency band";
    case error_code::IllegalMemorySpecifier: return "Illegal memory specifier";
    case error_code::FlashError: return "Flash error";
    case error_code::I2CError: return "I2C error";
    case error_code::DeviceError: return "Device error";
    case error_code::TooManyFrequencies: return "Too many frequencies";
    case error_code::WrongEndianess: return "Wrong endianess";
    case error_code::UnknownDatatype: return "Unknown datatype";
    case error_code::TimeoutError: return "Timeout error";
    case error_code::ProcessorBusy: return "Processor busy";
    case error_code::ProcessorDoesNotRespond: return "Process does not respond";
    case error_code::CommunicationError: return "Communication error";
    case error_code::UnknownParameterId: return "Unknown parameter ID";
    case error_code::InvalidParameterSize: return "Invalid parameter size";
    case error_code::InvalidMessageStructure: return "Invalid message structure";
    case error_code::ParameterTooLarge: return "Parameter too large";
    case error_code::CheckrepairFailed: return "Checkrepair failed";
    case error_code::LevelOverflow: return "Level overflow";
    case error_code::OptionalTableNotFound: return "Optional table not found";
    case error_code::BitPatternTestFailed: return "Bit pattern test failed";
    case error_code::AddressTestFailed: return "Address test failed";

    default:
      return "Unkown error code";
    }
  }

  error_code CorrectionProcessorInterface::wait_for_status_or_timeout(const uint_fast8_t &status,
                                                                      const std::chrono::microseconds &timeout,
                                                                      std::ostream &out) const {
    error_code err = error_code::CommandSuccessful;
    uint_fast8_t readback_status;
    const auto start_time = std::chrono::high_resolution_clock::now();

    do {
      readback_status = read<uint8_t>(1, 1, out)[0];
      read<uint8_t>(0x7fc, 1, out);
      std::this_thread::sleep_for(std::chrono::microseconds(100));

      if (readback_status == 5) {
        read<uint16_t>(4, 0x4f, out); // TODO
        write(0, std::vector<uint8_t>{5}, out);

        const auto extra_start_time = std::chrono::high_resolution_clock::now();
        do {
          if (read<uint8_t>(1, 1, out)[0] != 5) break;
        } while (std::chrono::high_resolution_clock::now() - extra_start_time < std::chrono::seconds(1));

        write(0, std::vector<uint8_t>{0}, out);
      }
      else if (((std::chrono::high_resolution_clock::now() - start_time) > timeout)
               && (readback_status != status)) {
        write(0, std::vector<uint8_t>{0}, out);
        err = error_code::TimeoutError;
      }

      if ((err != error_code::CommandSuccessful) || (readback_status == status))
        return err;
    } while (true);
  }

  template <typename T_in, typename T_out>
  std::pair<error_code, std::vector<T_out>>
  CorrectionProcessorInterface::interact(const uint_fast8_t &status,
                                         const std::chrono::microseconds &timeout,
                                         const std::vector<T_in> &data_in,
                                         const uint_fast16_t &number_elements_out,
                                         std::ostream &out) const {
    error_code err = error_code::CommandSuccessful;

    if (wait_for_status_or_timeout(0, std::chrono::seconds(2), out) == error_code::CommandSuccessful) {
      write(4, data_in, out);
      write(0, std::vector<uint8_t>{status}, out);
      if (read<uint8_t>(0, 1, out)[0] == status) {
        if (wait_for_status_or_timeout(status, std::chrono::seconds(10), out) == error_code::CommandSuccessful) {
          write(0, std::vector<uint8_t>{0}, out);
          err = wait_for_status_or_timeout(0, timeout, out);
          if (err == error_code::CommandSuccessful)
            err = error_code(read<int16_t>(2, 2, out)[0]);
        }
        else err = error_code::ProcessorDoesNotRespond;
      }
      else err = error_code::CommunicationError;
    }
    else err = error_code::ProcessorBusy;

    std::vector<T_out> result{read<T_out>(4, number_elements_out, out)};
    if (err != error_code::CommandSuccessful)
      write(0, std::vector<uint8_t>{0}, out);

    return std::make_pair(err, result);
  }

  std::pair<error_code, std::vector<uint16_t>>
  CorrectionProcessorInterface::interact(const uint_fast8_t &status,
                                         const std::chrono::microseconds &timeout,
                                         const std::vector<uint16_t> &data_in,
                                         const uint_fast16_t &number_elements_out,
                                         std::ostream &out) const {
    return interact<uint16_t, uint16_t>(status, timeout, data_in, number_elements_out, out);
  }


  uint_fast16_t CorrectionProcessorInterface::get_version(std::ostream &out) const {
    const auto [err, result] = interact<uint8_t, uint16_t>(0x0C, std::chrono::seconds(1), {}, 0x30 >> 1, out);
    if (err == error_code::CommandSuccessful) {
      char second_val = '.', fourth_val = 'i';
      if ((result[0] >= 0x53C) && (*(((char*)result.data()) + 0x2C) != 'V')) {
        second_val = 'X';
        // TODO no idea what is going on here
        // char decision_val = *(((char*)result.data()) + 0x2D);
        // if ((result[0] >= 0x70A) && (decision_val >= 0x61) && (decision_val <= 0x7A)) {
        //   fourth_val = decision_val;

        // }
      }

      out << fmt::format("Version {}{}{:02}{}, Date {}-{:02}-{:02}",
                         result[0] >> 8, second_val, result[0] & 0xFF, fourth_val,
                         (result[1] >> 9) + 1980, (result[1] >> 5) & 0xF, result[1] & 0x1F) << std::endl;
      if (result[0] >= 0x500)
        out << fmt::format("{:.{}}", (char*)(result.data() + 2), 40) << std::endl;

      return (result[0] & 0xFF) + ((result[0] & 0xFFFF) >> 8) * 100;
    }
    else
      LOGGER_ERROR("Correction processor communication failed ({})", error_code_to_string(err));
    return 0;
  }

  struct cor_board_info {
    uint16_t board_revision;
    uint16_t read_code_0, read_code_1;
    uint16_t fpga_type;
    uint8_t product_index_0, product_index_1;
    uint16_t product_year;
    uint8_t product_month, product_day;
    uint8_t test_instruction_0, test_instruction_1;
    uint16_t part_number_0, part_number_1;
    uint8_t part_number_2;
    uint8_t run_state;
    uint8_t hw_code_0, hw_code_1;
    uint32_t serial_number_0;
    uint16_t serial_number_1;
    uint8_t user_correction_rx;
    uint8_t user_correction_tx;
    uint16_t unknown;
  } __attribute__((packed));

  void CorrectionProcessorInterface::get_board_info(std::ostream &out) const {
    const auto version = get_version(out);

    if (version >= 580) {
      const auto [err, result] = interact<uint8_t, uint16_t>(0x55, std::chrono::seconds(1), {}, 0x22 >> 1, out);
      cor_board_info *cor_board_info = (struct cor_board_info*)result.data();
      if (err == error_code::CommandSuccessful) {
        out << fmt::format("Part number:           {:04}.{:04}.{:02}\n"
                           "HW-Code EEPROM/FPGA:   {:02}/{:02}\n"
                           "Product index:         {:02}.{:02}\n"
                           "Serial number:         {:06}/{:03}\n"
                           "Product date:          {:04}-{:02}-{:02}\n"
                           "Read code EEPROM/INT:  {}/{}\n"
                           "Test instruction:      {:02}.{:02}\n"
                           "Board revision:        DG{:02}\n"
                           "Run state:             {}\n"
                           "FPGA type:             {}\n",
                           cor_board_info->part_number_0, cor_board_info->part_number_1, cor_board_info->part_number_2,
                           cor_board_info->hw_code_0, cor_board_info->hw_code_1,
                           cor_board_info->product_index_0, cor_board_info->product_index_1,
                           cor_board_info->serial_number_0, cor_board_info->serial_number_1,
                           cor_board_info->product_year, cor_board_info->product_month, cor_board_info->product_day,
                           cor_board_info->read_code_0, cor_board_info->read_code_1,
                           cor_board_info->test_instruction_0, cor_board_info->test_instruction_1,
                           cor_board_info->board_revision,
                           cor_board_info->run_state,
                           cor_board_info->fpga_type);
        if (version >= 650) {
          out << fmt::format("User correction RX:    RF4IN:  {:b}, RF2IN:  {:b}, RF1IN:  {:b}\n"
                             "User correction TX:    RF3OUT: {:b}, RF2OUT: {:b}, RF1OUT: {:b}\n",
                             cor_board_info->user_correction_rx & 4,
                             cor_board_info->user_correction_rx & 2,
                             cor_board_info->user_correction_rx & 1,
                             cor_board_info->user_correction_tx & 4,
                             cor_board_info->user_correction_tx & 2,
                             cor_board_info->user_correction_tx & 1);
        }
        if (version >= 631) {
          if (version >= 800) {
            out << fmt::format("Syncon ref.-divider:   {}\n", cor_board_info->unknown == 1 ? "10E016" : "RFDIV");
          } else {
            out << fmt::format("Frontend type:         {}\n", cor_board_info->unknown == 1 ? "RF1 = RF2" : "RF1 <> RF2");
          }
        }

      }
      else
        LOGGER_ERROR("Correction processor communication failed ({})", error_code_to_string(err));
    }
  }

  struct cor_status {
    uint16_t operation_mode;
    uint16_t meas_mode;
    uint16_t connector;
    uint16_t active_channel;
    uint16_t next_active_channel;
    uint16_t edit_channel;
    uint16_t background_operation;
    double frequency;
    double wideband_level;
    double narrowband_level;
    double alternate_level;
    double level_first_mixer;
    double level_if_io;
    double fe_temperature;
    double iqif_temperature;
    double rxtx_rf_temperature;
    double rxtx_if_temperature;
    uint16_t meas_mode_1;
    uint16_t rf_gain_bank_1;
    uint16_t rf_gain_bank_2;
    uint16_t if_coarse_setting_bank_1;
    uint16_t if_coarse_setting_bank_2;
    uint16_t if_fine_setting_bank_1;
    uint16_t if_fine_setting_bank_2;
    uint16_t stared_idx;
    double level_if_io_arr[3];
    double external_gain;
    double if_amp_switch_level;
  } __attribute__((packed));

  void CorrectionProcessorInterface::get_status(const rx_tx rx_tx, std::ostream &out) const {
    const auto version = get_version(out);

    const auto [err, result] = interact<uint16_t, uint16_t>(0x0B, std::chrono::seconds(1), {rx_tx}, 0x96 >> 1, out);
    struct cor_status *cor_status = (struct cor_status*)result.data();
    if (err == error_code::CommandSuccessful) {
      out << fmt::format("Operation Mode       :       0x{:04X}\n"
                         "Connector            :       0x{:04X}\n",
                         cor_status->operation_mode,
                         cor_status->connector);

      if (version < 640) {
        out << fmt::format("Measurement Mode 0   :       0x{:04X}\n"
                           "Measurement Mode 1   :       0x{:04X}\n",
                           cor_status->meas_mode,
                           cor_status->meas_mode_1);
      } else {
        out << fmt::format("External Gain        :      {:7.2f} dBm\n"
                           "IF Amp. Switch Level :      {:7.2f} dBm\n",
                           cor_status->external_gain,
                           cor_status->if_amp_switch_level);
      }

      out << fmt::format("Active Channel       :         {:4}\n"
                         "Next Active Channel  :         {:4}\n"
                         "Edit Channel         :         {:4}\n"
                         "Background Operation :         {:4}\n"
                         "Frequency            : {:12.7f} MHz\n"
                         "Wideband Level       :      {:7.2f} dBm\n"
                         "Narrowband Level     :      {:7.2f} dBm\n"
                         "Alternate Level      :      {:7.2f} dB\n"
                         "Level at 1st mixer   :      {:7.2f} dBm\n"
                         "Level at IF I/O      :      {:7.2f} dBm",
                         cor_status->active_channel,
                         cor_status->next_active_channel,
                         cor_status->edit_channel,
                         cor_status->background_operation,
                         cor_status->frequency / 1.0E6,
                         cor_status->wideband_level,
                         cor_status->narrowband_level,
                         cor_status->alternate_level,
                         cor_status->level_first_mixer,
                         cor_status->level_if_io);

      if (version >= 630) {
        out << "  [";
        for (std::uint_fast8_t k = 0; k < 3; ++k) {
          out << fmt::format(" {:0.2f}dBm", cor_status->level_if_io_arr[k]);
          if (k == cor_status->stared_idx)
            out << "* ";
        }
        out << " ]";
      }

      out << fmt::format("\nFE Temperature       :      {:7.2f} °C\n"
                         "IqIf Temperature     :      {:7.2f} °C\n"
                         "RXTX RF Temperature  :      {:7.2f} °C\n"
                         "RXTX IF Temperature  :      {:7.2f} °C\n"
                         "Hardware Settings    :       Bank1    Bank2\n"
                         "RF-Gain              :      0x{:04X}   0x{:04X}\n"
                         "IF-Coarse-Setting    :      {:X}/{:4}   {:X}/{:4}\n"
                         "IF-Fine-Setting      :        {:4}     {:4}\n",
                         cor_status->fe_temperature,
                         cor_status->iqif_temperature,
                         cor_status->rxtx_rf_temperature,
                         cor_status->rxtx_if_temperature,
                         cor_status->rf_gain_bank_1,
                         cor_status->rf_gain_bank_2,
                         cor_status->if_coarse_setting_bank_1 >> 10,
                         cor_status->if_coarse_setting_bank_1 & 0x3FF,
                         cor_status->if_coarse_setting_bank_2 >> 10,
                         cor_status->if_coarse_setting_bank_2 & 0x3FF,
                         cor_status->if_fine_setting_bank_1,
                         cor_status->if_fine_setting_bank_2);

    }
    else
      LOGGER_ERROR("Correction processor communication failed ({})", error_code_to_string(err));
  }

  struct cor_status_2 {
    uint16_t operation_mode;
    uint16_t measurement_mode[5];
    uint16_t active_channel;
    uint16_t next_active_channel;
    uint16_t edit_channel;
    double frequency;
    uint16_t subsid_supr_mode;
    double subsid_outer_border;
    double subsid_inner_border;
    double subsid_freq_offset;
    double lo1_frequency;
    double lo3_frequency;
    double if1_frequency;
    double if2_frequency;
    double if3_frequency;
    uint16_t subsid_active_chan;
    uint16_t filter_group_id;
    double filter_band_dev_low;
    double filter_band_dev_med;
    double filter_band_dev_high;
    double filter_band_dev_vhigh;
    double filter_dev_if1_high;
    double filter_dev_if1_low;
    double filter_dev_if3_pre;
    double filter_dev_if3_narrow;
    double filter_dev_saw;
    uint16_t field_0xa8[6];
    uint16_t corpo_path;
    uint16_t iqif_path;
    uint16_t if_signal_mode;
    uint16_t application_mode;
  } __attribute__((packed));


  std::string_view filter_group_id_to_string(const decltype(cor_status_2::filter_group_id)& filter_group_id) {
    switch(filter_group_id) {
    case 0:
      return "10M7   NB: WIDE\n";
    case 1:
      return "10M7   NB: 300K\n";
    case 2:
      return "10M7   NB: NOTCH\n";
    case 3:
      return "7M68   NB: WIDE\n";
    case 4:
      return "10M0   NB: WIDE\n";
    case 5:
      return "10M0   NB: 2M00\n";
    case 6:
      return "10M0   NB: 300K\n";
    case 7:
      return "10M7   NB: 2M00\n";
    default:
      return " UNKNOWN!\n";
    }
  }

  std::string_view application_mode_to_string(const decltype(cor_status_2::application_mode) &application_mode,
                                              const rx_tx &rx_tx) {
    if (rx_tx == rx_tx::RX) {
      switch (application_mode) {
      case 0:
        return "IF3 = 10.7 MHz, Pd = 1.385 MHz, IF2 = 486.515MHz, fix";
      case 1:
        return "IF3 = 7.68 MHz, Pd = 1.385 MHz, IF2 = 486.765MHz, fix";
      case 2:
        return "IF3 = 10.0 MHz, Pd = 1.385 MHz, IF2 = 485.830MHz, fix";
      case 3:
        return "IF3 = 10.7 MHz, Pd = 69.25 kHz, IF2 = 487.215MHz, var";
      case 4:
        return "IF3 = 7.68 MHz, Pd = 69.25 MHz, IF2 = 486.765MHz, var";
      case 5:
        return "IF3 = 7.68 MHz, Pd = 69.25 MHz, IF2 = 487.873MHz, var";
      default:
        return "Not defined yet";
      }
    } else {
      switch (application_mode) {
      case 0:
        return "Default";
      case 1:
        return "AMPS";
      case 2:
        return "Bluetooth";
      case 3:
        return "Low rate FM";
      case 4:
        return "IF3 = 15.36 MHz, Pd = 1.385 MHz, IF2 = fix";
      default:
        return "Not defined yet";
      }
    }
  }

  void CorrectionProcessorInterface::get_status_2(const rx_tx rx_tx, std::ostream &out) const {
    const auto version = get_version(out);

    const auto [err, result] = interact<uint16_t, uint16_t>(0x51, std::chrono::seconds(1), {rx_tx}, 0xBC >> 1, out);
    struct cor_status_2 *cor_status_2 = (struct cor_status_2*)result.data();
    if (err == error_code::CommandSuccessful) {
      if (version < 700) {
        out << fmt::format("Operation Mode       :    0x{:04X}\n"
                           "Measurement Mode 0..4:    0x{:04X}   0x{:04X}   0x{:04X}   0x{:04X}   0x{:04X}\n"
                           "Active Channel       :         {:4}\n"
                           "Next Active Channel  :         {:4}\n"
                           "Edit Channel         :         {:4}\n"
                           "Frequency            : {:12.7f} MHz\n",
                           cor_status_2->operation_mode,
                           cor_status_2->measurement_mode[0], cor_status_2->measurement_mode[1],
                           cor_status_2->measurement_mode[2], cor_status_2->measurement_mode[3],
                           cor_status_2->measurement_mode[4],
                           cor_status_2->active_channel,
                           cor_status_2->next_active_channel,
                           cor_status_2->edit_channel,
                           cor_status_2->frequency / 1.0E3);
      }

      out << fmt::format("IF1 frequency        : {:12.7f} MHz\n"
                         "IF2 frequency        : {:12.7f} MHz\n"
                         "IF3 frequency        : {:12.7f} MHz\n"
                         "LO3 frequency        : {:12.7f} MHz\n"
                         "\nFollowing is only valid if subsidiary suppression is active:\n"
                         "LO1 frequency        : {:12.7f} MHz\n"
                         "Subsid. Suppression  :     ",
                         cor_status_2->if1_frequency / 1.0E3,
                         cor_status_2->if2_frequency / 1.0E3,
                         cor_status_2->if3_frequency / 1.0E3,
                         cor_status_2->lo3_frequency / 1.0E3,
                         cor_status_2->lo1_frequency / 1.0E3);

      if (cor_status_2->subsid_outer_border < std::abs(cor_status_2->subsid_freq_offset))
        out << "inactive\n";  //TODO on default startup setting that one is reported wrongly
      else
        out << "  active\n";

      out << fmt::format("Subsid. Suppr. Mode  :         {:4}\n"
                         "Subsid. Active Chan. :         {:4}\n"
                         "Subsid. Freq. Offset : {:12.3f} kHz\n"
                         "Subsid. Inner Bound  : {:12.3f} kHz\n"
                         "Subsid. Outer Bound  : {:12.3f} kHz\n",
                         cor_status_2->subsid_supr_mode,
                         cor_status_2->subsid_active_chan,
                         cor_status_2->subsid_freq_offset,
                         cor_status_2->subsid_inner_border,
                         cor_status_2->subsid_outer_border);

      if (version >= 700) {
        out << fmt::format("Filter Group (ID{:02})  : PRE: ", cor_status_2->filter_group_id);
        out << filter_group_id_to_string(cor_status_2->filter_group_id);
        out << fmt::format("Filter-Band dev.     : Low:     {:+6.2f} dB  Med:       {:+6.2f} dB\n"
                           "                       High:    {:+6.2f} dB  VHigh:     {:+6.2f} dB\n"
                           "Single-Filter dev.   : If1High: {:+6.2f} dB  If1Low:    {:+6.2f} dB\n"
                           "                       IF3Pre:  {:+6.2f} dB  If3Narrow: {:+6.2f} dB\n"
                           "                       SAW:     {:+6.2f} dB\n",
                           cor_status_2->filter_band_dev_low, cor_status_2->filter_band_dev_med,
                           cor_status_2->filter_band_dev_high, cor_status_2->filter_band_dev_vhigh,
                           cor_status_2->filter_dev_if1_high, cor_status_2->filter_dev_if1_low,
                           cor_status_2->filter_dev_if3_pre, cor_status_2->filter_dev_if3_narrow,
                           cor_status_2->filter_dev_saw);
        out << fmt::format("Applic. mode (ID{:02})  : ", cor_status_2->application_mode);
        out << application_mode_to_string(cor_status_2->application_mode, rx_tx) << "\n";
        out << fmt::format("CoPro Path           : {:}\n"
                           "IqIf Path            : {:}\n"
                           "If Signal Mode       : {:}\n",
                           cor_status_2->corpo_path,
                           cor_status_2->iqif_path,
                           cor_status_2->if_signal_mode);
      }

    } else
      LOGGER_ERROR("Correction processor communication failed ({})",
                   error_code_to_string(err));
  }

  struct self_check_result {
    uint8_t id;
    uint8_t action;
    uint8_t idx;
    uint8_t unknown;
    int16_t min_val;
    int16_t max_val;
    int16_t fmdv;
    double meas_val;
  } __attribute__((packed));

  struct diag_type {
    std::string_view name;
    std::string_view unit;
  };

  constexpr std::array<diag_type, 38> diag_translation =
    {{{"DIAG+28V", "V"},
      {"DIAG+12V", "V"},
      {"DIAG-8V", "V"},
      {"DIAG+5V", "V"},
      {"DIAG+3.3V", "V"},
      {"DIAG-5V", "V"},
      {"DIAG-12V", "V"},
      {"DIAG+0.8V", "V"},
      {"VTEMP_BM (TX-RF)", "Deg C"},
      {"5VREF", "V"},
      {"VFILTTX (Notch)", "V"},
      {"THRESHOLD", "V"},
      {"VGAINRX", "V"},
      {"VGAINTX", "V"},
      {"POW", "V"},
      {"POWIF3", "V"},
      {"TXLO1TUNEDIAG", "V"},
      {"TXLO3LEVDIAG", "V"},
      {"TXLO3TUNEDIAG", "V"},
      {"LO2TUNEDIAG", "V"},
      {"LO2LEVDIAG", "V"},
      {"TRIPLDIAG", "V"},
      {"VTEMP_BO (TX-IF)", "Deg C"},
      {"TXLO1LEVDIAG", "V"},
      {"RXLO1TUNEDIAG", "V"},
      {"RXLO3LEVDIAG", "V"},
      {"RXLO3TUNEDIAG", "V"},
      {"LO0LEVDIAG", "V"},
      {"LO0TUNEDIAG", "V"},
      {"VTEMP_AM (RX-RF)", "Deg C"},
      {"RXLO1LEVDIAG", "V"},
      {"VTEMP_AO (RX-IF)", "Deg C"},
      {"VFILTTX (Pass)", "V"},
      {"TXIF3LEV", "V"},
      {"TXIF3REF", "V"},
      {"DIAG+8VTX", "V"},
      {"DIAG+8VRX", "V"},
      {"DIAG+6VT", "V"}}};

  void CorrectionProcessorInterface::self_check(std::ostream &out) const {
    logger::RAIIFlush raii_flush(out);

    const auto [err, result] = interact<uint16_t, uint16_t>(
        0x4C, std::chrono::seconds(10), {}, 0x70C >> 1, out);
    if (err == error_code::CommandSuccessful) {
      uint8_t *count = (uint8_t*)result.data();
      if (*count > 100) {
        LOGGER_ERROR("Count is indicating more diagnosis results than maximally available.");
        return;
      }

      out << "ID Name               Action    FMDV    Min Limit   Max Limit   Meas. Val  \n";
      for (uint_fast8_t idx = 0; idx < *count; ++idx) {
        struct self_check_result *self_check_result = (struct self_check_result*)(result.data() + 2 + (9 * idx));
        const uint8_t translation_idx = self_check_result->idx - 1;
        std::string name = "UNKNOWN NAME";
        double scale = 1.;
        if (translation_idx < diag_translation.size())
          name = diag_translation[translation_idx].name;
        if (diag_translation[translation_idx].unit == "Deg C")
          scale = 100.;
        out << fmt::format("{:2d} {:16s} : {:3d}    {:8.3f}   {:8.3f}    {:8.3f}    {:8.3f}",
                           self_check_result->id, name, self_check_result->action,
                           self_check_result->fmdv * scale / 1000.,
                           self_check_result->min_val * scale / 1000.,
                           self_check_result->max_val * scale / 1000.,
                           self_check_result->meas_val * scale);
        if ((self_check_result->meas_val < self_check_result->min_val / 1000.) ||
            (self_check_result->meas_val > self_check_result->max_val / 1000.))
          out << "  false";
        out << "\n";
      }
    } else
      LOGGER_ERROR("Correction processor communication failed ({})",
                   error_code_to_string(err));
  }

}; /* namespace mmio_interface */

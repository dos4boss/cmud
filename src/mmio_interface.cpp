#include "mmio_interface.hpp"
#include "logger.hpp"

#include <fcntl.h>
#include <gsl/gsl-lite.hpp>
#include <stdexcept>
#include <sys/mman.h>
#include <unistd.h>

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

    const auto [err, result] = interact<uint16_t, uint16_t>(0x0B,
                                                            std::chrono::seconds(1),
                                                            {to_underlying(rx_tx)},
                                                            0x96 >> 1,
                                                            out);
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

    const auto [err, result] = interact<uint16_t, uint16_t>(0x51,
                                                            std::chrono::seconds(1),
                                                            {to_underlying(rx_tx)},
                                                            0xBC >> 1,
                                                            out);
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


  struct set_frequency {
    double frequency;
    uint16_t rx_tx;
    uint16_t frequency_band;
  } __attribute__((packed));

  template <typename StructT, typename VectorT>
  std::vector<VectorT> struct_to_vector(const StructT &struct_) {
    const auto ptr = reinterpret_cast<const VectorT*>(&struct_);
    return std::vector<VectorT>(ptr, ptr + (sizeof(StructT) / sizeof(VectorT)));
  }

  template <typename StructT>
  std::vector<uint8_t> struct_to_vector_8(const StructT &struct_) {
    return struct_to_vector<StructT, uint8_t>(struct_);
  }

  template <typename StructT>
  std::vector<uint16_t> struct_to_vector_16(const StructT &struct_) {
    return struct_to_vector<StructT, uint16_t>(struct_);
  }

  void CorrectionProcessorInterface::set_frequency(const double& frequency,
                                                   const rx_tx& rx_tx,
                                                   const frequency_band& frequency_band,
                                                   std::ostream &out) const {
    logger::RAIIFlush raii_flush(out);

    const struct set_frequency set_frequency_ = {frequency,
                                                 to_underlying(rx_tx),
                                                 to_underlying(frequency_band)};

    const auto [err, result] = interact<uint16_t, uint16_t>(0x40,
                                                            std::chrono::seconds(1),
                                                            struct_to_vector_16(set_frequency_),
                                                            0,
                                                            out);
    if (err != error_code::CommandSuccessful)
      LOGGER_ERROR("Correction processor communication failed ({})",
                   error_code_to_string(err));
  }

  struct switch_id_translation {
    std::string_view name;
    switch_id id;
  };

  struct switch_state_translation {
    std::string_view name;
    switch_state state;
  };

  static const std::array<switch_id_translation, 174> switch_id_translations = {
   {{"RXPATH", switch_id::RXPATH}, // 0x0001
    {"RXBWFLT", switch_id::RXBWFLT}, // 0x0002
    {"RXSELFLT", switch_id::RXSELFLT}, // 0x0003
    {"RXDET1", switch_id::RXDET1}, // 0x0004
    {"RXDET2", switch_id::RXDET2}, // 0x0005
    {"RXDET3", switch_id::RXDET3}, // 0x0006
    {"RXSHRES", switch_id::RXSHRES}, // 0x0007
    {"RXIF3OUT", switch_id::RXIF3OUT}, // 0x0008
    {"RXIF1ATT", switch_id::RXIF1ATT}, // 0x0009
    {"RXSPEED", switch_id::RXSPEED}, // 0x000A
    {"RXFILTER", switch_id::RXFILTER}, // 0x000B
    {"RXDETECTOR", switch_id::RXDETECTOR}, // 0x000C
    {"RXRFATT0", switch_id::RXRFATT0}, // 0x000D
    {"RXRFATT1", switch_id::RXRFATT1}, // 0x000E
    {"RXRFATT2", switch_id::RXRFATT2}, // 0x000F
    {"RXRFATT3", switch_id::RXRFATT3}, // 0x0010
    {"RXRFATT4", switch_id::RXRFATT4}, // 0x0011
    {"RXRFATT5", switch_id::RXRFATT5}, // 0x0012
    {"RXRFATT6", switch_id::RXRFATT6}, // 0x0013
    {"RXRFAMP", switch_id::RXRFAMP}, // 0x0014
    {"RXRFLFGAIN", switch_id::RXRFLFGAIN}, // 0x0015
    {"RXRFSTEPATT", switch_id::RXRFSTEPATT}, // 0x0016
    {"RXIFLCU", switch_id::RXIFLCU}, // 0x0017
    {"RXIFGAIN0", switch_id::RXIFGAIN0}, // 0x0018
    {"RXIFGAIN1", switch_id::RXIFGAIN1}, // 0x0019
    {"RXIFGAIN", switch_id::RXIFGAIN}, // 0x001A
    {"RXFRHIGHIF", switch_id::RXFRHIGHIF}, // 0x001B
    {"RXFRHIGHRF", switch_id::RXFRHIGHRF}, // 0x001C
    {"RXFR2700", switch_id::RXFR2700}, // 0x001D
    {"RXFRLOWRF", switch_id::RXFRLOWRF}, // 0x001E
    {"RXFRLO1EN", switch_id::RXFRLO1EN}, // 0x001F
    {"RXFRVCO1", switch_id::RXFRVCO1}, // 0x0020
    {"RXFRVCO2", switch_id::RXFRVCO2}, // 0x0021
    {"RXFRPDEN", switch_id::RXFRPDEN}, // 0x0022
    {"RXFRTUNE", switch_id::RXFRTUNE}, // 0x0023
    {"RXFROFFSET", switch_id::RXFROFFSET}, // 0x0024
    {"RXFRLO3EN", switch_id::RXFRLO3EN}, // 0x0025
    {"RXFRNCO", switch_id::RXFRNCO}, // 0x0026
    {"RXLO3DATAACQ", switch_id::RXLO3DATAACQ}, // 0x0027
    {"RXLO3MODE1", switch_id::RXLO3MODE1}, // 0x0028
    {"RXLO3MODE2", switch_id::RXLO3MODE2}, // 0x0029
    {"RXLO3PDPOL", switch_id::RXLO3PDPOL}, // 0x002A
    {"RXLO3STANDBY1", switch_id::RXLO3STANDBY1}, // 0x002B
    {"RXLO3STANDBY2", switch_id::RXLO3STANDBY2}, // 0x002C
    {"RXLO3ANTIBL1", switch_id::RXLO3ANTIBL1}, // 0x002D
    {"RXLO3ANTIBL2", switch_id::RXLO3ANTIBL2}, // 0x002E
    {"RXLO3PREAMPSEL", switch_id::RXLO3PREAMPSEL}, // 0x002F
    {"RXLO3SGLDUAL", switch_id::RXLO3SGLDUAL}, // 0x0030
    {"RXLO3PORT1", switch_id::RXLO3PORT1}, // 0x0031
    {"RXLO3PDCUR1", switch_id::RXLO3PDCUR1}, // 0x0032
    {"RXLO3PDCUR2", switch_id::RXLO3PDCUR2}, // 0x0033
    {"RXLO3PDCUR3", switch_id::RXLO3PDCUR3}, // 0x0034
    {"RXLO3MODE", switch_id::RXLO3MODE}, // 0x0035
    {"RXLO3ANTIBL", switch_id::RXLO3ANTIBL}, // 0x0036
    {"RXLO3PDCUR", switch_id::RXLO3PDCUR}, // 0x0037
    {"TXIF2IN", switch_id::TXIF2IN}, // 0x0039
    {"TXCAL", switch_id::TXCAL}, // 0x003A
    {"TXSPEED", switch_id::TXSPEED}, // 0x003B
    {"TXRFATT0", switch_id::TXRFATT0}, // 0x003C
    {"TXRFATT1", switch_id::TXRFATT1}, // 0x003D
    {"TXRFATT2", switch_id::TXRFATT2}, // 0x003E
    {"TXRFATT3", switch_id::TXRFATT3}, // 0x003F
    {"TXRFATT4", switch_id::TXRFATT4}, // 0x0040
    {"TXRFATT5", switch_id::TXRFATT5}, // 0x0041
    {"TXRFATT6", switch_id::TXRFATT6}, // 0x0042
    {"TXRFAMP", switch_id::TXRFAMP}, // 0x0043
    {"TXFEATT", switch_id::TXFEATT}, // 0x0044
    {"TXRFSTEPATT", switch_id::TXRFSTEPATT}, // 0x0045
    {"TXIFLCU", switch_id::TXIFLCU}, // 0x0046
    {"TXFRHIGHIF", switch_id::TXFRHIGHIF}, // 0x0047
    {"TXFRHIGHRF", switch_id::TXFRHIGHRF}, // 0x0048
    {"TXFR2700", switch_id::TXFR2700}, // 0x0049
    {"TXFRLOWRF", switch_id::TXFRLOWRF}, // 0x004A
    {"TXFRLO1EN", switch_id::TXFRLO1EN}, // 0x004B
    {"TXFRVCO1", switch_id::TXFRVCO1}, // 0x004C
    {"TXFRVCO2", switch_id::TXFRVCO2}, // 0x004D
    {"TXFRPDEN", switch_id::TXFRPDEN}, // 0x004E
    {"TXFRTUNE", switch_id::TXFRTUNE}, // 0x004F
    {"TXFROFFSET", switch_id::TXFROFFSET}, // 0x0050
    {"TXFRLO3EN", switch_id::TXFRLO3EN}, // 0x0051
    {"TXFRNCO", switch_id::TXFRNCO}, // 0x0052
    {"TXLO3DATAACQ", switch_id::TXLO3DATAACQ}, // 0x0053
    {"TXLO3MODE1", switch_id::TXLO3MODE1}, // 0x0054
    {"TXLO3MODE2", switch_id::TXLO3MODE2}, // 0x0055
    {"TXLO3PDPOL", switch_id::TXLO3PDPOL}, // 0x0056
    {"TXLO3STANDBY1", switch_id::TXLO3STANDBY1}, // 0x0057
    {"TXLO3STANDBY2", switch_id::TXLO3STANDBY2}, // 0x0058
    {"TXLO3ANTIBL1", switch_id::TXLO3ANTIBL1}, // 0x0059
    {"TXLO3ANTIBL2", switch_id::TXLO3ANTIBL2}, // 0x005A
    {"TXLO3PREAMPSEL", switch_id::TXLO3PREAMPSEL}, // 0x005B
    {"TXLO3SGLDUAL", switch_id::TXLO3SGLDUAL}, // 0x005C
    {"TXLO3PORT1", switch_id::TXLO3PORT1}, // 0x005D
    {"TXLO3PDCUR1", switch_id::TXLO3PDCUR1}, // 0x005E
    {"TXLO3PDCUR2", switch_id::TXLO3PDCUR2}, // 0x005F
    {"TXLO3PDCUR3", switch_id::TXLO3PDCUR3}, // 0x0060
    {"TXLO3MODE", switch_id::TXLO3MODE}, // 0x0061
    {"TXLO3ANTIBL", switch_id::TXLO3ANTIBL}, // 0x0062
    {"TXLO3PDCUR", switch_id::TXLO3PDCUR}, // 0x0063
    {"BITTRIPL", switch_id::BITTRIPL}, // 0x0065
    {"BITLO2EN", switch_id::BITLO2EN}, // 0x0066
    {"BITREFDIV", switch_id::BITREFDIV}, // 0x0067
    {"LO0EN", switch_id::LO0EN}, // 0x0068
    {"RXSTEPEN", switch_id::RXSTEPEN}, // 0x006F
    {"TXSTEPEN", switch_id::TXSTEPEN}, // 0x0070
    {"RXPOWEN", switch_id::RXPOWEN}, // 0x0071
    {"MODETXPOW", switch_id::MODETXPOW}, // 0x0072
    {"RXACKEN", switch_id::RXACKEN}, // 0x0073
    {"MODETXACK", switch_id::MODETXACK}, // 0x0074
    {"LEDON", switch_id::LEDON}, // 0x0075
    {"ADCDIAGMUX0", switch_id::ADCDIAGMUX0}, // 0x0078
    {"ADCDIAGMUX1", switch_id::ADCDIAGMUX1}, // 0x0079
    {"ADCDIAGMUX2", switch_id::ADCDIAGMUX2}, // 0x007A
    {"ADCDIAGADR", switch_id::ADCDIAGADR}, // 0x007B
    {"ADCSGL", switch_id::ADCSGL}, // 0x007C
    {"ADCUNI", switch_id::ADCUNI}, // 0x007D
    {"ADCPD", switch_id::ADCPD}, // 0x007E
    {"ADCBUSY", switch_id::ADCBUSY}, // 0x007F
    {"SERRXVALID", switch_id::SERRXVALID}, // 0x0080
    {"SERRXOVFLW", switch_id::SERRXOVFLW}, // 0x0081
    {"SERRXIRQEN", switch_id::SERRXIRQEN}, // 0x0082
    {"SERRXCLEAR", switch_id::SERRXCLEAR}, // 0x0083
    {"SERTXBUSY", switch_id::SERTXBUSY}, // 0x0084
    {"SERTXLENGTH", switch_id::SERTXLENGTH}, // 0x0085
    {"COMLO3RXEN", switch_id::COMLO3RXEN}, // 0x0088
    {"COMLO3TXEN", switch_id::COMLO3TXEN}, // 0x0089
    {"COMLO3DATA", switch_id::COMLO3DATA}, // 0x008A
    {"COMLO3CLOCK", switch_id::COMLO3CLOCK}, // 0x008B
    {"RXLO1JEN", switch_id::RXLO1JEN}, // 0x008E
    {"RXLO1UPDOWN", switch_id::RXLO1UPDOWN}, // 0x008F
    {"RXLO1LOCK", switch_id::RXLO1LOCK}, // 0x0090
    {"RXFEAMP", switch_id::RXFEAMP}, // 0x0091
    {"RXBUSY", switch_id::RXBUSY}, // 0x0095
    {"TXBUSY", switch_id::TXBUSY}, // 0x0096
    {"RXACTEN", switch_id::RXACTEN}, // 0x0097
    {"TXACTEN", switch_id::TXACTEN}, // 0x0098
    {"RXPHASE", switch_id::RXPHASE}, // 0x0099
    {"TXPHASE", switch_id::TXPHASE}, // 0x009A
    {"FMEN", switch_id::FMEN}, // 0x009B
    {"FMATT1", switch_id::FMATT1}, // 0x009C
    {"FMATT2", switch_id::FMATT2}, // 0x009D
    {"RXIF3PREFILT", switch_id::RXIF3PREFILT}, // 0x009E
    {"TXPATH", switch_id::TXPATH}, // 0x009F
    {"RXLO0EN", switch_id::RXLO0EN}, // 0x00A0
    {"TXLO0EN", switch_id::TXLO0EN}, // 0x00A1
    {"RXSETEDGE", switch_id::RXSETEDGE}, // 0x00A2
    {"TXSETEDGE", switch_id::TXSETEDGE}, // 0x00A3
    {"ADCDOUT", switch_id::ADCDOUT}, // 0x00A4
    {"ADCCLK", switch_id::ADCCLK}, // 0x00A5
    {"ADCDIN", switch_id::ADCDIN}, // 0x00A6
    {"ADCSTRB", switch_id::ADCSTRB}, // 0x00A7
    {"RXIF2OUT", switch_id::RXIF2OUT}, // 0x00A8
    {"RXLO1EXT", switch_id::RXLO1EXT}, // 0x00A9
    {"SWITCHLEV", switch_id::SWITCHLEV}, // 0x00AA
    {"RXSTATIFGAIN0", switch_id::RXSTATIFGAIN0}, // 0x00AB
    {"RXSTATIFGAIN1", switch_id::RXSTATIFGAIN1}, // 0x00AC
    {"RXSTATIFGAIN", switch_id::RXSTATIFGAIN}, // 0x00AD
    {"RXRELOAD", switch_id::RXRELOAD}, // 0x00AE
    {"TXRELOAD", switch_id::TXRELOAD}, // 0x00AF
    {"RXPRELOWBANDPASS", switch_id::RXPRELOWBANDPASS}, // 0x00B1
    {"RXPREBANDPASS", switch_id::RXPREBANDPASS}, // 0x00B2
    {"SELDET2AB", switch_id::SELDET2AB}, // 0x00B3
    {"RXNARROWFLT", switch_id::RXNARROWFLT}, // 0x00B4
    {"RXIF3FILT_MODULE", switch_id::RXIF3FILT_MODULE}, // 0x00B5
    {"RXLOWRFA", switch_id::RXLOWRFA}, // 0x00BA
    {"RXLOWRFB", switch_id::RXLOWRFB}, // 0x00BB
    {"RXCALPATH", switch_id::RXCALPATH}, // 0x00BC
    {"RXSELSH1_3", switch_id::RXSELSH1_3}, // 0x00BD
    {"RXSELSH13_2", switch_id::RXSELSH13_2}, // 0x00BE
    {"RXSELPOWINT1_3", switch_id::RXSELPOWINT1_3}, // 0x00C0
    {"RXSELPOWINT13_2", switch_id::RXSELPOWINT13_2}, // 0x00C1
    {"ADCDIAGMUX3", switch_id::ADCDIAGMUX3}, // 0x00C3
    {"ACKSETTIME", switch_id::ACKSETTIME}, // 0x00DA
    {"ACKBUSYTIME", switch_id::ACKBUSYTIME} // 0x00DB
   }};

  static const std::array<switch_state_translation, 98> switch_state_translations = {
   {{"ADP28V", switch_state::ADP28V}, // 0x0001
    {"ADP12V", switch_state::ADP12V}, // 0x0002
    {"ADM8V", switch_state::ADM8V}, // 0x0003
    {"ADP5V", switch_state::ADP5V}, // 0x0004
    {"ADP3V3", switch_state::ADP3V3}, // 0x0005
    {"ADM5V", switch_state::ADM5V}, // 0x0006
    {"ADM12V", switch_state::ADM12V}, // 0x0007
    {"ADP2V", switch_state::ADP2V}, // 0x0008
    {"ADVTEMPBM", switch_state::ADVTEMPBM}, // 0x0009
    {"ADREF5V", switch_state::ADREF5V}, // 0x000A
    {"ADVFILTTX", switch_state::ADVFILTTX}, // 0x000B
    {"ADTHRESHOLD", switch_state::ADTHRESHOLD}, // 0x000C
    {"ADVGAINRX", switch_state::ADVGAINRX}, // 0x000D
    {"ADVGAINTX", switch_state::ADVGAINTX}, // 0x000E
    {"ADIF3POWPEAK", switch_state::ADIF3POWPEAK}, // 0x000F
    {"ADIF3POWCUR", switch_state::ADIF3POWCUR}, // 0x0010
    {"ADTXLO1TUNE", switch_state::ADTXLO1TUNE}, // 0x0011
    {"ADTXLO3LEV", switch_state::ADTXLO3LEV}, // 0x0012
    {"ADTXLO3TUNE", switch_state::ADTXLO3TUNE}, // 0x0013
    {"ADLO2TUNE", switch_state::ADLO2TUNE}, // 0x0014
    {"ADLO2LEV", switch_state::ADLO2LEV}, // 0x0015
    {"ADTRIPL", switch_state::ADTRIPL}, // 0x0016
    {"ADVTEMPBO", switch_state::ADVTEMPBO}, // 0x0017
    {"ADTXLO1LEV", switch_state::ADTXLO1LEV}, // 0x0018
    {"ADRXLO1TUNE", switch_state::ADRXLO1TUNE}, // 0x0019
    {"ADRXLO3LEV", switch_state::ADRXLO3LEV}, // 0x001A
    {"ADRXLO3TUNE", switch_state::ADRXLO3TUNE}, // 0x001B
    {"ADLO0LEV", switch_state::ADLO0LEV}, // 0x001C
    {"ADLO0TUNE", switch_state::ADLO0TUNE}, // 0x001D
    {"ADVTEMPAM", switch_state::ADVTEMPAM}, // 0x001E
    {"ADRXLO1LEV", switch_state::ADRXLO1LEV}, // 0x001F
    {"ADVTEMPAO", switch_state::ADVTEMPAO}, // 0x0020
    {"OFF", switch_state::OFF}, // 0x0041
    {"ON", switch_state::ON}, // 0x0042
    {"LOW", switch_state::LOW}, // 0x0043
    {"HIGH", switch_state::HIGH}, // 0x0044
    {"NARROW", switch_state::NARROW}, // 0x0045
    {"WIDE", switch_state::WIDE}, // 0x0046
    {"DISABLED", switch_state::DISABLED}, // 0x0047
    {"ENABLED", switch_state::ENABLED}, // 0x0048
    {"HOLD", switch_state::HOLD}, // 0x0049
    {"RESET", switch_state::RESET}, // 0x004A
    {"INT", switch_state::INT}, // 0x004B
    {"EXT", switch_state::EXT}, // 0x004C
    {"ASYNC", switch_state::ASYNC}, // 0x004D
    {"SYNC", switch_state::SYNC}, // 0x004E
    {"NEGATIVE", switch_state::NEGATIVE}, // 0x004F
    {"POSITIVE", switch_state::POSITIVE}, // 0x0050
    {"STANDBY", switch_state::STANDBY}, // 0x0051
    {"ACTIVE", switch_state::ACTIVE}, // 0x0052
    {"SINGLE", switch_state::SINGLE}, // 0x0053
    {"DUAL", switch_state::DUAL}, // 0x0054
    {"TEST", switch_state::TEST}, // 0x0055
    {"ICPM", switch_state::ICPM}, // 0x0058
    {"PW1_3", switch_state::PW1_3}, // 0x0059
    {"PW5", switch_state::PW5}, // 0x005A
    {"PW10", switch_state::PW10}, // 0x005B
    {"PW13", switch_state::PW13}, // 0x005C
    {"CUR0_175", switch_state::CUR0_175}, // 0x005D
    {"CUR0_7", switch_state::CUR0_7}, // 0x005E
    {"CUR0_35", switch_state::CUR0_35}, // 0x005F
    {"CUR1_4", switch_state::CUR1_4}, // 0x0060
    {"CUR0_25", switch_state::CUR0_25}, // 0x0061
    {"CUR1", switch_state::CUR1}, // 0x0062
    {"CUR0_5", switch_state::CUR0_5}, // 0x0063
    {"CUR2", switch_state::CUR2}, // 0x0064
    {"FILTER1", switch_state::FILTER1}, // 0x0065
    {"FILTER2", switch_state::FILTER2}, // 0x0066
    {"NCOREG1", switch_state::NCOREG1}, // 0x0067
    {"NCOREG2", switch_state::NCOREG2}, // 0x0068
    {"IFGAIN0", switch_state::IFGAIN0}, // 0x0069
    {"IFGAIN1", switch_state::IFGAIN1}, // 0x006A
    {"IFGAIN2", switch_state::IFGAIN2}, // 0x006B
    {"IFGAIN3", switch_state::IFGAIN3}, // 0x006C
    {"READY", switch_state::READY}, // 0x006D
    {"BUSY", switch_state::BUSY}, // 0x006E
    {"POWERDN", switch_state::POWERDN}, // 0x006F
    {"INTCLK", switch_state::INTCLK}, // 0x0071
    {"EXTCLK", switch_state::EXTCLK}, // 0x0072
    {"CLEAR", switch_state::CLEAR}, // 0x0073
    {"UNIPOLAR", switch_state::UNIPOLAR}, // 0x0074
    {"BIPOLAR", switch_state::BIPOLAR}, // 0x0075
    {"DOWN", switch_state::DOWN}, // 0x0076
    {"UP", switch_state::UP}, // 0x0077
    {"UNLOCKED", switch_state::UNLOCKED}, // 0x0078
    {"LOCKED", switch_state::LOCKED}, // 0x0079
    {"PREFILT10M7", switch_state::PREFILT10M7}, // 0x007A
    {"PREFILT10M0", switch_state::PREFILT10M0}, // 0x007B
    {"PREFILT7M68", switch_state::PREFILT7M68}, // 0x007C
    {"PREFILTBAND", switch_state::PREFILTBAND}, // 0x007D
    {"NBFILT300K", switch_state::NBFILT300K}, // 0x007E
    {"NBFILT2M0", switch_state::NBFILT2M0}, // 0x007F
    {"DET2A", switch_state::DET2A}, // 0x0080
    {"DET2B", switch_state::DET2B}, // 0x0081
    {"PATH1", switch_state::PATH1}, // 0x0082
    {"PATH3", switch_state::PATH3}, // 0x0083
    {"PATH13", switch_state::PATH13}, // 0x0084
    {"PATH2", switch_state::PATH2}, // 0x0085
   }};

  std::optional<std::string_view> switch_id_to_string(const switch_id &switch_id) {
    auto result = std::find_if(switch_id_translations.begin(),
                               switch_id_translations.end(),
                               [&switch_id](const switch_id_translation& trans) {
                                 return trans.id == switch_id;
                               });
    if (result != switch_id_translations.end())
      return result->name;
    else
      return std::nullopt;
  }

  std::optional<switch_id> string_to_switch_id(const std::string& str) {
    auto result = std::find_if(switch_id_translations.begin(),
                               switch_id_translations.end(),
                               [&str](const switch_id_translation &trans) {
                                 return trans.name == str;
                               });
    if (result != switch_id_translations.end())
      return result->id;
    else
      return std::nullopt;
  }

  std::optional<std::string_view> switch_state_to_string(const switch_state &switch_state) {
    auto result = std::find_if(switch_state_translations.begin(),
                               switch_state_translations.end(),
                               [&switch_state](const switch_state_translation &trans) {
                                 return trans.state == switch_state;
                               });
    if (result != switch_state_translations.end())
      return result->name;
    else
      return std::nullopt;
  }

  std::optional<switch_state> string_to_switch_state(const std::string &str) {
    auto result = std::find_if(switch_state_translations.begin(),
                               switch_state_translations.end(),
                               [&str](const switch_state_translation &trans) {
                                 return trans.name == str;
                               });
    if (result != switch_state_translations.end())
      return result->state;
    else
      return std::nullopt;
  }

  std::optional<switch_id> safe_switch_id_from_string(const std::string &switch_id_str) {
    const auto switch_id_opt = string_to_switch_id(switch_id_str);
    if (!switch_id_opt.has_value()) {
      try {
        using sw_id_underlying = std::underlying_type_t<switch_id>;
        return static_cast<switch_id>(gsl::narrow<sw_id_underlying>(std::stoul(switch_id_str, 0, 0)));
      } catch (std::exception &e) {
        LOGGER_ERROR("Failed to convert '{:}' to a useable switch_id ({:}).",
                     switch_id_str, e.what());
        return std::nullopt;
      }
    } else
      return switch_id_opt.value();
  }

  std::optional<switch_state> safe_switch_state_from_string(const std::string &switch_state_str) {
    const auto switch_state_opt = string_to_switch_state(switch_state_str);
    if (!switch_state_opt.has_value()) {
      try {
        using sw_state_underlying = std::underlying_type_t<switch_state>;
        return static_cast<switch_state>(gsl::narrow<sw_state_underlying>(std::stoul(switch_state_str, 0, 0)));
      } catch (std::exception &e) {
        LOGGER_ERROR(
            "Failed to convert '{:}' to a writeable switch_state ({:}).",
            switch_state_str, e.what());
        return std::nullopt;
      }
    }
    else
      return switch_state_opt.value();
  }

  std::optional<switch_state> CorrectionProcessorInterface::read_switch_state(const switch_id &switch_id,
                                                                              std::ostream &out) const {
    logger::RAIIFlush raii_flush(out);

    const auto [err, result] = interact<uint16_t, uint16_t>(0x17,
                                                            std::chrono::seconds(1),
                                                            {to_underlying(switch_id)},
                                                            2 >> 1,
                                                            out);
    if (err == error_code::CommandSuccessful) {
      return static_cast<switch_state>(result[0]);
    } else {
      LOGGER_ERROR("Correction processor communication failed ({})",
                   error_code_to_string(err));
      return std::nullopt;
    }
  }

  void CorrectionProcessorInterface::read_switch_state(const std::string &switch_id_str,
                                                       std::ostream &out) const {
    logger::RAIIFlush raii_flush(out);
    const auto switch_id_opt = safe_switch_id_from_string(switch_id_str);
    if (!switch_id_opt.has_value())
      return;

    if (auto switch_state_opt = read_switch_state(switch_id_opt.value(), out)) {
      const auto switch_id_str_opt = switch_id_to_string(switch_id_opt.value());
      const auto switch_state_str_opt = switch_state_to_string(switch_state_opt.value());

      out << fmt::format("switch {}: {}\n",
                         switch_id_str_opt.value_or("unknown"),
                         switch_state_str_opt.value_or(std::to_string(to_underlying(switch_state_opt.value()))));
    }
  }

  std::optional<std::underlying_type_t<switch_state>>
  CorrectionProcessorInterface::read_switch_value(const switch_id &switch_id,
                                                  std::ostream &out) const {
    logger::RAIIFlush raii_flush(out);

    const auto [err, result] = interact<uint16_t, uint16_t>(0x19,
                                                            std::chrono::seconds(1),
                                                            {to_underlying(switch_id)},
                                                            2 >> 1,
                                                            out);
    if (err == error_code::CommandSuccessful) {
      return result[0];
    } else {
      LOGGER_ERROR("Correction processor communication failed ({})",
                   error_code_to_string(err));
      return std::nullopt;
    }
  }

  void CorrectionProcessorInterface::read_switch_value(const std::string& switch_id_str,
                                                       std::ostream &out) const {
    logger::RAIIFlush raii_flush(out);
    const auto switch_id_opt = safe_switch_id_from_string(switch_id_str);
    if (!switch_id_opt.has_value())
      return;

    if (auto switch_value_opt = read_switch_value(switch_id_opt.value(), out)) {
      const auto switch_id_str_opt = switch_id_to_string(switch_id_opt.value());

      out << fmt::format("switch {}: 0x{:04X} ({})\n",
                         switch_id_str_opt.value_or("unknown"),
                         switch_value_opt.value(),
                         switch_value_opt.value());
    }
  }

  struct write_switch_state {
    switch_id switch_id;
    switch_state switch_state;
  } __attribute__((packed));

  void CorrectionProcessorInterface::write_switch_state(const switch_id &switch_id,
                                                        const switch_state &switch_state,
                                                        std::ostream &out) const {
    logger::RAIIFlush raii_flush(out);
    struct write_switch_state write_switch_state_ = {switch_id, switch_state};
    const auto [err, result] = interact<uint16_t, uint16_t>(0x16,
                                                            std::chrono::seconds(1),
                                                            {struct_to_vector_16(write_switch_state_)},
                                                            0,
                                                            out);
    if (err != error_code::CommandSuccessful)
      LOGGER_ERROR("Correction processor communication failed ({})",
                   error_code_to_string(err));
  }

  void CorrectionProcessorInterface::write_switch_state(const std::string &switch_id_str,
                                                        const std::string &switch_state_str,
                                                        std::ostream &out) const {
    logger::RAIIFlush raii_flush(out);
    const auto switch_id_opt = safe_switch_id_from_string(switch_id_str);
    if (!switch_id_opt.has_value())
      return;

    const auto switch_state_opt = safe_switch_state_from_string(switch_state_str);
    if (!switch_state_opt.has_value())
      return;

    write_switch_state(switch_id_opt.value(), switch_state_opt.value(), out);
  }

  void CorrectionProcessorInterface::write_switch_value(const switch_id &switch_id,
                                                        const switch_state &switch_state,
                                                        std::ostream &out) const {
    logger::RAIIFlush raii_flush(out);
    struct write_switch_state write_switch_state_ = {switch_id, switch_state};
    const auto [err, result] = interact<uint16_t, uint16_t>(0x18,
                                                            std::chrono::seconds(1),
                                                            {struct_to_vector_16(write_switch_state_)},
                                                            0,
                                                            out);
    if (err != error_code::CommandSuccessful)
      LOGGER_ERROR("Correction processor communication failed ({})",
                   error_code_to_string(err));
  }

  void CorrectionProcessorInterface::write_switch_value(const std::string &switch_id_str,
                                                        const std::string &switch_state_str,
                                                        std::ostream &out) const {
    logger::RAIIFlush raii_flush(out);
    const auto switch_id_opt = safe_switch_id_from_string(switch_id_str);
    if (!switch_id_opt.has_value())
      return;

    const auto switch_state_opt =
        safe_switch_state_from_string(switch_state_str);
    if (!switch_state_opt.has_value())
      return;

    write_switch_value(switch_id_opt.value(), switch_state_opt.value(), out);
  }

  }; /* namespace mmio_interface */

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
          out << fmt::format("User correction RX:   RF4IN:  {:b}, RF2IN:  {:b}, RF1IN:  {:b}\n"
                             "User correction TX:   RF3OUT: {:b}, RF2OUT: {:b}, RF1OUT: {:b}\n",
                             cor_board_info->user_correction_rx & 4, cor_board_info->user_correction_rx & 2, cor_board_info->user_correction_rx & 1,
                             cor_board_info->user_correction_tx & 4, cor_board_info->user_correction_tx & 2, cor_board_info->user_correction_tx & 1);
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

}; /* namespace mmio_interface */

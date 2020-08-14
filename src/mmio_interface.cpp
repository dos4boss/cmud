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
    case error_code::CommunicationError: return "Communication error";
    case error_code::ProcessorBusy: return "Processor busy";
    case error_code::ProcessorDoesNotRespond: return "Process does not respond";
    case error_code::TimeoutError: return "Timeout error";
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
            err = error_code(read<uint16_t>(2, 2, out)[0]);
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

  void CorrectionProcessorInterface::get_version(std::ostream &out) const {
    const auto [err, result] = interact<uint8_t, uint16_t>(0xc, std::chrono::seconds(1), {}, 0x30 >> 1, out);
    if (err == error_code::CommandSuccessful) {
      char second_val = '.';
      out << fmt::format("Version {}{}{}", result[0] >> 8, second_val, result[0] & 0xFF);
    }
    else
      LOGGER_ERROR("Correction processor communication failed ({})", error_code_to_string(err));
  }

}; /* namespace mmio_interface */

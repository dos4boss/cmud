#include "logger.hpp"
#include "io_port_interface.hpp"
#include "gsl/gsl-lite.hpp"

#include <sys/io.h>

namespace io_port_interface {
  MAKE_LOCAL_LOGGER("io_port");

  void register_access(const decltype(hw_interface::stream_info::address) &address,
                       const decltype(hw_interface::stream_info::length) &length) {
    logger::RAIIFlush raii_flush(std::cout);

    static std::unordered_map<decltype(hw_interface::stream_info::address),
                              decltype(hw_interface::stream_info::length)> ioport_map;
    const auto search = ioport_map.find(address);
    if (search != ioport_map.end() && search->second == length)
      return;
    const auto return_value = ioperm(address, length, 1);
    ioport_map[address] = length;
    if (return_value != 0) {
      if (errno == EINVAL) {
        LOGGER_ERROR("Failed to get access to I/O Ports 0x{:04X} - 0x{:04X} ({})",
                     address, address + length, std::strerror(errno));
      } else if (errno == ENOMEM) {
        LOGGER_CRITICAL("Out of memory during I/O Ports 0x{:04X} - 0x{:04X} access request",
                        address, address + length);
        std::cout << logger::LoggerSink::get();
        exit(EXIT_FAILURE);
      } else if (errno == EPERM) {
        LOGGER_CRITICAL("Permission denied on access request. Make sure to run as root.");
        std::cout << logger::LoggerSink::get();
        exit(EXIT_FAILURE);
      } else {
        LOGGER_CRITICAL("Unknown failure during I/O Port access request ({})",
                        std::strerror(errno));
        std::cout << logger::LoggerSink::get();
        exit(EXIT_FAILURE);
      }
    }
  }

  void write(const uint32_t address, const uint32_t value,
             const uint8_t length, hw_interface::access_width access_width,
             std::ostream &out) {
    logger::RAIIFlush raii_flush(out);

    switch (access_width) {
    case hw_interface::access_width::BYTE: {
      if (length != 1) {
        LOGGER_ERROR("Byte access is usually only done for 1 byte long streams.");
        return;
      }
      const auto sized_value = gsl::narrow<uint8_t>(value);
      LOGGER_INFO("Writing byte: 0x{:02X} @ 0x{:04X}", sized_value, address);
      outb(sized_value, address);
      break;
    }

    case hw_interface::access_width::WORD: {
      if (length != 2) {
        LOGGER_ERROR("Word access is usually only done for 2 byte long streams.");
        return;
      }
      const auto sized_value = gsl::narrow<uint16_t>(value);
      LOGGER_INFO("Writing byte: 0x{:04X} @ 0x{:04X}", sized_value, address);
      outw(sized_value, address);
      break;
    }

    case hw_interface::access_width::DWORD: {
      if (length != 4) {
        LOGGER_ERROR("Dword access is usually only done for 4 byte long streams.");
        return;
      }
      const auto sized_value = gsl::narrow<uint32_t>(value);
      LOGGER_INFO("Writing byte: 0x{:08X} @ 0x{:04X}", sized_value, address);
      outl(sized_value, address);
      break;
    }

    default:
      LOGGER_ERROR("Access mode currently not supported.");
    }
  }

  uint32_t read(const uint32_t address, uint32_t &value,
                const uint8_t length, hw_interface::access_width access_width,
                std::ostream &out) {
    logger::RAIIFlush raii_flush(out);

    switch (access_width) {
    case hw_interface::access_width::BYTE: {
      if (length != 1) {
        LOGGER_ERROR("Byte access is usually only done for 1 byte long streams.");
        return EXIT_FAILURE;
      }
      const auto read_value = inb(address);
      LOGGER_INFO("Read byte: 0x{:02X} @ 0x{:04X}", read_value, address);
      value = read_value;
      break;
    }

    case hw_interface::access_width::WORD: {
      if (length != 2) {
        LOGGER_ERROR("Word access is usually only done for 2 byte long streams.");
        return EXIT_FAILURE;
      }
      const auto read_value = inw(address);
      LOGGER_INFO("Read word: 0x{:04X} @ 0x{:04X}", read_value, address);
      value = read_value;
      break;
    }

    case hw_interface::access_width::DWORD: {
      if (length != 4) {
        LOGGER_ERROR("Dword access is usually only done for 4 byte long streams.");
        return EXIT_FAILURE;
      }
      const auto read_value = inl(address);
      LOGGER_INFO("Read word: 0x{:08X} @ 0x{:04X}", read_value, address);
      value = read_value;
      break;
    }

    default:
      LOGGER_ERROR("Access mode currently not supported.");
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }

} // namespace io_port_interface

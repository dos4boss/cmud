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

  std::vector<uint8_t> MMIOInterface::read(const unsigned &offset,
                                           const unsigned &length,
                                           std::ostream &out) {
    logger::RAIIFlush raii_flush(out);
    if (offset >= _isa_size)
      throw std::runtime_error("Offset too large.");
    std::vector<uint8_t> result(length);
    for(unsigned idx = 0; idx < length; ++idx) {
      result[idx] = *(_isa_base_ptr + offset + idx);
      LOGGER_INFO("Read byte: 0x{:02X} @ 0x{:08X}", result[idx], _isa_base_address + offset + idx);
    }
    return result;
  }

  void MMIOInterface::write(const unsigned &offset,
                            const std::vector<uint8_t> data,
                            std::ostream &out) {
    logger::RAIIFlush raii_flush(out);
    if (offset >= _isa_size)
      throw std::runtime_error("Offset too large.");
    unsigned idx = 0;
    for(const auto &datum : data)
      *(_isa_base_ptr + offset + idx++) = datum;
  }

  bool CorrectionProcessorInterface::is_present(std::ostream &out) {
    std::vector<uint8_t> check_data = {0x55, 0xaa};
    auto restore_data = read(0x7fa, 2, out);
    write(0x7fa, check_data, out);
    auto readback_data = read(0x7fa, 2, out);
    write(0x7fa, restore_data, out);
    return check_data == readback_data;
  }

}; /* namespace mmio_interface */

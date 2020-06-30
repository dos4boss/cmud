#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

#include <stdexcept>

#include "correction_processor_interface.hpp"

namespace correction_processor_interface {

  CorrectionProcessorInterface::CorrectionProcessorInterface(const unsigned &isa_base_address_) {
    _isa_base_address = isa_base_address_;

    _memfd = open("/dev/mem", O_RDWR | O_SYNC);
    if (_memfd < 0)
      throw std::runtime_error("Failed to open '/dev/mem'. Must be run as root.");
    _isa_base_ptr = (uint8_t*)mmap(0, _isa_size, PROT_READ | PROT_WRITE, MAP_SHARED, _memfd, _isa_base_address);
    if (_isa_base_ptr == MAP_FAILED)
      throw std::runtime_error("Memory mapping failed.");
  }

  CorrectionProcessorInterface::~CorrectionProcessorInterface(void) {
    close(_memfd);
  }

  std::vector<uint8_t> CorrectionProcessorInterface::read(const unsigned &offset, const unsigned &length) {
    if (offset >= _isa_size)
      throw std::runtime_error("Offset too large.");
    std::vector<uint8_t> result(length);
    for(unsigned idx = 0; idx < length; ++idx)
      result[idx] = *(_isa_base_ptr + offset + idx);
    return result;
  }

  void CorrectionProcessorInterface::write(const unsigned &offset, const std::vector<uint8_t> data) {
    if (offset >= _isa_size)
      throw std::runtime_error("Offset too large.");
    unsigned idx = 0;
    for(const auto &datum : data)
      *(_isa_base_ptr + offset + idx++) = datum;
  }

  bool CorrectionProcessorInterface::is_present(void) {
    std::vector<uint8_t> check_data = {0x55, 0xaa};
    auto restore_data = read(0x7fa, 2);
    write(0x7fa, check_data);
    auto readback_data = read(0x7fa, 2);
    write(0x7fa, restore_data);
    return check_data == readback_data;
  }

}; /* namespace correction_processor_interface */

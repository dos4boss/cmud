#pragma once

#include <vector>

namespace correction_processor_interface {

  class CorrectionProcessorInterface {
  public:
    CorrectionProcessorInterface(const unsigned &isa_base_address_);
    ~CorrectionProcessorInterface(void);

    bool is_present(void);
    std::vector<uint8_t> read(const unsigned &offset, const unsigned &legnth);
    void write(const unsigned &offset, const std::vector<uint8_t> data);

  private:
    unsigned _isa_base_address;
    int _memfd;
    uint8_t *_isa_base_ptr;
    unsigned _isa_size = 0x1000;
  };

}; /*  namespace correction_processor_interface */

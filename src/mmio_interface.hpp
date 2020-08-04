#pragma once

#include <vector>
#include <inttypes.h>

#include <stdexcept>

namespace mmio_interface {

  class MMIOInterface {
  public:
    MMIOInterface(const unsigned &isa_base_address_);
    ~MMIOInterface(void);

    std::vector<uint8_t> read(const unsigned &offset,
                              const unsigned &legnth,
                              std::ostream &out) const;
    void write(const unsigned &offset,
               const std::vector<uint8_t> data,
               std::ostream &out) const;

  private:
    unsigned _isa_base_address;
    int _memfd;
    uint8_t *_isa_base_ptr;
    unsigned _isa_size = 0x1000;
  };

  class CorrectionProcessorInterface : MMIOInterface {
  public:
    CorrectionProcessorInterface(const unsigned rxtx_board_idx)
      : MMIOInterface{(rxtx_board_idx == 0) ? MMIOInterface{0xD4000}
                      : ((rxtx_board_idx == 1) ? MMIOInterface{0xD5000}
                         : throw std::runtime_error("Wrong board idx (muste be 0 or 1)."))} {}
    bool is_present(std::ostream &out);
  };

}; /*  namespace mmio_interface */

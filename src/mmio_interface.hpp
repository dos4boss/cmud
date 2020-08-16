#pragma once

#include <vector>
#include <inttypes.h>
#include <stdexcept>
#include <chrono>

namespace mmio_interface {

  class MMIOInterface {
  public:
    MMIOInterface(const unsigned &isa_base_address_);
    ~MMIOInterface(void);

  protected:
    template <typename T>
    std::vector<T> read(const unsigned &offset,
                        const unsigned &number_elements,
                        std::ostream &out) const;

    template <typename T>
    void write(const unsigned &offset,
               const std::vector<T> data,
               std::ostream &out) const;

  private:
    unsigned _isa_base_address;
    int _memfd;
    uint8_t *_isa_base_ptr;
    unsigned _isa_size = 0x1000;
  };


  enum error_code : int32_t {
                             CommandSuccessful = 0,
                             TimeoutError = -27,
                             ProcessorBusy = -28,
                             ProcessorDoesNotRespond = -29,
                             CommunicationError = -30,
  };

  std::string error_code_to_string(const error_code &err);


  class CorrectionProcessorInterface : MMIOInterface {
  public:
    CorrectionProcessorInterface(const unsigned rxtx_board_idx)
      : MMIOInterface{(rxtx_board_idx == 0) ? MMIOInterface{0xD4000}
                      : ((rxtx_board_idx == 1) ? MMIOInterface{0xD5000}
                         : throw std::runtime_error("Wrong board idx (muste be 0 or 1)."))} {}

    bool is_present(std::ostream &out) const;
    uint_fast16_t get_version(std::ostream &out) const;
    void get_board_info(std::ostream &out) const;

    std::pair<error_code, std::vector<uint16_t>>
    interact(const uint_fast8_t &status, const std::chrono::microseconds &timeout,
             const std::vector<uint16_t> &data_in, const uint_fast16_t &number_elements_out,
             std::ostream &out) const;


  protected:
    error_code wait_for_status_or_timeout(const uint_fast8_t &status,
                                          const std::chrono::microseconds &timeout,
                                          std::ostream &out) const;

    template <typename T_in, typename T_out>
    std::pair<error_code, std::vector<T_out>>
    interact(const uint_fast8_t &status, const std::chrono::microseconds &timeout,
             const std::vector<T_in> &data_in, const uint_fast16_t &number_elements_out,
             std::ostream &out) const;
  };

}; /*  namespace mmio_interface */

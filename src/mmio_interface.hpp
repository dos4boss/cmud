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
                             GeneralError = -1,
                             TableNotFound = -2,
                             OutOfMemory = -3,
                             ChecksumError = -4,
                             DataExceedsMemoryLimits = -5,
                             PermissionDenied = -6,
                             TooManyOpenTables = -7,
                             EPROMBufferOverflow = -8,
                             TableBlockNotValid = -9,
                             IndexOutOfRange = -10,
                             UnknownCommand = -11,
                             TableIdNotValid = -12,
                             NoSettingTable = -13,
                             NoDeviationTable = -14,
                             HardwareNotSupported = -15,
                             InvalidSwitchSetting = -16,
                             InvalidSignalDirection = -17,
                             InvalidArgument = -18,
                             InvalidFrequencyBand = -19,
                             IllegalMemorySpecifier = -20,
                             FlashError = -21,
                             I2CError = -22,
                             DeviceError = -23,
                             TooManyFrequencies = -24,
                             WrongEndianess = -25,
                             UnknownDatatype = -26,
                             TimeoutError = -27,
                             ProcessorBusy = -28,
                             ProcessorDoesNotRespond = -29,
                             CommunicationError = -30,
                             UnknownParameterId = -31,
                             InvalidParameterSize = -32,
                             InvalidMessageStructure = -33,
                             ParameterTooLarge = -34,
                             CheckrepairFailed = -35,
                             LevelOverflow = -36,
                             OptionalTableNotFound = -37,
                             BitPatternTestFailed = -38,
                             AddressTestFailed = -39
  };

  std::string error_code_to_string(const error_code &err);

  enum rx_tx : uint16_t {
                         RX = 0,
                         TX = 1
  };


  class CorrectionProcessorInterface : MMIOInterface {
  public:
    CorrectionProcessorInterface(const unsigned rxtx_board_idx)
      : MMIOInterface{(rxtx_board_idx == 0) ? MMIOInterface{0xD4000}
                      : ((rxtx_board_idx == 1) ? MMIOInterface{0xD5000}
                         : throw std::runtime_error("Wrong board idx (muste be 0 or 1)."))} {}

    bool is_present(std::ostream &out) const;
    uint_fast16_t get_version(std::ostream &out) const;
    void get_board_info(std::ostream &out) const;
    void get_status(const rx_tx rx_tx, std::ostream &out) const;
    void get_status_2(const rx_tx rx_tx, std::ostream &out) const;
    void self_check(std::ostream &out) const;

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

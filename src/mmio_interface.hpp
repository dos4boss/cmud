#pragma once

#include <vector>
#include <array>
#include <inttypes.h>
#include <stdexcept>
#include <chrono>
#include <optional>

template <typename E>
constexpr auto to_underlying(E e) -> typename std::underlying_type<E>::type {
  return static_cast<typename std::underlying_type<E>::type>(e);
}

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


  enum class rx_tx : uint16_t { RX = 0, TX = 1 };
  enum class frequency_band : uint16_t { LOW = 0,
                                         MEDIUM1 = 1,
                                         MEDIUM2 = 2,
                                         HIGH1 = 3,
                                         HIGH2 = 4,
                                         VHIGH1 = 5,
                                         VHIGH2 = 6};

  // forward declared here for better readability; definition is at the of this file
  enum class switch_id : uint16_t;
  enum class switch_state : uint16_t;
  enum class error_code : int32_t;

  std::string error_code_to_string(const error_code &err);

  //std::string_view switch_id_to_string(const switch_id &switch_id);

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
    void set_frequency(const double &frequency,
                       const rx_tx &rx_tx,
                       const frequency_band &frequency_band,
                       std::ostream &out) const;

    void read_switch_state(const std::string &switch_id_str,
                           std::ostream &out) const;
    std::optional<switch_state> read_switch_state(const switch_id &switch_id,
                                                  std::ostream &out) const;
    void write_switch_state(const switch_id &switch_id,
                            const switch_state &switch_state,
                            std::ostream &out) const;
    void write_switch_state(const std::string &switch_id_str,
                            const std::string &switch_state_str,
                            std::ostream &out) const;

    std::optional<std::underlying_type_t<switch_state>>
    read_switch_value(const switch_id &switch_id, std::ostream &out) const;
    void read_switch_value(const std::string &switch_id_str,
                           std::ostream &out) const;
    void write_switch_value(const switch_id &switch_id,
                            const switch_state &switch_state,
                            std::ostream &out) const;
    void write_switch_value(const std::string &switch_id_str,
                            const std::string &switch_state_str,
                            std::ostream &out) const;

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

  enum class error_code : int32_t {
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

  enum class switch_id : uint16_t {
    RXPATH             = 0x0001,
    RXBWFLT            = 0x0002,
    RXSELFLT           = 0x0003,
    RXDET1             = 0x0004,
    RXDET2             = 0x0005,
    RXDET3             = 0x0006,
    RXSHRES            = 0x0007,
    RXIF3OUT           = 0x0008,
    RXIF1ATT           = 0x0009,
    RXSPEED            = 0x000A,
    RXFILTER           = 0x000B,
    RXDETECTOR         = 0x000C,
    RXRFATT0           = 0x000D,
    RXRFATT1           = 0x000E,
    RXRFATT2           = 0x000F,
    RXRFATT3           = 0x0010,
    RXRFATT4           = 0x0011,
    RXRFATT5           = 0x0012,
    RXRFATT6           = 0x0013,
    RXRFAMP            = 0x0014,
    RXRFLFGAIN         = 0x0015,
    RXRFSTEPATT        = 0x0016,
    RXIFLCU            = 0x0017,
    RXIFGAIN0          = 0x0018,
    RXIFGAIN1          = 0x0019,
    RXIFGAIN           = 0x001A,
    RXFRHIGHIF         = 0x001B,
    RXFRHIGHRF         = 0x001C,
    RXFR2700           = 0x001D,
    RXFRLOWRF          = 0x001E,
    RXFRLO1EN          = 0x001F,
    RXFRVCO1           = 0x0020,
    RXFRVCO2           = 0x0021,
    RXFRPDEN           = 0x0022,
    RXFRTUNE           = 0x0023,
    RXFROFFSET         = 0x0024,
    RXFRLO3EN          = 0x0025,
    RXFRNCO            = 0x0026,
    RXLO3DATAACQ       = 0x0027,
    RXLO3MODE1         = 0x0028,
    RXLO3MODE2         = 0x0029,
    RXLO3PDPOL         = 0x002A,
    RXLO3STANDBY1      = 0x002B,
    RXLO3STANDBY2      = 0x002C,
    RXLO3ANTIBL1       = 0x002D,
    RXLO3ANTIBL2       = 0x002E,
    RXLO3PREAMPSEL     = 0x002F,
    RXLO3SGLDUAL       = 0x0030,
    RXLO3PORT1         = 0x0031,
    RXLO3PDCUR1        = 0x0032,
    RXLO3PDCUR2        = 0x0033,
    RXLO3PDCUR3        = 0x0034,
    RXLO3MODE          = 0x0035,
    RXLO3ANTIBL        = 0x0036,
    RXLO3PDCUR         = 0x0037,
    TXIF2IN            = 0x0039,
    TXCAL              = 0x003A,
    TXSPEED            = 0x003B,
    TXRFATT0           = 0x003C,
    TXRFATT1           = 0x003D,
    TXRFATT2           = 0x003E,
    TXRFATT3           = 0x003F,
    TXRFATT4           = 0x0040,
    TXRFATT5           = 0x0041,
    TXRFATT6           = 0x0042,
    TXRFAMP            = 0x0043,
    TXFEATT            = 0x0044,
    TXRFSTEPATT        = 0x0045,
    TXIFLCU            = 0x0046,
    TXFRHIGHIF         = 0x0047,
    TXFRHIGHRF         = 0x0048,
    TXFR2700           = 0x0049,
    TXFRLOWRF          = 0x004A,
    TXFRLO1EN          = 0x004B,
    TXFRVCO1           = 0x004C,
    TXFRVCO2           = 0x004D,
    TXFRPDEN           = 0x004E,
    TXFRTUNE           = 0x004F,
    TXFROFFSET         = 0x0050,
    TXFRLO3EN          = 0x0051,
    TXFRNCO            = 0x0052,
    TXLO3DATAACQ       = 0x0053,
    TXLO3MODE1         = 0x0054,
    TXLO3MODE2         = 0x0055,
    TXLO3PDPOL         = 0x0056,
    TXLO3STANDBY1      = 0x0057,
    TXLO3STANDBY2      = 0x0058,
    TXLO3ANTIBL1       = 0x0059,
    TXLO3ANTIBL2       = 0x005A,
    TXLO3PREAMPSEL     = 0x005B,
    TXLO3SGLDUAL       = 0x005C,
    TXLO3PORT1         = 0x005D,
    TXLO3PDCUR1        = 0x005E,
    TXLO3PDCUR2        = 0x005F,
    TXLO3PDCUR3        = 0x0060,
    TXLO3MODE          = 0x0061,
    TXLO3ANTIBL        = 0x0062,
    TXLO3PDCUR         = 0x0063,
    BITTRIPL           = 0x0065,
    BITLO2EN           = 0x0066,
    BITREFDIV          = 0x0067,
    LO0EN              = 0x0068,
    RXSTEPEN           = 0x006F,
    TXSTEPEN           = 0x0070,
    RXPOWEN            = 0x0071,
    MODETXPOW          = 0x0072,
    RXACKEN            = 0x0073,
    MODETXACK          = 0x0074,
    LEDON              = 0x0075,
    ADCDIAGMUX0        = 0x0078,
    ADCDIAGMUX1        = 0x0079,
    ADCDIAGMUX2        = 0x007A,
    ADCDIAGADR         = 0x007B,
    ADCSGL             = 0x007C,
    ADCUNI             = 0x007D,
    ADCPD              = 0x007E,
    ADCBUSY            = 0x007F,
    SERRXVALID         = 0x0080,
    SERRXOVFLW         = 0x0081,
    SERRXIRQEN         = 0x0082,
    SERRXCLEAR         = 0x0083,
    SERTXBUSY          = 0x0084,
    SERTXLENGTH        = 0x0085,
    COMLO3RXEN         = 0x0088,
    COMLO3TXEN         = 0x0089,
    COMLO3DATA         = 0x008A,
    COMLO3CLOCK        = 0x008B,
    RXLO1JEN           = 0x008E,
    RXLO1UPDOWN        = 0x008F,
    RXLO1LOCK          = 0x0090,
    RXFEAMP            = 0x0091,
    RXBUSY             = 0x0095,
    TXBUSY             = 0x0096,
    RXACTEN            = 0x0097,
    TXACTEN            = 0x0098,
    RXPHASE            = 0x0099,
    TXPHASE            = 0x009A,
    FMEN               = 0x009B,
    FMATT1             = 0x009C,
    FMATT2             = 0x009D,
    RXIF3PREFILT       = 0x009E,
    TXPATH             = 0x009F,
    RXLO0EN            = 0x00A0,
    TXLO0EN            = 0x00A1,
    RXSETEDGE          = 0x00A2,
    TXSETEDGE          = 0x00A3,
    ADCDOUT            = 0x00A4,
    ADCCLK             = 0x00A5,
    ADCDIN             = 0x00A6,
    ADCSTRB            = 0x00A7,
    RXIF2OUT           = 0x00A8,
    RXLO1EXT           = 0x00A9,
    SWITCHLEV          = 0x00AA,
    RXSTATIFGAIN0      = 0x00AB,
    RXSTATIFGAIN1      = 0x00AC,
    RXSTATIFGAIN       = 0x00AD,
    RXRELOAD           = 0x00AE,
    TXRELOAD           = 0x00AF,
    RXPRELOWBANDPASS   = 0x00B1,
    RXPREBANDPASS      = 0x00B2,
    SELDET2AB          = 0x00B3,
    RXNARROWFLT        = 0x00B4,
    RXIF3FILT_MODULE   = 0x00B5,
    RXLOWRFA           = 0x00BA,
    RXLOWRFB           = 0x00BB,
    RXCALPATH          = 0x00BC,
    RXSELSH1_3         = 0x00BD,
    RXSELSH13_2        = 0x00BE,
    RXSELPOWINT1_3     = 0x00C0,
    RXSELPOWINT13_2    = 0x00C1,
    ADCDIAGMUX3        = 0x00C3,
    ACKSETTIME         = 0x00DA,
    ACKBUSYTIME        = 0x00DB
  };

  enum class switch_state : uint16_t {
    ADP28V       = 0x0001,
    ADP12V       = 0x0002,
    ADM8V        = 0x0003,
    ADP5V        = 0x0004,
    ADP3V3       = 0x0005,
    ADM5V        = 0x0006,
    ADM12V       = 0x0007,
    ADP2V        = 0x0008,
    ADVTEMPBM    = 0x0009,
    ADREF5V      = 0x000A,
    ADVFILTTX    = 0x000B,
    ADTHRESHOLD  = 0x000C,
    ADVGAINRX    = 0x000D,
    ADVGAINTX    = 0x000E,
    ADIF3POWPEAK = 0x000F,
    ADIF3POWCUR  = 0x0010,
    ADTXLO1TUNE  = 0x0011,
    ADTXLO3LEV   = 0x0012,
    ADTXLO3TUNE  = 0x0013,
    ADLO2TUNE    = 0x0014,
    ADLO2LEV     = 0x0015,
    ADTRIPL      = 0x0016,
    ADVTEMPBO    = 0x0017,
    ADTXLO1LEV   = 0x0018,
    ADRXLO1TUNE  = 0x0019,
    ADRXLO3LEV   = 0x001A,
    ADRXLO3TUNE  = 0x001B,
    ADLO0LEV     = 0x001C,
    ADLO0TUNE    = 0x001D,
    ADVTEMPAM    = 0x001E,
    ADRXLO1LEV   = 0x001F,
    ADVTEMPAO    = 0x0020,
    OFF          = 0x0041,
    ON           = 0x0042,
    LOW          = 0x0043,
    HIGH         = 0x0044,
    NARROW       = 0x0045,
    WIDE         = 0x0046,
    DISABLED     = 0x0047,
    ENABLED      = 0x0048,
    HOLD         = 0x0049,
    RESET        = 0x004A,
    INT          = 0x004B,
    EXT          = 0x004C,
    ASYNC        = 0x004D,
    SYNC         = 0x004E,
    NEGATIVE     = 0x004F,
    POSITIVE     = 0x0050,
    STANDBY      = 0x0051,
    ACTIVE       = 0x0052,
    SINGLE       = 0x0053,
    DUAL         = 0x0054,
    TEST         = 0x0055,
    ICPM         = 0x0058,
    PW1_3        = 0x0059,
    PW5          = 0x005A,
    PW10         = 0x005B,
    PW13         = 0x005C,
    CUR0_175     = 0x005D,
    CUR0_7       = 0x005E,
    CUR0_35      = 0x005F,
    CUR1_4       = 0x0060,
    CUR0_25      = 0x0061,
    CUR1         = 0x0062,
    CUR0_5       = 0x0063,
    CUR2         = 0x0064,
    FILTER1      = 0x0065,
    FILTER2      = 0x0066,
    NCOREG1      = 0x0067,
    NCOREG2      = 0x0068,
    IFGAIN0      = 0x0069,
    IFGAIN1      = 0x006A,
    IFGAIN2      = 0x006B,
    IFGAIN3      = 0x006C,
    READY        = 0x006D,
    BUSY         = 0x006E,
    POWERDN      = 0x006F,
    INTCLK       = 0x0071,
    EXTCLK       = 0x0072,
    CLEAR        = 0x0073,
    UNIPOLAR     = 0x0074,
    BIPOLAR      = 0x0075,
    DOWN         = 0x0076,
    UP           = 0x0077,
    UNLOCKED     = 0x0078,
    LOCKED       = 0x0079,
    PREFILT10M7  = 0x007A,
    PREFILT10M0  = 0x007B,
    PREFILT7M68  = 0x007C,
    PREFILTBAND  = 0x007D,
    NBFILT300K   = 0x007E,
    NBFILT2M0    = 0x007F,
    DET2A        = 0x0080,
    DET2B        = 0x0081,
    PATH1        = 0x0082,
    PATH3        = 0x0083,
    PATH13       = 0x0084,
    PATH2        = 0x0085
  };

  }; /*  namespace mmio_interface */


#include "i2c_interface.hpp"

namespace i2c_interface {

void I2CInterface::delay(void) {
  std::this_thread::sleep_for(delay_ns_);
}

} // namespace i2c_interface

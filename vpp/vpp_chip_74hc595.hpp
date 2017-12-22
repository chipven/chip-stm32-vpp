#ifndef __vpp_chip_c74hc595_hpp_
#define __vpp_chip_c74hc595_hpp_

#include "../make/stm32f103c8t6.h"

namespace vpp {
class chip_74hc595 {
private:
  u32 *gpio_serial_input;
  u32 *gpio_reset_clock;
  u32 *gpio_shift_clock;

public:
  chip_74hc595(u32 *serial_input_bit_banding,
               u32 *reset_clock_bit_binding,
               u32 *shift_clock_bit_banding) {
    gpio_serial_input = serial_input_bit_banding;
    gpio_reset_clock = reset_clock_bit_binding;
    gpio_shift_clock = shift_clock_bit_banding;
  }
  void
  send_bytes(u32 bytes_amount, u32 data_to_send) {
    for (u32 i = 0; i < bytes_amount * 8; i++) {
      *gpio_serial_input = data_to_send >> (bytes_amount * 8 - 1);
      data_to_send <<= 1;
      *gpio_shift_clock = 0;
      *gpio_shift_clock = 1;
    }
    *gpio_reset_clock = 1;
    *gpio_reset_clock = 0;
  }
};

} // namespace vpp

#endif

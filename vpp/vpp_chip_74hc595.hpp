#ifndef __vpp_chip_c74hc595_hpp_
#define __vpp_chip_c74hc595_hpp_

#include "../make/stm32f103c8t6.h"

namespace vpp {
class chip_74hc595 {
private:
  unsigned int *gpio_serial_input;
  unsigned int *gpio_reset_clock;
  unsigned int *gpio_shift_clock;

public:
  chip_74hc595(unsigned int *serial_input_bit_banding,
               unsigned int *reset_clock_bit_binding,
               unsigned int *shift_clock_bit_banding) {
    gpio_serial_input = serial_input_bit_banding;
    gpio_reset_clock = reset_clock_bit_binding;
    gpio_shift_clock = shift_clock_bit_banding;
  }
  void
  send_bytes(unsigned int bytes_amount, unsigned int data_to_send) {
    for (unsigned int i = 0; i < bytes_amount * 8; i++) {
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

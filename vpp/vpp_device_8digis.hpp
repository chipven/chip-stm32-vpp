#ifndef __vpp_device_8digis_hpp_
#define __vpp_device_8digis_hpp_

#include "../make/stm32f103c8t6.h"
#include "vpp_chip_74hc595.hpp"

namespace vpp {
class device_8digis : chip_74hc595 {
public:
  device_8digis(u32 *serial_input_bit_banding,
                u32 *reset_clock_bit_binding,
                u32 *shift_clock_bit_banding,
                u32 the_type_digital,
                u32 the_number_system)
      : chip_74hc595((u32 *)serial_input_bit_banding,
                     (u32 *)reset_clock_bit_binding,
                     (u32 *)shift_clock_bit_banding) {
    type_digital = the_type_digital;
    number_system = the_number_system;
  }
  void
  operator=(u32 number_to_show) {
    show_number_trice(number_to_show);
  }

  void
  show_number_trice(u32 number_to_show) {
    u32 data;
    for (u32 i = 0; i < 8; i++) {
      if (type_digital == 0) {
        data = (7 - i) << 8 |
               type_table[type_digital]
                         [(number_to_show / divisor(i) % number_system)];
      }
      if (type_digital == 1) {
        data =
            (1 << i) | type_table[type_digital]
                                 [(number_to_show / divisor(i) % number_system)]
                           << 8;
      }
	  send_bytes(2, data);
	  
    }
  }

private:
  u32 number_system;
  u32 type_digital;
  u32 type_table[2][16] = {{0xfc, 0x60, 0xda, 0xf2, 0x66, 0xb6, 0xbe, 0xe0,
                            0xfe, 0xf6, 0xee, 0x3e, 0x1a, 0x7a, 0x9e, 0x8e},
                           {0xc0, 0xf9, 0xa4, 0xb0, 0x99, 0x92, 0x82, 0xf8,
                            0x80, 0x90, 0x88, 0x83, 0xa7, 0xa1, 0x86, 0x8e}};
  u32
  divisor(u32 bit_position) {
    u32 po[] = {
        1,
        number_system,
        number_system * number_system,
        number_system * number_system * number_system,
        number_system * number_system * number_system * number_system,
        number_system * number_system * number_system * number_system *
            number_system,
        number_system * number_system * number_system * number_system *
            number_system * number_system,
        number_system * number_system * number_system * number_system *
            number_system * number_system * number_system,
    };
    return po[bit_position];
  }
};
} // namespace vpp

#endif /*  __vpp_device_8digis_hpp_ */

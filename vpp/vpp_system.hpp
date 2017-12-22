#ifndef __vpp_system_hpp_
#define __vpp_system_hpp_

#include "../make/stm32f103c8t6.h"

namespace vpp {
class system {
public:
  static void
  on_72m() {
    FLASH->ACR |= 0x32;
    RCC->CFGR |= 0x001d0402;
    RCC->CR |= 0x01010000;
  }

  static void
  on_pp_gpio_ab() {
    RCC->APB2ENR |= 0x0000000c;
    GPIOA->CRL = 0x33333333;
    GPIOA->CRH = 0x33333333;
    GPIOB->CRL = 0x33333333;
    GPIOB->CRH = 0x33333333;
  }

  static void
  delay() {
    if (RCC->CFGR) {
      delay_us(480 + 80);
    } else {
      delay_us(60 + 10);
    }
  }

private:
  static void
  delay_us(u32 time) {
    u32 i = 0;
    while (time--) {
      i = 12; // calibration
      while (i--)
        ;
    }
  }

  static void
  delay_ms(u32 time) {
    u32 i = 0;
    while (time--) {
      i = 12000; // calibration
      while (i--)
        ;
    }
  }
};

} // namespace vpp

#endif // __vpp_system_hpp_

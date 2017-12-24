#include "vpp_chip_74hc595.hpp"
#include "vpp_device_8digis.hpp"
#include "vpp_protocal_i2c.hpp"
#include "vpp_system.hpp"

namespace vpp {
class example {
public:
  static void
  system_example() {
    vpp::system::on_72m();
    vpp::system::on_pp_gpio_ab();
  }
  static void
  device_8digis_example() {
    /* Enable GPIO B */
    RCC->APB2ENR |= 0x00000008;
    GPIOB->CRH &= 0xe000eeee;
    GPIOB->CRH |= 0x03330000;
    vpp::device_8digis d8((unsigned int *)(0x42000000 + 0x10c0c * 0x20 + 12 * 4),
                          (unsigned int *)(0x42000000 + 0x10c0c * 0x20 + 13 * 4),
                          (unsigned int *)(0x42000000 + 0x10c0c * 0x20 + 14 * 4), 1, 16);
    while (1)
      d8.show_number_trice(8);
  }
};

} // namespace vpp

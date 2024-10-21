#include "mmio.h"

#define ADDER_STATUS 0x4000
#define ADDER_X 0x4004
#define ADDER_Y 0x4008
#define ADDER_ADDER 0x400C

unsigned int ADDER_ref(unsigned int x, unsigned int y) {
  return x+y;
}

// DOC include start: ADDER test
int main(void)
{
  uint32_t result, ref, x = 520, y = 15;

  // wait for peripheral to be ready
  while ((reg_read8(ADDER_STATUS) & 0x2) == 0) ;

  reg_write32(ADDER_X, x);
  reg_write32(ADDER_Y, y);


  // wait for peripheral to complete
  while ((reg_read8(ADDER_STATUS) & 0x1) == 0) ;

  result = reg_read32(ADDER_ADDER);
  ref = ADDER_ref(x, y);

  if (result != ref) {
    printf("Hardware result %d does not match reference value %d\n", result, ref);
    return 1;
  }
  printf("Hardware result %d is correct for ADDER\n", result);
  return 0;
}
// DOC include end: ADDER test

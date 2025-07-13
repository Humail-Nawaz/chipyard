// simple_peripheral_test.c

#include <stdint.h>  // <-- Required for uint32_t

#define PERIPHERAL_ADDR   ((volatile uint32_t*)0x00001000)

int main(void) {
    // Step 1: Write to the peripheral
    *PERIPHERAL_ADDR = 0xABCD1234;

    // Step 2: Read from the peripheral
    uint32_t read_value = *PERIPHERAL_ADDR;

    // Simple loop if value matches (optional debug trap)
    if (read_value == 0xABCD1234) {
        while (1); // Infinite loop to observe in simulation
    }

    return 0;
}


#include "mmio.h"

#define FACTORIAL_STATUS 0x4000   // Status register address
#define FACTORIAL_INPUT 0x4004     // Input register for the number to compute factorial
#define FACTORIAL_RESULT 0x4008    // Output register for the computed factorial

// Reference implementation of factorial
unsigned int factorial_ref(unsigned int n) {
    if (n == 0) return 1;
    unsigned int result = 1;
    for (unsigned int i = 1; i <= n; i++) {
        result *= i;
    }
    return result;
}

// DOC include start: Factorial test
int main(void) {
    uint32_t result, ref, n = 5;  // Change this value for different inputs

    // Wait for peripheral to be ready
    while ((reg_read8(FACTORIAL_STATUS) & 0x2) == 0);

    // Write input to the peripheral
    reg_write32(FACTORIAL_INPUT, n);

    // Wait for peripheral to complete calculation
    while ((reg_read8(FACTORIAL_STATUS) & 0x1) == 0);

    // Read result from the peripheral
    result = reg_read32(FACTORIAL_RESULT);
    ref = factorial_ref(n); // Get the reference value

    // Compare hardware result with reference value
    if (result != ref) {
        printf("Hardware result %d does not match reference value %d\n", result, ref);
        return 1;
    }
    printf("Hardware result %d is correct for factorial of %d\n", result, n);
    return 0;
}
// DOC include end: Factorial test

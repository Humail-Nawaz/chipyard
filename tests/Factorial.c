#include <stdio.h>
#include <stdint.h>

#define FACTORIAL_BASE_ADDR 0x4000 // Base address of the factorial accelerator
#define FACTORIAL_BUSY_ADDR (FACTORIAL_BASE_ADDR + 0x00) // Busy status register
#define FACTORIAL_INPUT_ADDR (FACTORIAL_BASE_ADDR + 0x04) // Input register for x
#define FACTORIAL_OUTPUT_ADDR (FACTORIAL_BASE_ADDR + 0x08) // Output register for factorial result

// Function to check if the accelerator is busy
int is_busy() {
    return *((volatile uint32_t*)FACTORIAL_BUSY_ADDR);
}

// Function to set the input value for the factorial calculation
void set_input(uint32_t x) {
    *((volatile uint32_t*)FACTORIAL_INPUT_ADDR) = x;
}

// Function to get the factorial result
uint32_t get_result() {
    return *((volatile uint32_t*)FACTORIAL_OUTPUT_ADDR);
}

int main() {
    uint32_t x;
    printf("Enter a number to calculate its factorial: ");
    scanf("%u", &x);

    // Set the input value
    set_input(x);

    // Wait until the accelerator finishes the calculation
    printf("Calculating...\n");
    while (is_busy()) {
        // Busy wait
    }

    // Get the result
    uint32_t result = get_result();
    printf("Factorial of %u is %u\n", x, result);

    return 0;
}

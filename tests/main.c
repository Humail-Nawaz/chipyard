#define PERIPHERAL_ADDR ((volatile unsigned int*) 0x80000000)

int main() {
    *PERIPHERAL_ADDR = 0x12345678;
    while (1);  // Stay here
    return 0;
}

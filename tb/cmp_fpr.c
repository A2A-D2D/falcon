#include <stdio.h>
#include <stdint.h>
static uint64_t u16_to_f64(uint16_t x) {
    if (x == 0) return 0;
    int pos = 0;
    for (int i = 0; i < 16; i++) {
        if (x & (1 << (15 - i))) { pos = 15 - i; break; }
    }
    uint64_t x64 = (uint64_t)x;
    uint64_t exp = 1023ULL + (uint64_t)pos;
    uint64_t frac = (x64 << (63 - pos)) >> 11;
    return (exp << 52) | frac;
}
int main() {
    uint16_t test[] = {8129, 5792, 5364, 0, 1, 42, 255, 1024, 4096, 8192, 12289};
    for (int i = 0; i < 11; i++) {
        uint64_t v = u16_to_f64(test[i]);
        double dv = *(double*)&v;
        printf("u16=%5u  f64=0x%016llx  val=%.17g\n", test[i], (unsigned long long)v, dv);
    }
    return 0;
}

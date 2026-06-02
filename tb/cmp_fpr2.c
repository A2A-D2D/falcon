#include <stdio.h>
#include <stdint.h>
static uint64_t u16_to_f64_debug(uint16_t x) {
    if (x == 0) return 0;
    int pos = 0;
    for (int i = 0; i < 16; i++) {
        int bit = 15 - i;
        uint32_t mask = 1u << bit;
        if (x & mask) { pos = bit; break; }
    }
    uint64_t x64 = (uint64_t)x;
    uint64_t exp = 1023ULL + (uint64_t)pos;
    uint64_t frac = (x64 << (63 - pos)) >> 11;
    fprintf(stderr, "x=%u pos=%d exp=%llu (0x%llx) frac_top4=0x%llx\n",
            x, pos, (unsigned long long)exp, (unsigned long long)exp,
            (unsigned long long)(frac >> 48));
    return (exp << 52) | frac;
}
int main() {
    uint16_t t[] = {42, 255, 1024, 8129, 8192};
    for (int i = 0; i < 5; i++) {
        uint64_t v = u16_to_f64_debug(t[i]);
        double dv = *(double*)&v;
        printf("u16=%5u  f64=0x%016llx  val=%.17g\n", t[i], (unsigned long long)v, dv);
    }
    return 0;
}

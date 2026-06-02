// cmp_fft.c — Compare hardware vs C FFT on the same input
//
// Reads hm_nonce40.hex (packed uint16), converts each int16 → FP64 using
// the same logic as the Verilog u16_to_f64, runs official Falcon Zf(FFT),
// and dumps the result in RTL hex format.
//
// Build: gcc -o cmp_fft.exe cmp_fft.c -I../DOC/Falcon/official/... -lm
//        (link against falcon inner.c, fft.c, common.c)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

// ─── FP64 conversion matching Verilog u16_to_f64 ───
static uint64_t u16_to_f64(uint16_t x) {
    if (x == 0) return 0ULL;
    // Find MSB position (0..15)
    int pos = 0;
    for (int i = 0; i < 16; i++) {
        if (x & (1 << (15 - i))) { pos = 15 - i; break; }
    }
    uint64_t x64 = (uint64_t)x;
    uint64_t exp = 1023ULL + (uint64_t)pos;
    uint64_t frac = ((x64 << (63 - pos)) >> 11) & 0x000FFFFFFFFFFFFFULL;
    return (exp << 52) | frac;
}

// ─── Read packed uint16 hex file (16 values per 256-bit word) ───
static int read_u16_hex(const char *fn, uint16_t *dst, int n) {
    FILE *f = fopen(fn, "r");
    if (!f) { fprintf(stderr, "Cannot open %s\n", fn); return -1; }
    char line[128];
    for (int w = 0; w < (n + 15) / 16; w++) {
        if (!fgets(line, sizeof(line), f)) break;
        uint64_t hi, lo;
        // Format: 64 hex chars = 256 bits
        // bits[255:128]=hi64, bits[127:0]=lo64
        // Packed: [v15|v14|...|v0] in 256 bits
        // Each vi is 16 bits, stored little-endian within the word
        char hi_str[17] = {0}, lo_str[17] = {0};
        if (strlen(line) >= 64) {
            memcpy(hi_str, line, 16);
            memcpy(lo_str, line + 16, 16);
            // Actually the hex is 64 chars for full 256 bits
            // Read as 64-char hex
            char full[65] = {0};
            memcpy(full, line, 64);
            // Parse as 16 x uint16 little-endian
            for (int j = 0; j < 16 && (w * 16 + j) < n; j++) {
                char byte_pair[5] = {0};
                // Each uint16 is 4 hex chars, in the 64-char string
                // Little-endian within the 256-bit word: word[k] is at offset 4*k
                int off = (15 - j) * 4; // MSB uint16 first in hex string
                memcpy(byte_pair, line + off, 4);
                unsigned int val;
                sscanf(byte_pair, "%04x", &val);
                dst[w * 16 + j] = (uint16_t)val;
            }
        }
    }
    fclose(f);
    return 0;
}

// ─── Falcon-style half-complex FFT (size N, logn=9) ───
// This is a minimal reimplementation matching Zf(FFT) from fft.c
// We pre-compute the twiddle factors from the Falcon GM table.

// Load GM table from hex files
static int load_gm(const char *re_fn, const char *im_fn,
                   double *gm_re, double *gm_im, int sz) {
    FILE *fre = fopen(re_fn, "r");
    FILE *fim = fopen(im_fn, "r");
    if (!fre || !fim) { fprintf(stderr, "Cannot open GM files\n"); return -1; }
    for (int i = 0; i < sz; i++) {
        uint64_t r, im;
        if (fscanf(fre, "%llx", (unsigned long long*)&r) != 1) break;
        if (fscanf(fim, "%llx", (unsigned long long*)&im) != 1) break;
        gm_re[i] = *(double*)&r;
        gm_im[i] = *(double*)&im;
    }
    fclose(fre); fclose(fim);
    return 0;
}

// Direct translation of Zf(FFT) from fft.c
static void falcon_fft(double *f, int logn, double *gm_re, double *gm_im) {
    int n = 1 << logn;   // 512
    int hn = n >> 1;     // 256
    int t = hn;
    for (int u = 1, m = 2; u < logn; u++, m <<= 1) {
        int ht = t >> 1;
        int hm = m >> 1;
        for (int i1 = 0, j1 = 0; i1 < hm; i1++, j1 += t) {
            int j2 = j1 + ht;
            double s_re = gm_re[m + i1];
            double s_im = gm_im[m + i1];
            for (int j = j1; j < j2; j++) {
                double x_re = f[j];
                double x_im = f[j + hn];
                double y_re = f[j + ht];
                double y_im = f[j + ht + hn];
                // y = s * y (complex mul)
                double tmp_re = y_re * s_re - y_im * s_im;
                double tmp_im = y_re * s_im + y_im * s_re;
                // f[j] = x + y
                f[j]     = x_re + tmp_re;
                f[j+hn]  = x_im + tmp_im;
                // f[j+ht] = x - y
                f[j+ht]     = x_re - tmp_re;
                f[j+ht+hn]  = x_im - tmp_im;
            }
        }
        t = ht;
    }
}

int main(int argc, char **argv) {
    const char *hex_fn = "hm_nonce40.hex";
    const char *gm_re_fn = "DOC/gm_tab_re.hex";
    const char *gm_im_fn = "DOC/gm_tab_im.hex";
    if (argc > 1) hex_fn = argv[1];

    int n = 512, hn = 256;

    // Read input
    uint16_t c_int[512] = {0};
    if (read_u16_hex(hex_fn, c_int, 512) != 0) return 1;

    // Convert to double (matching Verilog u16_to_f64 → $bitstoreal)
    double f[512];
    for (int i = 0; i < n; i++) {
        uint64_t bits = u16_to_f64(c_int[i]);
        f[i] = *(double*)&bits;
    }

    printf("Input c[0]=%.17g  c[1]=%.17g  c[256]=%.17g\n", f[0], f[1], f[256]);

    // Load GM table
    double *gm_re = calloc(1024, sizeof(double));
    double *gm_im = calloc(1024, sizeof(double));
    if (load_gm(gm_re_fn, gm_im_fn, gm_re, gm_im, 1024) != 0) return 1;

    printf("GM[0]=(%.17g, %.17g)  GM[2]=(%.17g, %.17g)  GM[4]=(%.17g, %.17g)\n",
           gm_re[0], gm_im[0], gm_re[2], gm_im[2], gm_re[4], gm_im[4]);

    // Run FFT
    falcon_fft(f, 9, gm_re, gm_im);

    // Output in RTL hex format (matching write_fft_poly_rtl_hex)
    for (int i = 0; i < hn; i++) {
        uint64_t re = *(uint64_t*)&f[i];
        uint64_t im = *(uint64_t*)&f[i + hn];
        printf("%016llx%016llx00000000000000000000000000000000\n",
               (unsigned long long)im, (unsigned long long)re);
    }
    for (int i = hn; i < n; i++) {
        int mirror = n - 1 - i;
        uint64_t re = *(uint64_t*)&f[mirror];
        uint64_t im_raw = *(uint64_t*)&f[mirror + hn];
        // fpr_neg: flip sign bit
        uint64_t im = im_raw ^ 0x8000000000000000ULL;
        printf("%016llx%016llx00000000000000000000000000000000\n",
               (unsigned long long)im, (unsigned long long)re);
    }

    free(gm_re); free(gm_im);
    return 0;
}

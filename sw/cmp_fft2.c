// cmp_fft2.c — Compare HW vs C FFT (clean hex output)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

static uint64_t u16_to_f64(uint16_t x) {
    if (x == 0) return 0;
    int pos = 0;
    for (int i = 0; i < 16; i++) {
        int bit = 15 - i;
        if (x & (1u << bit)) { pos = bit; break; }
    }
    uint64_t x64 = (uint64_t)x;
    uint64_t exp = 1023ULL + (uint64_t)pos;
    uint64_t frac = ((x64 << (63 - pos)) >> 11) & 0x000FFFFFFFFFFFFFULL;
    return (exp << 52) | frac;
}

static int read_u16_hex(const char *fn, uint16_t *dst, int n) {
    FILE *f = fopen(fn, "r");
    if (!f) { fprintf(stderr, "Cannot open %s\n", fn); return -1; }
    char line[128];
    for (int w = 0; w < (n + 15) / 16; w++) {
        if (!fgets(line, sizeof(line), f)) break;
        for (int j = 0; j < 16 && (w * 16 + j) < n; j++) {
            int off = (15 - j) * 4;
            unsigned int val;
            sscanf(line + off, "%04x", &val);
            dst[w * 16 + j] = (uint16_t)val;
        }
    }
    fclose(f);
    return 0;
}

static int load_gm(const char *re_fn, const char *im_fn,
                   double *gm_re, double *gm_im, int sz) {
    FILE *fre = fopen(re_fn, "r");
    FILE *fim = fopen(im_fn, "r");
    if (!fre || !fim) return -1;
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

static void falcon_fft(double *f, int logn, double *gm_re, double *gm_im) {
    int n = 1 << logn, hn = n >> 1, t = hn;
    for (int u = 1, m = 2; u < logn; u++, m <<= 1) {
        int ht = t >> 1, hm = m >> 1;
        for (int i1 = 0, j1 = 0; i1 < hm; i1++, j1 += t) {
            int j2 = j1 + ht;
            double s_re = gm_re[m + i1], s_im = gm_im[m + i1];
            for (int j = j1; j < j2; j++) {
                double x_re = f[j], x_im = f[j + hn];
                double y_re = f[j + ht], y_im = f[j + ht + hn];
                double tr = y_re * s_re - y_im * s_im;
                double ti = y_re * s_im + y_im * s_re;
                f[j] = x_re + tr; f[j+hn] = x_im + ti;
                f[j+ht] = x_re - tr; f[j+ht+hn] = x_im - ti;
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
    uint16_t c_int[512] = {0};
    if (read_u16_hex(hex_fn, c_int, 512) != 0) return 1;

    double f[512];
    for (int i = 0; i < n; i++) {
        uint64_t bits = u16_to_f64(c_int[i]);
        f[i] = *(double*)&bits;
    }

    double *gm_re = calloc(1024, sizeof(double));
    double *gm_im = calloc(1024, sizeof(double));
    if (load_gm(gm_re_fn, gm_im_fn, gm_re, gm_im, 1024) != 0) return 1;

    falcon_fft(f, 9, gm_re, gm_im);

    for (int i = 0; i < hn; i++) {
        uint64_t re = *(uint64_t*)&f[i];
        uint64_t im = *(uint64_t*)&f[i + hn];
        printf("%016llx%016llx00000000000000000000000000000000\n",
               (unsigned long long)im, (unsigned long long)re);
    }
    for (int i = hn; i < n; i++) {
        int mirror = n - 1 - i;
        uint64_t re = *(uint64_t*)&f[mirror];
        uint64_t im = *(uint64_t*)&f[mirror + hn] ^ 0x8000000000000000ULL;
        printf("%016llx%016llx00000000000000000000000000000000\n",
               (unsigned long long)im, (unsigned long long)re);
    }

    free(gm_re); free(gm_im);
    return 0;
}

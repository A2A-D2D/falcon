/*
 * Falcon Hardware Key Generator
 *
 * Generates all key material needed by the FalconSign hardware accelerator
 * to run the complete signing process. Uses the official Falcon reference
 * implementation.
 *
 * Output files (all in Verilog $readmemh hex format):
 *   b00.hex, b01.hex, b10.hex, b11.hex  — B matrix (FFT representation)
 *   tree.hex                            — LDL tree
 *   h_ntt.hex                           — public key h in NTT+Montgomery format
 *   hm.hex                              — hashed message (challenge c)
 *   expanded_key.bin                    — raw binary dump of expanded key
 *
 * Build:
 *   gcc -std=c99 -Wall -O2 -o gen_falcon_hw_key gen_falcon_hw_key.c \
 *       fpr.c fft.c keygen.c sign.c vrfy.c common.c shake.c rng.c codec.c falcon.c
 *
 * Memory map for FalconSign hardware (falconsign_top.v, 256-bit words):
 *   B00: addr 3072..3583  (512 words)
 *   B01: addr 3584..4095  (512 words)
 *   B10: addr 4096..4607  (512 words)
 *   B11: addr 4608..5119  (512 words)
 *   Tree: addr 1024..?   (treesize words)
 *   h_ntt: addr 5760..5791 (32 words, 16 uint16 per word)
 *
 * Each 256-bit memory word for complex data stores:
 *   [63:0] = real part (f64), [127:64] = imag part (f64), [255:128] = unused
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "inner.h"

/* ─── Hardcoded Falcon-512 test vectors (from official test_falcon.c) ─── */

static const int8_t ntru_f_512[] = {
    4, -4, 0, -6, 6, -6, 2, 1, -8, 0, -2, 0, -1, -1, -4, 8, -5, 3,
    -2, 2, 0, -5, -2, -1, 3, -4, -5, -1, 8, 1, 1, 7, 5, 1, 6, 2, -1,
    -13, 1, -4, 9, -4, -2, 4, -4, 0, -1, -1, -3, 2, 1, 1, 1, 3, -3,
    2, -1, -1, -5, 9, 4, -7, -3, -8, -3, -2, -3, -6, -6, -3, -2, -2,
    2, 1, -10, -2, -2, 4, 2, 0, -2, -2, 4, -3, 5, 2, -2, 3, 8, 1, 8,
    -3, -4, 2, 7, -5, -4, -2, -2, -3, 5, -5, 0, -3, -5, 3, -6, -2,
    3, 0, 3, 1, 2, -2, 1, 6, -1, -7, 0, -5, 3, -5, 9, 0, 1, 5, -4,
    0, 5, -1, 4, 3, 5, -6, 2, 0, -7, 1, 0, 0, 2, 4, 1, -7, -3, 4, 4,
    -2, -7, -5, 6, 3, 2, -5, 6, -1, -1, -4, 1, 2, 1, 2, -10, -9, -9,
    -1, 3, -2, -2, -6, 1, -2, -4, -1, 2, 3, 8, 2, 1, -1, 8, 0, 7, 3,
    1, 5, 0, -7, 1, -6, -4, 4, 2, 0, 0, -3, 2, 0, -3, 0, 7, -1, -1,
    -7, 2, 5, 3, 0, 1, 6, -2, -1, 2, 0, -1, -3, -6, -5, -5, -4, 0,
    1, 7, 1, -3, 2, 2, -5, 0, -4, 3, -4, 5, 3, 4, 7, -2, 15, -3, 1,
    1, 4, 5, -9, -3, 4, 2, -4, -4, -3, -1, -4, 3, -1, 1, -8, -4, -1,
    0, -3, 1, -1, 3, 3, 3, -3, -6, -7, 7, 0, -6, 2, -1, 4, 7, 1, 4,
    0, 1, 6, -1, -2, -2, 5, 0, 6, -3, -2, -5, 3, -1, 0, 5, -2, 8,
    -5, -4, 1, -3, 8, 2, -4, 1, 6, 0, 0, -1, 0, -4, -5, -2, 3, -2,
    5, 1, 4, 5, -4, 4, -1, 4, -5, -2, 1, 3, -5, 1, 2, -2, 0, -5, 1,
    8, -3, -4, 3, -2, -3, -4, 4, 3, -2, 6, -3, -2, 4, 0, -2, 0, -5,
    1, -9, 5, 6, -2, -6, 1, 5, -1, -7, 1, 2, 5, 2, 0, -1, 0, -2, -4,
    -1, -8, 5, -5, 9, -4, -4, 2, -5, -1, 0, 1, 4, 3, 1, -2, -7, -8,
    -4, -4, 4, 3, -1, 4, -1, -1, 1, 0, 6, 1, 0, -6, -2, 0, -3, 0,
    -1, -1, 0, 3, -5, -2, -5, 6, 2, -4, -3, 4, -8, 1, -1, 4, -3, 5,
    -2, 8, 7, -1, -3, -3, -2, 0, -4, 4, 0, -6, -4, -2, 5, 8, -3, 3,
    -1, 0, -5, -5, 0, 2, -5, -2, -3, 1, 6, 3, 1, -3, 4, -3, 0, -7,
    -1, -3, 1, -5, 1, -4, -2, 2, 4, 0, 1, 5, 2, 2, -3, -5, -8, 4,
    -2, -3, 2, 2, 0, 8, -5, 2, -7, 0, 3, -1, 0, 4, -3, 1, -2, -4,
    -6, -5, 0, -4, 1, -3, 9, 1, -3, -2, -3, 5, -1, -4, -7, 1, 1
};

static const int8_t ntru_g_512[] = {
    -6, -2, 4, -8, -4, 2, 3, 4, 1, -1, 3, 0, 2, 3, -3, 1, -7, -5, 3,
    -3, -1, 3, -3, 8, -6, -6, 0, 6, 4, 7, 3, 5, 0, -5, -3, -5, 7, 3,
    -1, -4, 3, 4, -1, 1, 3, -3, -4, -4, 4, -5, -1, 3, 7, -2, -4, -2,
    -3, -1, -2, -1, -2, -6, -7, -3, -6, -3, -6, 4, -1, -5, 1, 4, -4,
    3, -1, -6, 6, -2, 2, -6, 5, -7, 8, -3, 0, -2, 0, 7, 1, 3, 6, 4,
    -5, 2, 2, 2, 4, -4, -5, -4, -3, 4, -7, 7, -6, -2, -7, 1, -2, -2,
    -3, 1, 3, 7, 0, -1, -5, 4, -8, -8, 0, 3, 6, -3, 2, 6, -1, 1, -5,
    -4, 2, -3, 8, -2, 2, 3, 0, 1, 6, 4, 4, -4, -1, -3, -2, -5, 3, 9,
    0, 4, -1, 1, -4, 0, 3, 0, -2, 8, 0, 1, 0, -1, 1, 9, -1, -4, -1,
    3, 5, -2, -2, 1, -1, 1, -1, 0, 1, 0, -6, -2, 0, 7, -3, -4, -1,
    -6, -2, 5, -2, 0, 4, -3, -5, 0, 1, -1, -3, 5, 5, -4, -4, -5, -1,
    9, -1, -5, -7, -1, -2, 4, 2, 5, -4, -1, -5, 8, -3, -6, -2, 1,
    -2, 1, 1, 4, -4, -1, 4, 1, 1, 0, -5, 1, 7, 2, -3, 3, -2, -4, 1,
    -6, -1, -3, 7, 6, 0, -2, 2, -6, -4, -3, 2, -7, 7, 0, -11, -1, 3,
    4, 0, 6, -8, -4, -1, 1, 0, -3, 7, 0, 0, -2, -1, -4, 0, -1, -3,
    7, -6, -2, -2, -1, 0, -2, 8, -6, 4, 4, 6, -2, -1, 0, -13, 1, 2,
    0, 5, -7, 3, -2, -6, -3, -4, 4, -1, 1, 3, -6, 1, -5, -8, 2, -11,
    -1, 2, -2, 0, 0, 1, 1, -4, -5, 0, 1, 0, 1, -6, -2, 2, 0, 7, 1,
    -1, 1, -2, 1, -3, 1, 2, 1, -7, -2, 2, -1, 4, 1, -2, -2, 0, 4,
    -3, -6, 2, 3, 1, 1, -4, 6, -2, -4, -3, 0, 4, -5, 0, 1, 8, 2, 2,
    -1, 1, -2, -4, -1, 4, 4, -1, 7, 2, -1, -3, -8, 3, 1, 1, 0, -1,
    1, -7, -8, 2, 1, -2, 1, 0, 4, 1, 1, -2, -1, -5, 3, -4, -1, -1,
    -8, 2, -4, 3, 2, -5, 0, 1, 5, 2, -5, -2, 3, 7, 5, 6, 5, -2, 1,
    3, -7, 7, -3, -8, -2, 2, 3, 3, 5, -2, -4, -1, 7, -2, 7, -3, -2,
    0, 3, 5, 0, 0, 4, 8, -1, -5, 3, -2, -2, -5, -5, -2, 2, 5, -8,
    -1, -2, -4, 6, 0, 6, -5, -1, -5, -6, 9, 5, -2, 4, -1, -8, -2,
    -2, 1, -8, -5, 6, -1, 0, 5, -6, -3, -3, -2, -6, -2, 0, -1, -3,
    7, -3, -1, 3, 6, 3, -2, -4, 2, 1, -1, 11, 3, 4, -1, -6, 1, 2, 3, 3
};

static const int8_t ntru_F_512[] = {
    -3, -27, 4, 18, 39, 7, 20, -13, 33, -29, 3, 38, 30, 26, -6, 24,
    -26, 16, 24, -48, -18, -21, 3, -14, -2, 6, -9, 42, 22, 21, 33,
    -27, -14, -14, -56, -68, -2, -33, 6, -38, -43, 21, 13, 6, 2,
    -69, -10, -30, -27, 23, -1, 41, -21, 11, -20, 15, 39, 5, 41, 15,
    -28, -34, 9, -11, 9, -1, -8, 61, 8, 13, -23, 2, 7, -23, -21,
    -54, -11, -9, -19, 40, 37, -2, -16, 19, -16, 2, -78, -35, -19,
    11, 17, -46, -16, 25, 0, 22, 13, -15, -33, 13, -15, -34, 33,
    -13, 38, 39, 37, -29, 40, 7, 63, 35, 15, 21, -24, 16, -6, 30,
    12, 18, 61, 17, -11, -15, 11, 0, -15, -2, -14, -26, -1, -42,
    -10, -52, 64, 45, 22, 6, -22, 32, -50, -16, -12, -16, -8, 34,
    -17, -18, 7, 19, 37, 41, -5, -22, -12, -7, -17, -27, -17, 4, 36,
    0, 22, -4, -50, 24, 30, 5, 1, -50, 43, 0, 0, -6, -9, 34, 0, 14,
    -27, 17, 35, -30, -13, 3, -23, -46, 17, -34, 30, 24, 47, 31, -7,
    11, 10, 16, 30, 27, -4, 11, -4, -14, -28, 49, 0, 27, -5, -10,
    53, -50, -13, -15, 13, -10, -26, 2, -3, 88, 22, -27, 40, -23, 3,
    -42, 2, -27, -12, 35, 26, -33, 38, -42, -5, 17, -24, 6, -10, 13,
    -10, -30, -35, -17, 25, 49, -29, 48, 19, 37, 48, -25, -31, -41,
    -15, -1, 19, -17, -7, -16, 2, 5, 12, 0, -15, -19, -6, -32, -4,
    -56, 14, -6, -7, 17, 24, -1, 17, -35, 5, 3, -64, -15, 4, 0, -31,
    4, -10, -18, 55, 13, -13, 23, -30, -11, -29, -21, 15, -18, 30,
    39, 16, -27, 31, 4, 31, 39, -49, 11, -25, 37, -42, -72, 28, -57,
    13, 34, 6, 10, -17, -3, -19, -43, -1, -32, 9, -11, 9, 11, -23,
    6, 28, -34, -12, -42, -7, 42, -18, -2, 22, -30, -4, -42, 10, 54,
    -16, 19, -23, -4, 18, -58, 26, -3, -38, 20, 38, 23, 20, 0, 10,
    49, 47, 18, 27, -11, -10, -14, 0, 6, 6, -18, -6, 14, -38, -16,
    12, -17, 17, -21, -52, -3, -53, 9, 9, -4, 44, 9, -4, 17, 2, 10,
    -28, -13, 28, -12, 11, -33, -2, 33, 0, -51, 2, -33, 20, -47, 23,
    42, 2, 52, -18, -17, 35, 6, 27, 3, 11, 24, -8, 0, -35, -44, -22,
    -49, 61, 3, -15, -2, -14, 46, -24, -10, -24, -24, -21, -10, -51,
    -3, 31, 20, 1, -44, 18, 9, 38, 26, -17, -8, 2, 33, 24, -8, -9,
    -20, 32, 54, 47, -11, 40, 3, -58, 13, 17, 29, -21, 27, 4, -31,
    14, 14, 17, 19, -29, 19, -86, -29, -15, -35, 18, 53, -10, 9, 13,
    -38, 9, -4, 80, 0, 6, 1, 15, -14, 0, -5, 45, 26, 50, 28, 21, 1,
    -8, -6, 12, 32, 5, -21, -1, 54, 14, 22, 27, 6, 8, -18, 33, -5
};

/* ─── Helper functions ─── */

/* Write an fpr (double) as a 16-char hex string */
static uint64_t fpr_bits(fpr x) {
    uint64_t u;
    memcpy(&u, &x, sizeof(u));
    return u;
}

/* Write Verilog memory file (256-bit words = 4 x 64-bit, each line is one word).
 * For complex data, we interpret fpr array as [N/2 complex numbers]:
 *   f[0]=re0, f[1]=im0, f[2]=re1, f[3]=im1, ...
 * Each memory word: {64'd0, im, re} (128 bits used of 256, upper 128 = 0)
 */
static void __attribute__((unused))
write_complex_hex(const char *filename, const fpr *data, size_t n_fpr,
                  const char *desc) {
    FILE *f = fopen(filename, "w");
    if (!f) { fprintf(stderr, "Cannot open %s\n", filename); exit(1); }
    size_t npairs = n_fpr / 2;
    for (size_t i = 0; i < npairs; i++) {
        uint64_t re, im;
        memcpy(&re, &data[2*i + 0], sizeof(re));
        memcpy(&im, &data[2*i + 1], sizeof(im));
        /* 256-bit word: [255:224]=0, [223:192]=0, [191:160]=0, [159:128]=0,
         *               [127:96]=im_hi, [95:64]=im_lo, [63:32]=re_hi, [31:0]=re_lo */
        fprintf(f, "%08x%08x%08x%08x%08x%08x%08x%08x\n",
                0u, 0u, 0u, 0u,
                (unsigned)(im >> 32), (unsigned)(im & 0xFFFFFFFFu),
                (unsigned)(re >> 32), (unsigned)(re & 0xFFFFFFFFu));
    }
    fclose(f);
    printf("Wrote %s: %zu complex words (%zu fpr values) — %s\n",
           filename, npairs, n_fpr, desc);
}

static void write_word256(FILE *f, uint64_t lane3, uint64_t lane2,
                          uint64_t lane1, uint64_t lane0) {
    fprintf(f, "%016llx%016llx%016llx%016llx\n",
            (unsigned long long)lane3,
            (unsigned long long)lane2,
            (unsigned long long)lane1,
            (unsigned long long)lane0);
}

static void write_fft_poly_rtl_hex(const char *filename, const fpr *data,
                                   size_t n, const char *desc) {
    FILE *f = fopen(filename, "w");
    if (!f) { fprintf(stderr, "Cannot open %s\n", filename); exit(1); }
    size_t hn = n >> 1;
    for (size_t i = 0; i < hn; i++) {
        write_word256(f, 0, 0, fpr_bits(data[i + hn]), fpr_bits(data[i]));
    }
    /*
     * Official Falcon keeps only the first half of the negacyclic FFT
     * values. For real polynomials:
     *   f(w_{N-1-j}) = conj(f(w_j))
     * so the second half must be generated by reversed conjugates. This
     * keeps B-hat multiplication and the subsequent RTL IFFT from losing the
     * implicit half of the spectrum.
     */
    for (size_t i = hn; i < n; i++) {
        size_t mirror = n - 1 - i;
        write_word256(f, 0, 0,
                      fpr_bits(fpr_neg(data[mirror + hn])),
                      fpr_bits(data[mirror]));
    }
    fclose(f);
    printf("Wrote %s: %zu RTL complex words (%zu official half-complex + %zu conjugate mirror) - %s\n",
           filename, n, hn, hn, desc);
}

static void write_fft_poly_official_hex(const char *filename, const fpr *data,
                                        size_t n, const char *desc) {
    FILE *f = fopen(filename, "w");
    if (!f) { fprintf(stderr, "Cannot open %s\n", filename); exit(1); }
    size_t hn = n >> 1;
    for (size_t i = 0; i < hn; i++) {
        write_word256(f, 0, 0, fpr_bits(data[i + hn]), fpr_bits(data[i]));
    }
    fclose(f);
    printf("Wrote %s: %zu official half-complex words - %s\n",
           filename, hn, desc);
}

static void write_gm_tab_hex(const char *re_name, const char *im_name) {
    FILE *fre = fopen(re_name, "w");
    FILE *fim = fopen(im_name, "w");
    if (!fre || !fim) {
        fprintf(stderr, "Cannot open gm table output files\n");
        exit(1);
    }
    for (size_t u = 0; u < 1024; u++) {
        uint64_t re = fpr_bits(fpr_gm_tab[(u << 1) + 0]);
        uint64_t im = fpr_bits(fpr_gm_tab[(u << 1) + 1]);
        if (u == 0) {
            re = fpr_bits(fpr_one);
            im = fpr_bits(fpr_zero);
        }
        fprintf(fre, "%016llx\n", (unsigned long long)re);
        fprintf(fim, "%016llx\n", (unsigned long long)im);
    }
    fclose(fre);
    fclose(fim);
    printf("Wrote %s/%s: Falcon gm_tab for ffSampling split/merge\n",
           re_name, im_name);
}

/* Write packed uint16 data (for public key h_ntt, hm, etc.)
 * Each 256-bit memory word packs 16 uint16 values.
 */
static void write_u16_hex(const char *filename, const uint16_t *data, size_t n,
                           const char *desc) {
    FILE *f = fopen(filename, "w");
    if (!f) { fprintf(stderr, "Cannot open %s\n", filename); exit(1); }
    size_t words = (n + 15) / 16;
    for (size_t w = 0; w < words; w++) {
        /* Pack 16 uint16 values into one 256-bit word */
        /* Word layout: data[16*w+15]..data[16*w+0] from MSB to LSB */
        char line[65];
        int pos = 64;
        line[pos] = '\0';
        for (int j = 0; j < 16; j++) {
            uint16_t v = ((w * 16 + j) < n) ? data[w * 16 + j] : 0;
            pos -= 4;
            char tmp[5];
            snprintf(tmp, sizeof(tmp), "%04x", v);
            memcpy(line + pos, tmp, 4);
        }
        fprintf(f, "%s\n", line);
    }
    fclose(f);
    printf("Wrote %s: %zu words (%zu uint16 values) — %s\n",
           filename, words, n, desc);
}

static void write_fpr_scalar_hex(const char *filename, const fpr *data,
                                 size_t n, const char *desc) {
    FILE *f = fopen(filename, "w");
    if (!f) { fprintf(stderr, "Cannot open %s\n", filename); exit(1); }
    for (size_t i = 0; i < n; i++) {
        write_word256(f, 0, 0, 0, fpr_bits(data[i]));
    }
    fclose(f);
    printf("Wrote %s: %zu scalar fpr words - %s\n", filename, n, desc);
}

static size_t hw_ffldl_treesize(unsigned logn) {
    return ((size_t)logn + 1u) << logn;
}

static void flatten_tree_node(const fpr *tree, size_t official_off,
                              unsigned node_logn, unsigned level,
                              unsigned index, uint64_t *lane0,
                              uint64_t *lane1, size_t flat_words) {
    if (node_logn == 0) {
        size_t flat_off = 255u + index;
        if (flat_off < flat_words) {
            lane0[flat_off] = fpr_bits(tree[official_off]);
            lane1[flat_off] = fpr_bits(tree[official_off]);
        }
        return;
    }

    size_t flat_off = (((size_t)1u << level) - 1u) + index;
    size_t node_n = (size_t)1u << node_logn;
    size_t node_hn = node_n >> 1;
    if (flat_off < flat_words) {
        lane0[flat_off] = fpr_bits(tree[official_off]);
        lane1[flat_off] = fpr_bits(tree[official_off + node_hn]);
    }

    flatten_tree_node(tree, official_off + node_n, node_logn - 1u,
                      level + 1u, index << 1, lane0, lane1, flat_words);
    flatten_tree_node(tree,
                      official_off + node_n + hw_ffldl_treesize(node_logn - 1u),
                      node_logn - 1u, level + 1u, (index << 1) + 1u,
                      lane0, lane1, flat_words);
}

static void write_tree_flat_rtl_hex(const char *filename, const fpr *tree,
                                    unsigned logn, const char *desc) {
    size_t n = (size_t)1u << logn;
    size_t flat_words = 255u + n;
    uint64_t *lane0 = calloc(flat_words, sizeof(*lane0));
    uint64_t *lane1 = calloc(flat_words, sizeof(*lane1));
    if (!lane0 || !lane1) {
        fprintf(stderr, "Out of memory while flattening tree\n");
        exit(1);
    }

    flatten_tree_node(tree, 0, logn, 0, 0, lane0, lane1, flat_words);

    FILE *f = fopen(filename, "w");
    if (!f) { fprintf(stderr, "Cannot open %s\n", filename); exit(1); }
    for (size_t i = 0; i < flat_words; i++) {
        write_word256(f, 0, 0, lane1[i], lane0[i]);
    }
    fclose(f);
    free(lane0);
    free(lane1);
    printf("Wrote %s: %zu flattened RTL tree words - %s\n",
           filename, flat_words, desc);
}

/* ─── Full polynomial tree dump ───
 *
 * The official ffLDL tree stores, at each inner node of level logn=L,
 * the complete L10 matrix row as a polynomial in Falcon FFT representation:
 *   2^L fpr values  =  2^{L-1} complex pairs
 *
 * We walk the recursive tree and write every complex pair of every node
 * into a flat memory image organised level-by-level.  Leaf nodes (L=0)
 * carry a single scalar sigma (stored as re=sigma, im=sigma so that the
 * SamplerZ can read it as a complex word).
 *
 * Total RTL words = sum_{L=1..logn} (2^{L-1} * 2^{logn-L})  +  2^{logn}
 *                 = logn * 2^{logn-1}  +  2^{logn}
 *                 = logn * n/2  +  n   =  9*256 + 512  =  2816
 */

/* Per-node descriptor written into the companion .map file. */
static void dump_full_tree_node(const fpr *tree, size_t official_off,
                                unsigned node_logn, unsigned level,
                                unsigned index,
                                uint64_t *flat_buf,
                                FILE *map_f, size_t n) {
    size_t node_n    = (size_t)1u << node_logn;
    size_t node_hn   = node_n >> 1;          /* number of complex pairs */
    size_t my_base;

    if (node_logn == 0) {
        /* leaf: one scalar sigma */
        uint64_t sigma_bits = fpr_bits(tree[official_off]);
        my_base = ((size_t)level * 256u) + index;
        flat_buf[2 * my_base] = sigma_bits;       /* lane0 */
        flat_buf[2 * my_base + 1] = sigma_bits;   /* lane1 */
        if (map_f) {
            fprintf(map_f, "LEAF   L=%u idx=%-4u  flat_off=%-6zu  sigma= %016llx\n",
                    level, index, my_base,
                    (unsigned long long)sigma_bits);
        }
        return;
    }

    /* inner node:  node_hn consecutive complex pairs */
    my_base = ((size_t)level * 256u) + ((size_t)index * node_hn);
    for (size_t k = 0; k < node_hn; k++) {
        uint64_t re = fpr_bits(tree[official_off + k]);          /* real part */
        uint64_t im = fpr_bits(tree[official_off + k + node_hn]); /* imag part */
        flat_buf[my_base * 2 + 2*k]     = re;   /* lane0 */
        flat_buf[my_base * 2 + 2*k + 1] = im;   /* lane1 */
    }
    if (map_f) {
        fprintf(map_f, "INNER  L=%u idx=%-4u  flat_off=%-6zu  pairs=%-5zu  "
                "official_off=%-6zu\n",
                level, index, my_base, node_hn, official_off);
    }

    /* recurse into children */
    size_t child_treesize = hw_ffldl_treesize(node_logn - 1u);
    dump_full_tree_node(tree, official_off + node_n,
                        node_logn - 1u, level + 1u, index << 1,
                        flat_buf, map_f, n);
    dump_full_tree_node(tree, official_off + node_n + child_treesize,
                        node_logn - 1u, level + 1u, (index << 1) + 1u,
                        flat_buf, map_f, n);
}

static void write_tree_full_poly_hex(const char *hex_name,
                                      const char *map_name,
                                      const fpr *tree, unsigned logn,
                                      const char *desc) {
    size_t n            = (size_t)1u << logn;
    size_t total_pairs  = ((size_t)logn * n) >> 1;   /* logn * n/2 */
    size_t total_words  = total_pairs + n;            /* + leaves */
    size_t total_lanes  = total_words * 2;            /* lane0 + lane1 per word */

    uint64_t *flat = calloc(total_lanes, sizeof(*flat));
    if (!flat) {
        fprintf(stderr, "Out of memory for full tree dump (%zu lanes)\n",
                total_lanes);
        exit(1);
    }

    FILE *map_f = NULL;
    if (map_name) {
        map_f = fopen(map_name, "w");
        if (!map_f) fprintf(stderr, "Warning: cannot open %s\n", map_name);
    }

    dump_full_tree_node(tree, 0, logn, 0, 0, flat, map_f, n);

    if (map_f) {
        fprintf(map_f, "\nTotal words: %zu  (inner pairs: %zu  leaves: %zu)\n",
                total_words, total_pairs, n);
        fclose(map_f);
        printf("Wrote %s — per-node address map\n", map_name);
    }

    /* Write hex file: one 256-bit word per line (lane0, lane1 per word) */
    FILE *f = fopen(hex_name, "w");
    if (!f) { fprintf(stderr, "Cannot open %s\n", hex_name); exit(1); }
    for (size_t i = 0; i < total_words; i++) {
        write_word256(f, 0, 0, flat[2*i + 1], flat[2*i]);
    }
    fclose(f);
    free(flat);

    printf("Wrote %s: %zu RTL words (full L10 polynomial per node) — %s\n",
           hex_name, total_words, desc);
    printf("  Inner nodes: %zu complex-pair words\n", total_pairs);
    printf("  Leaves:      %zu scalar words\n", n);
    printf("  Total:       %zu words  (addr range needs %zu slots)\n",
           total_words, total_words);
}

/* ─── Falcon GM table ROM generator for ffSampling split/merge ─── */
static void write_gm_rom_hex(const char *re_name, const char *im_name) {
    /* fpr_gm_tab[] is the official Falcon GM table (declared in inner.h).
     * For split/merge at ffSampling level L (0=root, node_logn=9-L):
     *   hn = 2^(8-L)  = 256 >> L
     *   idx = 0 .. hn/2-1  (number of complex pairs to process in this level)
     *   needed value:  gm_tab[idx + hn]  as complex (re, im)
     *
     * Total ROM entries = sum_{L=0..8} 2^(7-L) = 256 entries.
     * Address mapping:  addr = base[L] + idx
     *   base[L] = 256 - 2^(8-L)   (cumulative sum of previous levels' sizes)
     */
    FILE *fr = fopen(re_name, "w");
    FILE *fi = fopen(im_name, "w");
    if (!fr || !fi) { fprintf(stderr, "Cannot open gm rom files\n"); exit(1); }

    for (unsigned L = 0; L <= 8; L++) {
        unsigned hn = 256u >> L;          /* 2^(8-L) */
        unsigned limit = hn >> 1;         /* hn/2 pairs to process */
        for (unsigned idx = 0; idx < limit; idx++) {
            unsigned k = idx + hn;         /* index into gm_tab */
            uint64_t re = fpr_bits(fpr_gm_tab[(k << 1) + 0]);
            uint64_t im = fpr_bits(fpr_gm_tab[(k << 1) + 1]);
            fprintf(fr, "%016llx\n", (unsigned long long)re);
            fprintf(fi, "%016llx\n", (unsigned long long)im);
        }
    }
    fclose(fr);
    fclose(fi);
    printf("Wrote %s + %s — Falcon GM ROM for ffSampling split/merge (256 entries)\n",
           re_name, im_name);
}

int main(void) {
    unsigned logn = 9;
    size_t n = 512;  /* 2^logn */

    printf("=== FalconSign Hardware Key Generator (Falcon-512) ===\n\n");

    /* ─── Step 0: Generate Falcon GM ROM for ffSampling split/merge ─── */
    write_gm_rom_hex("gm_rom_re.hex", "gm_rom_im.hex");

    /* ─── Step 1: Compute expanded private key ─── */
    /* expanded_key = B matrix (4*n fpr) + LDL tree (treesize fpr) */
    /* treesize = (logn+1)*n = 10*512 = 5120 */
    size_t treesize = ((size_t)(logn + 1)) << logn;
    size_t expanded_key_fpr = 4 * n + treesize;  /* 2048 + 5120 = 7168 */
    size_t expanded_key_bytes = expanded_key_fpr * sizeof(fpr);
    fpr *expanded_key = calloc(expanded_key_fpr, sizeof(fpr));

    /* Temporary buffer: use the largest needed size (sign_dyn: 72*2^logn + 7 bytes) */
    size_t tmp_bytes = (72u << logn) + 7;
    uint8_t *tmp = calloc(tmp_bytes, 1);

    /* ─── Also need a separate buffer for complete_private (4*2^logn bytes) ─── */
    uint8_t *tmp_cp = calloc((4u << logn), 1);

    /* Compute G (4th private key element) from f, g, F */
    int8_t G_computed[512];
    if (!Zf(complete_private)(G_computed, ntru_f_512, ntru_g_512,
                               ntru_F_512, logn, tmp_cp)) {
        fprintf(stderr, "ERROR: complete_private failed\n");
        return 1;
    }

    /* Expand the private key */
    Zf(expand_privkey)(expanded_key, ntru_f_512, ntru_g_512,
                        ntru_F_512, G_computed, logn, tmp);

    printf("Expanded key: %zu fpr values = %zu bytes\n",
           expanded_key_fpr, expanded_key_bytes);

    /* Extract B matrix components */
    const fpr *b00 = expanded_key + 0;           /* offset 0 */
    const fpr *b01 = expanded_key + n;           /* offset n */
    const fpr *b10 = expanded_key + 2 * n;       /* offset 2n */
    const fpr *b11 = expanded_key + 3 * n;       /* offset 3n */
    const fpr *tree = expanded_key + 4 * n;      /* offset 4n */

    /* Write B matrix as RTL hex files. The *_official.hex files preserve
     * Falcon's native half-FFT view for cross-checking.
     */
    write_fft_poly_rtl_hex("b00.hex", b00, n, "B00 matrix (RTL full Hermitian)");
    write_fft_poly_rtl_hex("b01.hex", b01, n, "B01 matrix (RTL full Hermitian)");
    write_fft_poly_rtl_hex("b10.hex", b10, n, "B10 matrix (RTL full Hermitian)");
    write_fft_poly_rtl_hex("b11.hex", b11, n, "B11 matrix (RTL full Hermitian)");
    write_fft_poly_official_hex("b00_official.hex", b00, n, "B00 matrix");
    write_fft_poly_official_hex("b01_official.hex", b01, n, "B01 matrix");
    write_fft_poly_official_hex("b10_official.hex", b10, n, "B10 matrix");
    write_fft_poly_official_hex("b11_official.hex", b11, n, "B11 matrix");
    write_gm_tab_hex("gm_tab_re.hex", "gm_tab_im.hex");
    write_tree_full_poly_hex("tree.hex", "tree.map",
                             tree, logn,
                             "full L10 polynomial per node (level-by-level layout)");
    write_tree_flat_rtl_hex("tree_compact.hex", tree, logn,
                            "compact single-word-per-node (legacy, for reference)");
    write_fpr_scalar_hex("tree_official_fpr.hex", tree, treesize,
                         "official expanded ffLDL tree, scalar fpr dump");

    /* ─── Step 2: Compute public key h ─── */
    uint16_t h[512];
    if (!Zf(compute_public)(h, ntru_f_512, ntru_g_512, logn, tmp)) {
        fprintf(stderr, "ERROR: compute_public failed\n");
        return 1;
    }
    printf("\nPublic key h computed (mod q, %zu coefficients)\n", n);

    /* ─── Step 3: Convert h to NTT + Montgomery format ─── */
    write_u16_hex("h_ntt.hex", h, n, "h raw mod-q coefficients for RTL NTT");
    Zf(to_ntt_monty)(h, logn);
    write_u16_hex("h_ntt_monty.hex", h, n, "h (NTT+Montgomery reference)");
    printf("Public key h written raw for RTL and converted reference to NTT+Montgomery\n");

    /* ─── Step 4: Hash a test message to get challenge c ─── */
    static const uint8_t rtl_msg[32] = {
        'F','A','L','C','O','N','_','S',
        'I','G','N','_','T','E','S','T',
        '_','M','S','G','_','V','1','.',
        '0','_','_','_','_','_','_','_'
    };
    size_t msg_len = sizeof rtl_msg;
    uint8_t nonce[40] = {0}; /* zero nonce for deterministic output */

    inner_shake256_context sc;
    inner_shake256_init(&sc);
    inner_shake256_inject(&sc, rtl_msg, msg_len);
    inner_shake256_flip(&sc);

    uint16_t hm[512];
    Zf(hash_to_point_vartime)(&sc, hm, logn);
    write_u16_hex("hm.hex", hm, n, "hashed message (challenge c)");
    printf("Hashed message written (hm = c, %zu coefficients)\n", n);

    inner_shake256_init(&sc);
    inner_shake256_inject(&sc, nonce, sizeof nonce);
    inner_shake256_inject(&sc, rtl_msg, msg_len);
    inner_shake256_flip(&sc);
    uint16_t hm_nonce40[512];
    Zf(hash_to_point_vartime)(&sc, hm_nonce40, logn);
    write_u16_hex("hm_nonce40.hex", hm_nonce40, n,
                  "standard zero-nonce hashed message reference");

    fpr *target_t0 = calloc(n, sizeof(fpr));
    fpr *target_t1 = calloc(n, sizeof(fpr));
    if (!target_t0 || !target_t1) {
        fprintf(stderr, "Out of memory while computing target vectors\n");
        return 1;
    }
    for (size_t u = 0; u < n; u++) {
        target_t0[u] = fpr_of(hm[u]);
    }
    Zf(FFT)(target_t0, logn);
    memcpy(target_t1, target_t0, n * sizeof(*target_t0));
    Zf(poly_mul_fft)(target_t1, b01, logn);
    Zf(poly_mulconst)(target_t1, fpr_neg(fpr_inverse_of_q), logn);
    Zf(poly_mul_fft)(target_t0, b11, logn);
    Zf(poly_mulconst)(target_t0, fpr_inverse_of_q, logn);
    write_fft_poly_rtl_hex("t0_target.hex", target_t0, n,
                           "official sign_tree target t0 for RTL preload");
    write_fft_poly_rtl_hex("t1_target.hex", target_t1, n,
                           "official sign_tree target t1 for RTL preload");

    for (size_t u = 0; u < n; u++) {
        target_t0[u] = fpr_of(hm_nonce40[u]);
    }
    Zf(FFT)(target_t0, logn);
    write_fft_poly_rtl_hex("c_fft_nonce40.hex", target_t0, n,
                           "standard zero-nonce FFT(c) before target multiply");
    memcpy(target_t1, target_t0, n * sizeof(*target_t0));
    Zf(poly_mul_fft)(target_t1, b01, logn);
    Zf(poly_mulconst)(target_t1, fpr_neg(fpr_inverse_of_q), logn);
    Zf(poly_mul_fft)(target_t0, b11, logn);
    Zf(poly_mulconst)(target_t0, fpr_inverse_of_q, logn);
    write_fft_poly_rtl_hex("t0_target_nonce40.hex", target_t0, n,
                           "standard zero-nonce target t0 for RTL preload");
    write_fft_poly_rtl_hex("t1_target_nonce40.hex", target_t1, n,
                           "standard zero-nonce target t1 for RTL preload");

    /* ─── Step 5: Compute expected signature (for cross-check) ─── */
    /* We also run the full signing to get expected s2 */
    inner_shake256_context rng_sc;
    inner_shake256_init(&rng_sc);
    {
        uint8_t rng_seed[48] = {0x01,0x23,0x45,0x67,0x89,0xAB,0xCD,0xEF,
                                 0x01,0x23,0x45,0x67,0x89,0xAB,0xCD,0xEF,
                                 0x01,0x23,0x45,0x67,0x89,0xAB,0xCD,0xEF,
                                 0x01,0x23,0x45,0x67,0x89,0xAB,0xCD,0xEF,
                                 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
        inner_shake256_inject(&rng_sc, rng_seed, sizeof rng_seed);
    }
    inner_shake256_flip(&rng_sc);

    int16_t sig[512];
    /* Use sign_dyn for a self-contained signature computation */
    Zf(sign_dyn)(sig, &rng_sc, ntru_f_512, ntru_g_512,
                  ntru_F_512, G_computed, hm, logn, tmp);

    write_u16_hex("s2_expected.hex", (const uint16_t *)sig, n,
                  "expected s2 signature (int16)");
    printf("Expected signature s2 computed (for cross-check)\n");

    /* ─── Also output s1 which is in tmp after sign_dyn ─── */
    /* s1 from sign_dyn is centered int16; convert to mod-q uint16
     * to match the HW norm checker (falconsign_norm_i16_sig_check.v)
     * which expects s1 in [0, q-1] and center-lifts internally. */
    const int16_t *s1 = (const int16_t *)tmp;
    uint16_t s1_modq[512];
    for (size_t i = 0; i < n; i++) {
        int32_t x = s1[i];
        s1_modq[i] = (uint16_t)((x < 0) ? x + 12289 : x);
    }
    write_u16_hex("s1_expected.hex", s1_modq, n,
                  "expected s1 (mod-q uint16)");
    printf("Expected s1 from tmp buffer (converted to mod-q format)\n");
    printf("Expected s1 from tmp buffer\n");

    /* ─── Write raw binary dump ─── */
    {
        FILE *f = fopen("expanded_key.bin", "wb");
        if (f) {
            fwrite(expanded_key, sizeof(fpr), expanded_key_fpr, f);
            fclose(f);
            printf("\nRaw expanded key written to expanded_key.bin (%zu bytes)\n",
                   expanded_key_bytes);
        }
    }

    /* ─── Summary ─── */
    printf("\n=== Files generated ===\n");
#if 0
    printf("  b00.hex, b01.hex, b10.hex, b11.hex  — B matrix (FFT rep, 256 complex words each)\n");
    printf("  tree.hex                             — LDL tree (%zu complex words)\n", treesize/2);
    printf("  h_ntt.hex                            — public key (NTT+Montgomery, 32 words)\n");
    printf("  hm.hex                               — hashed message / challenge c (32 words)\n");
    printf("  s1_expected.hex, s2_expected.hex     — expected signature (for verification)\n");
    printf("  expanded_key.bin                     — raw binary dump\n");

#endif
    printf("  b00.hex, b01.hex, b10.hex, b11.hex  - RTL B matrix (512 words each)\n");
    printf("  b*_official.hex                      - official half-FFT B view (256 words each)\n");
    printf("  tree.hex                             - legacy compact tree (767 words)\n");
    printf("  tree_full_poly.hex + .map            - FULL L10 polynomial per node (2816 words)\n");
    printf("  tree_official_fpr.hex                - official scalar ffLDL tree (%zu fpr)\n", treesize);
    printf("  h_ntt.hex                            - public key (NTT+Montgomery, 32 words)\n");
    printf("  hm.hex, hm_nonce40.hex               - challenge c variants (32 words each)\n");
    printf("  t0_target.hex, t1_target.hex         - official preimage-center targets\n");
    printf("  t0_target_nonce40.hex, t1_target_nonce40.hex - zero-nonce standard targets\n");
    printf("  s1_expected.hex, s2_expected.hex     - expected signature (software reference)\n");
    printf("  expanded_key.bin                     - raw binary dump\n");

    printf("\n=== RTL-format notes ===\n");
    printf("  b00/b01/b10/b11.hex expand Falcon half-FFT data to 512 RTL words with conjugate mirrors.\n");
    printf("  tree_full_poly.hex is the RTL scheduler tree image and has 2816 words.\n");
    printf("  tree.hex is kept only as a compact legacy/debug dump.\n");
    printf("  tree_official_fpr.hex preserves the full official ffLDL tree.\n");
    printf("  hm.hex matches the current RTL 32-byte hardcoded message without nonce.\n");
    printf("  hm_nonce40.hex is the standard zero-nonce reference variant.\n");
    printf("  t0_target.hex and t1_target.hex are preimage-center preload helpers.\n");

    printf("\n=== Hardware Memory Map ===\n");
    printf("  T0:   addr %4d..%-4d  (512 words, load t0_target.hex)\n", 0, 511);
    printf("  T1:   addr %4d..%-4d  (512 words, load t1_target.hex)\n", 512, 1023);
    printf("  Tree: addr %4d..%-4d  (2816 words, load tree_full_poly.hex)\n", 1024, 3839);
    printf("  Z0:   addr %4d..%-4d  (512 words)\n", 3840, 4351);
    printf("  Z1:   addr %4d..%-4d  (512 words)\n", 4352, 4863);
    printf("  B00:  addr %4d..%-4d  (512 words, load b00.hex)\n", 4864, 5375);
    printf("  B01:  addr %4d..%-4d  (512 words, load b01.hex)\n", 5376, 5887);
    printf("  B10:  addr %4d..%-4d  (512 words, load b10.hex)\n", 5888, 6399);
    printf("  B11:  addr %4d..%-4d  (512 words, load b11.hex)\n", 6400, 6911);
    printf("  s2:   addr %4d..%-4d  (32 words)\n", 6912, 6943);
    printf("  c_int:addr %4d..%-4d  (32 words, load hm.hex)\n", 7424, 7455);
    printf("  h_ntt:addr %4d..%-4d  (32 words, load h_ntt.hex)\n", 7456, 7487);
    printf("  s1:   addr %4d..%-4d  (32 words)\n", 7488, 7519);

    /* Cleanup */
    free(expanded_key);
    free(target_t0);
    free(target_t1);
    free(tmp);
    free(tmp_cp);

    return 0;
}

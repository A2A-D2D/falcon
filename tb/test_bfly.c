#include <stdio.h>
#include <stdint.h>
#include <math.h>
// Same a,b,w as first FFT stage, first pair
int main() {
    // Use GM table values
    uint64_t gm_re[1024], gm_im[1024];
    FILE *fr = fopen("DOC/gm_tab_re.hex","r");
    FILE *fi = fopen("DOC/gm_tab_im.hex","r");
    for (int i=0;i<1024;i++) {
        fscanf(fr,"%llx",(unsigned long long*)&gm_re[i]);
        fscanf(fi,"%llx",(unsigned long long*)&gm_im[i]);
    }
    fclose(fr); fclose(fi);

    // First c values
    uint16_t c[512] = {0};
    FILE *fc = fopen("hm_nonce40.hex","r");
    char line[128];
    for (int w=0; w<32; w++) {
        fgets(line,sizeof(line),fc);
        for (int j=0; j<16; j++) {
            int off = (15-j)*4; unsigned int v;
            sscanf(line+off,"%04x",&v); c[w*16+j]=(uint16_t)v;
        }
    }
    fclose(fc);

    // u16_to_f64 (corrected)
    uint64_t u16f64[512];
    for (int i=0;i<512;i++) {
        uint16_t x=c[i];
        if(x==0){u16f64[i]=0;continue;}
        int pos=0;
        for(int k=0;k<16;k++){int bit=15-k;if(x&(1u<<bit)){pos=bit;break;}}
        u16f64[i]=(((uint64_t)1023+pos)<<52)|((((uint64_t)x<<(63-pos))>>11)&0xFFFFFFFFFFFFFULL);
    }
    double *f=(double*)u16f64;

    // Stage 1: u=1, m=2, t=256, ht=128, hm=1
    int hn=256;
    double s_re=*(double*)&gm_re[2], s_im=*(double*)&gm_im[2];
    printf("First butterfly twiddle: s=(%.17g, %.17g)\n",s_re,s_im);
    printf("  bits: re=0x%016llx im=0x%016llx\n",
        (unsigned long long)gm_re[2],(unsigned long long)gm_im[2]);

    // First pair: j=0, read f[0],f[256],f[128],f[384]
    double a_re=f[0], a_im=f[256], b_re=f[128], b_im=f[384];
    printf("First two pairs:\n");
    printf("  a=(%.17g, %.17g) bits: re=0x%016llx im=0x%016llx\n",
        a_re,a_im,*(uint64_t*)&a_re,*(uint64_t*)&a_im);
    printf("  b=(%.17g, %.17g) bits: re=0x%016llx im=0x%016llx\n",
        b_re,b_im,*(uint64_t*)&b_re,*(uint64_t*)&b_im);

    // y = s * b
    double tr = b_re*s_re - b_im*s_im;
    double ti = b_re*s_im + b_im*s_re;
    printf("  b*s = (%.17g, %.17g)\n",tr,ti);
    printf("  f[0] = a+b*s = (%.17g, %.17g)\n",a_re+tr,a_im+ti);
    printf("  f[128] = a-b*s = (%.17g, %.17g)\n",a_re-tr,a_im-ti);

    return 0;
}

#include <stdio.h>
#include <stdint.h>
int main(){
    double f[4]={1,2,3,4};
    double gm_re[1024],gm_im[1024];
    FILE *fr=fopen("DOC/gm_tab_re.hex","r"),*fi=fopen("DOC/gm_tab_im.hex","r");
    for(int i=0;i<1024;i++){uint64_t r,im;fscanf(fr,"%llx",&r);fscanf(fi,"%llx",&im);gm_re[i]=*(double*)&r;gm_im[i]=*(double*)&im;}
    fclose(fr);fclose(fi);
    int hn=2,t=2;
    for(int u=1,m=2;u<2;u++,m<<=1){
        int ht=t>>1,hm=m>>1;
        for(int i1=0,j1=0;i1<hm;i1++,j1+=t){
            int j2=j1+ht;
            double sr=gm_re[m+i1],si=gm_im[m+i1];
            printf("  twiddle[%d]: (%.17g, %.17g)\n",m+i1,sr,si);
            for(int j=j1;j<j2;j++){
                double xr=f[j],xi=f[j+hn],yr=f[j+ht],yi=f[j+ht+hn];
                double tr=yr*sr-yi*si,ti=yr*si+yi*sr;
                f[j]=xr+tr;f[j+hn]=xi+ti;f[j+ht]=xr-tr;f[j+ht+hn]=xi-ti;
            }
        }
        t=ht;
    }
    printf("C FFT N=4:\n");
    for(int i=0;i<2;i++) printf("  f[%d]: re=%.15g im=%.15g\n",i,f[i],f[i+2]);
    printf("  f[2] (conj f[0]): re=%.15g im=%.15g\n",f[0],-f[2]);
    printf("  f[3] (conj f[1]): re=%.15g im=%.15g\n",f[1],-f[3]);
    return 0;
}

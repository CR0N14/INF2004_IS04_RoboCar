#include "demo_profile.h"
/* sin/cos at multiples of 5 degrees, scaled 10000; no runtime trig. */
static int const sn[13]={0,872,1736,2588,3420,4226,5000,5736,6428,7071,7660,8192,8660};
static int const cs[13]={10000,9962,9848,9659,9397,9063,8660,8192,7660,7071,6428,5736,5000};
bool demo_profile_width(uint16_t const d[25],uint16_t *width){
 if(!d||!width||d[12]<2||d[12]>35)return false;
 bool hit[25];
 for(int i=0;i<25;i++){if(d[i]<2||d[i]>400)return false;int a=i<12?12-i:i-12;int depth=(d[i]*cs[a]+5000)/10000;int delta=depth-d[12];hit[i]=delta<=8;}
 int l=12,r=12;while(l>0&&hit[l-1])--l;while(r<24&&hit[r+1])++r;
 if(l==0||r==24||l==r)return false; /* edges must be bracketed */
 int x1=d[l]*sn[12-l],x2=d[r]*sn[r-12];
 unsigned w=(x1+x2+5000)/10000;
 if(w==0||w>127)return false;
 *width=w;return true;
}

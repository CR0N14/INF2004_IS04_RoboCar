#include "demo_profile.h"

#include <stddef.h>

#define FORWARD_DEG     90
#define MAX_RANGE_CM    400u
#define MIN_RANGE_CM    2u
#define MAX_WIDTH_CM    127u    /* width_cm is a 7-bit telemetry field */

/* sin and cos of 0..60 degrees, scaled by 10000: no runtime trig on the M0+. */
static uint16_t const g_sin[DEMO_SCAN_HALF_DEG + 1u] =
{
        0,   175,   349,   523,   698,   872,  1045,  1219,  1392,  1564,
     1736,  1908,  2079,  2250,  2419,  2588,  2756,  2924,  3090,  3256,
     3420,  3584,  3746,  3907,  4067,  4226,  4384,  4540,  4695,  4848,
     5000,  5150,  5299,  5446,  5592,  5736,  5878,  6018,  6157,  6293,
     6428,  6561,  6691,  6820,  6947,  7071,  7193,  7314,  7431,  7547,
     7660,  7771,  7880,  7986,  8090,  8192,  8290,  8387,  8480,  8572,
     8660,
};

static uint16_t const g_cos[DEMO_SCAN_HALF_DEG + 1u] =
{
    10000,  9998,  9994,  9986,  9976,  9962,  9945,  9925,  9903,  9877,
     9848,  9816,  9781,  9744,  9703,  9659,  9613,  9563,  9511,  9455,
     9397,  9336,  9272,  9205,  9135,  9063,  8988,  8910,  8829,  8746,
     8660,  8572,  8480,  8387,  8290,  8192,  8090,  7986,  7880,  7771,
     7660,  7547,  7431,  7314,  7193,  7071,  6947,  6820,  6691,  6561,
     6428,  6293,  6157,  6018,  5878,  5736,  5592,  5446,  5299,  5150,
     5000,
};

typedef struct
{
    demo_range_fn read;
    void         *p_ctx;
    unsigned int  readings;
} scan_t;

typedef struct
{
    unsigned int off_deg;   /* corner angle from straight ahead */
    unsigned int range_cm;  /* range at the last reading on the obstacle */
} edge_t;

static bool
valid(uint16_t range_cm)
{
    return (range_cm >= MIN_RANGE_CM) && (range_cm <= MAX_RANGE_CM);
}

/* One reading, repeated up to twice if it returned no echo, so a single
   dropped echo is not mistaken for the edge of the obstacle. 0 means no echo. */
static uint16_t
sample(scan_t *p_s, int angle_deg)
{
    uint16_t range_cm = 0u;

    for (unsigned int attempt = 0u; attempt < 3u; attempt++)
    {
        range_cm = p_s->read(p_s->p_ctx, (unsigned int)angle_deg);
        p_s->readings++;
        if (valid(range_cm))
        {
            return range_cm;
        }
    }
    return 0u;
}

/* A flat face square to the car has the same depth at every angle. A reading
   with no echo is background, not obstacle. */
static bool
is_hit(uint16_t range_cm, unsigned int off_deg, uint16_t ahead_cm)
{
    unsigned int depth_cm;

    if ((0u == range_cm) || (off_deg > DEMO_SCAN_HALF_DEG))
    {
        return false;
    }
    depth_cm = ((range_cm * (unsigned int)g_cos[off_deg]) + 5000u) / 10000u;
    return depth_cm <= (ahead_cm + DEMO_HIT_TOL_CM);
}

/* Median of up to three readings straight ahead; 0 if fewer than two echo. */
static uint16_t
read_ahead(scan_t *p_s)
{
    uint16_t v[3];
    unsigned int n = 0u;

    for (unsigned int i = 0u; i < 3u; i++)
    {
        uint16_t const r = sample(p_s, FORWARD_DEG);

        if (0u != r)
        {
            v[n++] = r;
        }
    }
    if (n < 2u)
    {
        return 0u;
    }
    for (unsigned int i = 1u; i < n; i++)
    {
        uint16_t const key = v[i];
        unsigned int j = i;

        while ((j > 0u) && (v[j - 1u] > key))
        {
            v[j] = v[j - 1u];
            j--;
        }
        v[j] = key;
    }
    return v[(n - 1u) / 2u];
}

/*
 * Walk outward from straight ahead in coarse steps until a reading is no
 * longer on the obstacle, then step back through that gap in fine steps.
 * dir is -1 for the left and +1 for the right.
 */
static bool
find_edge(scan_t *p_s, uint16_t ahead_cm, int dir, edge_t *p_e)
{
    unsigned int hit_off = 0u;
    unsigned int miss_off = 0u;
    unsigned int off;
    unsigned int edge_off;
    uint16_t hit_range = ahead_cm;

    for (off = DEMO_COARSE_STEP_DEG; off <= DEMO_SCAN_HALF_DEG;
         off += DEMO_COARSE_STEP_DEG)
    {
        uint16_t const r = sample(p_s, FORWARD_DEG + (dir * (int)off));

        if (is_hit(r, off, ahead_cm))
        {
            hit_off = off;
            hit_range = r;
        }
        else
        {
            miss_off = off;
            break;
        }
    }
    if (0u == miss_off)
    {
        return false;       /* still on the obstacle at the sweep limit */
    }

    for (off = hit_off + DEMO_FINE_STEP_DEG; off < miss_off;
         off += DEMO_FINE_STEP_DEG)
    {
        uint16_t const r = sample(p_s, FORWARD_DEG + (dir * (int)off));

        if (is_hit(r, off, ahead_cm))
        {
            hit_off = off;
            hit_range = r;
        }
        else
        {
            miss_off = off;
            break;
        }
    }

    /* The corner lies between the last reading on the obstacle and the first
       off it, then back toward the axis by the beam half-angle. */
    edge_off = (hit_off + miss_off) / 2u;
    edge_off = (edge_off > DEMO_BEAM_HALF_DEG) ? (edge_off - DEMO_BEAM_HALF_DEG)
                                               : 0u;
    p_e->off_deg = edge_off;
    p_e->range_cm = hit_range;
    return true;
}

bool
demo_measure_width(demo_range_fn read, void *p_ctx, demo_width_t *p_out)
{
    scan_t scan = { read, p_ctx, 0u };
    edge_t left;
    edge_t right;
    uint16_t ahead_cm;
    unsigned int lateral;
    unsigned int width_cm;

    if ((NULL == read) || (NULL == p_out))
    {
        return false;
    }
    ahead_cm = read_ahead(&scan);
    if ((0u == ahead_cm) || (ahead_cm > DEMO_AHEAD_MAX_CM))
    {
        return false;
    }
    if (!find_edge(&scan, ahead_cm, -1, &left) ||
        !find_edge(&scan, ahead_cm, +1, &right))
    {
        return false;
    }

    lateral = (left.range_cm * (unsigned int)g_sin[left.off_deg]) +
              (right.range_cm * (unsigned int)g_sin[right.off_deg]);
    width_cm = (lateral + 5000u) / 10000u;
    if ((0u == width_cm) || (width_cm > MAX_WIDTH_CM))
    {
        return false;
    }

    p_out->width_cm = (uint16_t)width_cm;
    p_out->ahead_cm = ahead_cm;
    p_out->left_deg = (uint8_t)(FORWARD_DEG - (int)left.off_deg);
    p_out->right_deg = (uint8_t)(FORWARD_DEG + (int)right.off_deg);
    p_out->bearing_deg = (uint8_t)((p_out->left_deg + p_out->right_deg) / 2u);
    p_out->readings = (uint8_t)((scan.readings > 255u) ? 255u : scan.readings);
    return true;
}

/* Demo bypass clearance analysis, retained alongside refined width scan. */

#include "../demo_config.h"
#include "../subsystem1/sm_types.h"
/* sin/cos at multiples of 5 degrees, scaled 10000; no runtime trig. */
static int const sn[13]={0,872,1736,2588,3420,4226,5000,5736,6428,7071,7660,8192,8660};
static int const cs[13]={10000,9962,9848,9659,9397,9063,8660,8192,7660,7071,6428,5736,5000};
#define FAR_CM 400u
static bool profile_valid(uint16_t d){return d>=2u&&d<=FAR_CM;}
bool demo_profile_analyse(uint16_t const d[25],demo_profile_t *p){
 if(!d||!p||!profile_valid(d[12])||d[12]>35u)return false;
 bool hit[25];
 for(int i=0;i<25;i++){
  int a=i<12?12-i:i-12;
  if(!profile_valid(d[i])){hit[i]=false;continue;}
  /* Forward depth of this return; the obstacle face is within 8 cm. */
  int depth=((int)d[i]*cs[a]+5000)/10000;
  hit[i]=depth-(int)d[12]<=8;
 }
 int l=12,r=12;while(l>0&&hit[l-1])--l;while(r<24&&hit[r+1])++r;
 if(l==0||r==24||l==r)return false; /* both edges must be inside the sweep */
 int w=((int)d[l]*sn[12-l]+(int)d[r]*sn[r-12]+5000)/10000-2*(int)DEMO_SONAR_EDGE_CORR_CM;
 if(w<=0||w>127)return false;
 uint16_t lo=FAR_CM,hi=FAR_CM,closest=d[12];
 for(int i=0;i<25;i++){
  uint16_t v=profile_valid(d[i])?d[i]:FAR_CM;
  if(i<l){if(v<lo)lo=v;}
  else if(i>r){if(v<hi)hi=v;}
  else if(v<closest)closest=v;
 }
 p->width_cm=(uint16_t)w;p->closest_cm=closest;
 p->low_edge=(uint8_t)l;p->high_edge=(uint8_t)r;
 p->left_cm=DEMO_SERVO_LEFT_LOW?lo:hi;p->right_cm=DEMO_SERVO_LEFT_LOW?hi:lo;
 return true;
}
uint8_t demo_profile_choose(demo_profile_t const *p){
 if(!p)return SM_AVOID_STOP;
 if(p->left_cm>=DEMO_SIDE_CLEAR_CM&&p->left_cm>=p->right_cm)return SM_AVOID_LEFT; /* tie: left */
 if(p->right_cm>=DEMO_SIDE_CLEAR_CM)return SM_AVOID_RIGHT;
 return SM_AVOID_STOP;
}
#define GRID_LO 30
#define GRID_HI 150
#define CACHE_NONE 0xFFFFu
#define SIDE_STEP_DEG 20
typedef struct {demo_range_fn read;void *ctx;uint16_t cache[GRID_HI-GRID_LO+1];} measure_t;
/* Remember the best reading per angle: a valid echo beats no echo, and the
   nearer of two valid echoes is kept (conservative for clearance). */
static uint16_t cached_read(void *p_ctx,unsigned int angle){
 measure_t *m=(measure_t *)p_ctx;
 uint16_t r=m->read(m->ctx,angle);
 if(angle>=(unsigned)GRID_LO&&angle<=(unsigned)GRID_HI){
  uint16_t *c=&m->cache[angle-GRID_LO];
  if(profile_valid(r)){if(*c==CACHE_NONE||!profile_valid(*c)||r<*c)*c=r;}
  else if(*c==CACHE_NONE)*c=r;
 }
 return r;
}
static uint16_t grid_read(measure_t *m,int angle){
 uint16_t v=m->cache[angle-GRID_LO];
 if(v==CACHE_NONE||!profile_valid(v)){
  /* Read, plus one retry, so a single dropped echo does not hide a near
     side wall. */
  v=cached_read(m,(unsigned)angle);
  if(!profile_valid(v))v=cached_read(m,(unsigned)angle);
 }
 return profile_valid(v)?v:(uint16_t)FAR_CM;
}
bool demo_profile_measure(demo_range_fn read,void *p_ctx,demo_profile_t *p,demo_width_t *p_w){
 measure_t m;demo_width_t w;
 if(!read||!p)return false;
 m.read=read;m.ctx=p_ctx;
 for(int i=0;i<=GRID_HI-GRID_LO;i++)m.cache[i]=CACHE_NONE;
 if(!demo_measure_width(cached_read,&m,&w))return false;
 int low_limit=(int)w.left_deg-(int)DEMO_BEAM_HALF_DEG;   /* beyond the low corner */
 int high_limit=(int)w.right_deg+(int)DEMO_BEAM_HALF_DEG; /* beyond the high corner */
 uint16_t lo=0,hi=0;bool lo_any=false,hi_any=false;
 /* The width scan ends on the high side: read it first (outer limit
    inwards), then the low side, to keep servo swings short. 20-degree
    grid from the sweep limit; cached 10-degree readings are reused. */
 for(int a=GRID_HI;a>high_limit;a-=SIDE_STEP_DEG){uint16_t v=grid_read(&m,a);if(!hi_any||v<hi)hi=v;hi_any=true;}
 for(int a=GRID_LO;a<low_limit;a+=SIDE_STEP_DEG){uint16_t v=grid_read(&m,a);if(!lo_any||v<lo)lo=v;lo_any=true;}
 /* Coarse misses already read beyond each corner count too. */
 for(int a=GRID_LO;a<=GRID_HI;a+=10){
  uint16_t v=m.cache[a-GRID_LO];
  if(v==CACHE_NONE)continue;
  v=profile_valid(v)?v:(uint16_t)FAR_CM;
  if(a<low_limit){if(!lo_any||v<lo)lo=v;lo_any=true;}
  else if(a>high_limit){if(!hi_any||v<hi)hi=v;hi_any=true;}
 }
 uint16_t closest=w.ahead_cm;
 for(int a=w.left_deg;a<=w.right_deg;a++){
  uint16_t v=(a>=GRID_LO&&a<=GRID_HI)?m.cache[a-GRID_LO]:CACHE_NONE;
  if(v!=CACHE_NONE&&profile_valid(v)&&v<closest)closest=v;
 }
 p->width_cm=w.width_cm;p->closest_cm=closest;
 p->low_edge=(uint8_t)(((int)w.left_deg-GRID_LO)/5);
 p->high_edge=(uint8_t)(((int)w.right_deg-GRID_LO+4)/5>24?24:((int)w.right_deg-GRID_LO+4)/5);
 p->left_cm=DEMO_SERVO_LEFT_LOW?lo:hi;p->right_cm=DEMO_SERVO_LEFT_LOW?hi:lo;
 if(p_w)*p_w=w;
 return true;
}
bool demo_profile_width(uint16_t const d[25],uint16_t *width){
 demo_profile_t p;
 if(!width||!demo_profile_analyse(d,&p))return false;
 *width=p.width_cm;return true;
}

/* Exercise the actual demo driver with simulated GPIO edges and clock. */
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#define S5_DRV_ULTRASONIC_H
#define ENCODER_FREE_DEMO 1
typedef unsigned int UW, UINT;
typedef int ER;
typedef void (*FP)(UINT);
typedef struct {unsigned int intatr; FP inthdr;} T_DINT;
#define IO_BANK0_BASE 0x1000u
#define TIMER_TIMERAWL 1u
#define SIO_CPUID_ADDR 2u
#define GPIO_MODE_IN 0
#define GPIO_MODE_OUT 1
#define BOARD_UART0_FREE 1
#define E_NOSPT -1
#define E_OK 0
#define S5_NO_ECHO_CM 511u
#define TA_HLNG 0
#define INTPRI_MAX_EXTINT_PRI 0
#define DI(m) ((m)=0)
#define EI(m) ((void)(m))
static UW clock_us, edges, width, rise_time;
static UINT phase, mode, initial_high;
static FP handler;
static UW in_w(UW a) {if(a==TIMER_TIMERAWL)return clock_us++;if(a==SIO_CPUID_ADDR)return 0;return edges;}
static void out_w(UW a,UW v){(void)a;edges &= ~v;}
static void set_w(UW a,UW v){(void)a;(void)v;}
static int gpio_set_pin(UINT p,UINT m){(void)p;(void)m;return 0;}
static int gpio_set_val(UINT p,UINT v){(void)p;(void)v;return 0;}
static UINT gpio_get_val(UINT p){(void)p;return initial_high;}
static int tk_def_int(UINT n,T_DINT *d){(void)n;handler=d->inthdr;return 0;}
static void EnableInt(UINT n,UINT p){(void)n;(void)p;}
static void tk_dly_tsk(UINT n){
 (void)n;
 if(mode==0 && phase==0){clock_us+=100;rise_time=clock_us;edges=8;handler(13);phase++;}
 else if(mode==0 && phase==1){clock_us=rise_time+width;edges=4;handler(13);phase++;clock_us+=5000;}
 else if(mode==1 && phase==0){edges=12;handler(13);phase++;}
 else clock_us+=1000;
}
#include "../../app_program/subsystem5/drv_ultrasonic.c"
static UINT measure(UINT pulse,UINT scenario){width=pulse;mode=scenario;phase=0;initial_high=0;return s5_ultrasonic_read_cm();}
int main(void){
 assert(s5_ultrasonic_init()==0);
 assert(measure(1740,0)==30); /* Task resumes 5 ms later: width unaffected. */
 assert(measure(58,0)==511);
 assert(measure(116,0)==2);
 assert(measure(23200,0)==400);
 assert(measure(23201,0)==511);
 assert(measure(1740,1)==511); /* Coalesced edges cannot be timed. */
 assert(measure(1740,2)==511); /* No edge timeout. */
 initial_high=1;assert(s5_ultrasonic_read_cm()==511);
 assert(measure(2900,0)==50); /* Recovers after failures. */
 return 0;
}

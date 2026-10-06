#include "i2c_bus_clear.h"
#include <stdio.h>
#define CHECK(x) do { if(!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); return 1; } } while(0)
typedef struct { int scl,sda,stuck_scl,release_after,pulses,stops; unsigned delay; } fake;
static unsigned elapsed;
static void clock_pin(void *p,int release) {
    fake *f=p; if(!f->scl && release)f->pulses++; f->scl=release;
}
static void data_pin(void *p,int release) {
    fake *f=p; if(!f->sda && release && f->scl && !f->stuck_scl)f->stops++;
    f->sda=release;
}
static int read_clock(void *p) { fake *f=p; return f->scl && !f->stuck_scl && elapsed>=f->delay; }
static int read_data(void *p) { fake *f=p; return f->sda && f->pulses>=f->release_after; }
static void delay(unsigned us) { elapsed+=us; }
static int run(fake *f) {
    elapsed=0; i2c_clear_ops o={f,clock_pin,data_pin,read_clock,read_data,delay}; return i2c_clear_bus(&o);
}
int main(void) {
    fake f={1,1,0,0,0,0,0}; CHECK(run(&f)); CHECK(f.stops==1 && f.pulses==1);
    f=(fake){1,1,0,3,0,0,0}; CHECK(run(&f)); CHECK(f.pulses==4 && f.stops==1);
    f=(fake){1,1,0,99,0,0,0}; CHECK(!run(&f)); CHECK(f.pulses==9 && f.stops==0);
    f=(fake){1,1,1,0,0,0,0}; CHECK(!run(&f)); CHECK(f.pulses==0 && elapsed<=510);
    f=(fake){1,1,0,0,0,0,30}; CHECK(run(&f)); CHECK(elapsed>=30 && elapsed<100);
    CHECK(!i2c_clear_bus(0));
    puts("PASS: idle STOP, interrupted read recovery, stuck SDA, stuck SCL, bounded stretch");
    return 0;
}

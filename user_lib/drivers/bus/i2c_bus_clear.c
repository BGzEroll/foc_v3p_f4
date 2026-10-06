#include "i2c_bus_clear.h"
static int wait_clock(const i2c_clear_ops *o) {
    /* Allow bounded clock stretching / slow pull-up, at most 500 us. */
    for(unsigned n=0;n<100;n++) {
        if(o->read_scl(o->context)) return 1;
        o->delay_us(5);
    }
    return o->read_scl(o->context)!=0;
}
int i2c_clear_bus(const i2c_clear_ops *o) {
    if(!o || !o->scl || !o->sda || !o->read_scl || !o->read_sda || !o->delay_us) return 0;
    o->scl(o->context,1); o->sda(o->context,1);
    o->delay_us(5);
    if(!wait_clock(o)) return 0;
    for(unsigned n=0;n<9 && !o->read_sda(o->context);n++) {
        o->scl(o->context,0); o->delay_us(5);
        o->scl(o->context,1);
        if(!wait_clock(o)) return 0;
        o->delay_us(5);
    }
    if(!o->read_sda(o->context)) return 0;
    /* Pull SDA low while SCL is LOW; release SDA only after real SCL HIGH.
     * Avoid accidentally generating START while preparing STOP. */
    o->scl(o->context,0); o->delay_us(5);
    o->sda(o->context,0); o->delay_us(5);
    o->scl(o->context,1);
    if(!wait_clock(o)) { o->sda(o->context,1); return 0; }
    o->delay_us(5);
    o->sda(o->context,1); o->delay_us(5);
    return o->read_scl(o->context) && o->read_sda(o->context);
}

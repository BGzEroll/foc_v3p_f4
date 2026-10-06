#ifndef I2C_BUS_CLEAR_H
#define I2C_BUS_CLEAR_H
#ifdef __cplusplus
extern "C" {
#endif
/* Driving 1 MUST release an open-drain output, never drive push-pull high.
 * This routine is bounded and must run while the I2C peripheral is disabled. */
typedef struct {
    void *context;
    void (*scl)(void *, int);
    void (*sda)(void *, int);
    int (*read_scl)(void *);
    int (*read_sda)(void *);
    void (*delay_us)(unsigned);
} i2c_clear_ops;
int i2c_clear_bus(const i2c_clear_ops *ops);
#ifdef __cplusplus
}
#endif
#endif

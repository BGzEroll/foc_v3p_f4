#ifndef PLL_STARTUP_H
#define PLL_STARTUP_H
#include "bemf_pll.h"
#ifdef __cplusplus
extern "C" {
#endif
enum { PLL_START_IDLE, PLL_START_ALIGN, PLL_START_RAMP, PLL_START_WAIT,
       PLL_START_BLEND, PLL_START_CLOSED, PLL_START_FAULT };
enum { PLL_START_OK, PLL_START_BAD_BUS, PLL_START_NO_LOCK, PLL_START_LOST_LOCK,
       PLL_START_TIMEOUT, PLL_START_STOP, PLL_START_TIMING, PLL_START_HARDWARE, PLL_START_STATIC_DONE };
typedef struct {
    uint32_t state, reason, ticks, stage_ticks, qualified_ticks, closed_ticks;
    uint32_t static_scan;
    float open_angle, open_speed, control_angle, control_speed;
    float target_speed, startup_id, target_id, target_iq;
    float angle_difference, max_phase_a;
} pll_startup;
void pll_startup_init(pll_startup *s);
void pll_startup_begin(pll_startup *s);
void pll_startup_stop(pll_startup *s, unsigned reason);
/* Called once per 50 us interval after the observer. No encoder input exists. */
void pll_startup_step(pll_startup *s, const bemf_pll *p, float bus);
int pll_startup_running(const pll_startup *s);
#ifdef __cplusplus
}
#endif
#endif

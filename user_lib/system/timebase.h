#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdint.h>

namespace timebase
{
    uint32_t now_us();
    bool init();
}

#endif

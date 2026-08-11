#ifndef AS5600_DEV_H
#define AS5600_DEV_H

#include "drivers/as5600.h"

namespace as5600_dev
{
    bool peek_latest(as5600_sample &sample);
    void init();
}

#endif

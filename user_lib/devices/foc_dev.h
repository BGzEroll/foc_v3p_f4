#ifndef FOC_DEV_H
#define FOC_DEV_H

namespace foc
{
    class instance;
    class commissioner;
}

namespace foc_dev
{
    void init();
    foc::instance &get_motor();
    foc::commissioner &get_motor_commissioner();
}

#endif

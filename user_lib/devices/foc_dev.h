#ifndef FOC_DEV_H
#define FOC_DEV_H

#include "SguanFOC/wrapper/sguan_foc_wrapper.h"

namespace foc_dev
{
    SguanFOCWrapper &controller();
    void init();
}

#endif

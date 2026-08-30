#ifndef FOC_DEV_H
#define FOC_DEV_H

#include "SguanFOC/wrapper/sguan_foc_wrapper.h"

namespace foc_dev
{
    sguan_foc_wrapper &controller();
    void init();
}

#endif

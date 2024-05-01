#ifndef BSP_INIT_h
#define BSP_INIT_h

#include "bsp_dwt.h"

/**
 * @brief bsp????????,?????????bsp??,?????????????????
 *        ???????????,???RobotoInit()??
 *
 * @note ?????????CAN?????????????????,???????
 */
//
void BSPInit()
{
    DWT_Init(168);
}

#endif // BSP_INIT_h
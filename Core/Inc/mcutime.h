/*
 * mcutime.h
 *
 *  Created on: Nov 4, 2024
 *      Author: cmos
 */

#ifndef INC_MCUTIME_H_
#define INC_MCUTIME_H_

#include <cstdio>
#include "main.h"

int64_t micros(){
    return static_cast<int64_t>(HAL_GetTick())*1000;
}
int64_t millis(){
    return static_cast<int64_t>(HAL_GetTick());
}

#endif /* INC_MCUTIME_H_ */

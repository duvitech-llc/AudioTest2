/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UTILS_H
#define __UTILS_H

#include <ctype.h>
#include <stdio.h>
#include <stdint.h>

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

void hexdump(const char *head, const uint8_t *data, int len);

uint32_t ZipCRC(CRC_HandleTypeDef hcrc, uint8_t * pData, uint32_t size);

#endif // __UTILS_H

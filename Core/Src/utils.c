#include <ctype.h>
#include <stdio.h>
#include <stdint.h>
#include "utils.h"

void hexdump(const char *head, const uint8_t *data, int len)
{
    int i;
    char str[16];

    printf("%s [%d] =>\n    0000  %02x ", head, len, data[0]);
    str[0] = isprint(data[0]) ? data[0] : '.';
    for (i=1; i<len; i++) {
        if ((i&0x0f) == 0) {
            printf(" |%16s|", str);
            printf("\n    %04d  ", i);
        } else if((i&0x07) == 0) {
            printf(" ");
        }
        printf("%02x ", data[i]);
        str[i&0x0f] = isprint(data[i]) ? data[i] : '.';
    }
    if ((i &= 0x0f) != 0) {
        if (i < 8) printf(" ");
        for (; i<16; i++) {
            printf("   ");
            str[i] = ' ';
        }
        printf(" |%16s|", str);
    }
    printf("\n");
}

uint32_t ZipCRC(CRC_HandleTypeDef hcrc, uint8_t * pData, uint32_t size)
{
	uint32_t uwCRCValue;
	uwCRCValue = 0xFFFFFFFF; // Initial state
	uwCRCValue = HAL_CRC_Calculate(&hcrc, (uint32_t *)pData, size); // byte at a time
	uwCRCValue = ~uwCRCValue;
	return uwCRCValue;
}

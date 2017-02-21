#ifndef OLED_H
#define OLED_H

#include "r_cg_macrodriver.h"
#include "r_cg_serial.h"

#define oled_address 0x78

typedef struct
{
	uint8_t length;
	uint8_t witgh;
}OledObject;

extern OledObject oled;

void oled_init(void);
void oled_showchar(uint8_t x, uint8_t y, uint8_t tr);
void oled_fill(uint8_t filldata);

#endif
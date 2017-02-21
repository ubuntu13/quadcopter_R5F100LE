#ifndef SHOW_H
#define SHOW_H

#include "r_cg_macrodriver.h"
#include "r_cg_userdefine.h"
#include "oled.h"

void oled_showstring(uint8_t x, uint8_t y, const char *p);
void main_interface(void);
void oled_showmessage(uint8_t pit, uint8_t rol, uint8_t alt);
void oled_showmode(uint8_t temp);

#endif
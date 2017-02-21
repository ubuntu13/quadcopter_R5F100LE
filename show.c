#include "show.h"

void oled_showstring(uint8_t x, uint8_t y, const char *p)
{
	while( *p!='\0' )
    {
        oled_showchar(x, y, *p);
        x+=8;
        p++;
    }
}

void oled_showmessage(uint8_t pit, uint8_t rol, uint8_t alt)
{
	oled_showchar(32, 0, (uint8_t)('0' + pit%10));
	oled_showchar(96, 0, (uint8_t)('0' + rol%10));
	oled_showchar(32, 2, (uint8_t)('0' + alt%10));
}

void oled_showmode(uint8_t temp)
{
	if(temp)
	{
		oled_showstring(64, 4, "mode1");
	}
	else
	{
		oled_showstring(64, 4, "mode0");
	}
}

void main_interface(void)
{
	oled_showstring(0 , 0, "pit=");
	oled_showstring(64, 0, "rol=");
	oled_showstring(0 , 2, "alt=");
	oled_showstring(64, 2, "pos=");
}
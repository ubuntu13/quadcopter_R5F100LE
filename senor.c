#include "senor.h"

void senor_trig(void)
{
	uint8_t i = 50;

	P3.0 = 1;
	while(i--);
	P3.0 = 0;
}

#include "key.h"

uint8_t key_scan(void)
{
	static key_up = 1;
	uint16_t i = 1000;
	
	if(key_up && (key1==0||key2==0))
	{
		while(i--);
		
		key_up = 0;
		
		if(key1==0)
		{
			return 1;
		}
		else if(key2==0)
		{
			return 2;
		}
	}
	else if( (key1==1) && (key2==1) )
	{
		key_up=1;
	}
	return 0;
}
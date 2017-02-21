#include "oled.h"
#include "oled_font.h"

OledObject oled;

const uint8_t *ascii;

extern uint8_t IICA0_Receiveend;
extern uint8_t IICA0_Sendend;

void oled_writecommand(uint8_t command)
{
	uint8_t tx_buf[2] = {0x00, 0x00};
	
	tx_buf[1] = command;
	R_IICA0_Master_Send(oled_address, tx_buf, 2, 200);
	while(IICA0_Sendend == 0);
	IICA0_Sendend = 0;
}

void oled_writedata(uint8_t data)
{
	uint8_t tx_buf[2] = {0x40, 0x00};
	
	tx_buf[1] = data;
	R_IICA0_Master_Send(oled_address, tx_buf, 2, 200);
	while(IICA0_Sendend == 0);
	IICA0_Sendend = 0;
}

void oled_init(void)
{
	oled_writecommand(0xAE);   	//display off
	oled_writecommand(0x20);	//Set Memory Addressing Mode	
	oled_writecommand(0x10);	//00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	oled_writecommand(0xb0);	//Set Page Start Address for Page Addressing Mode,0-7
	oled_writecommand(0xc8);	//Set COM Output Scan Direction
	oled_writecommand(0x00);	//---set low column address
	oled_writecommand(0x10);	//---set high column address
	oled_writecommand(0x40);	//--set start line address
	oled_writecommand(0x81);	//--set contrast control register
	oled_writecommand(0x7f);
	oled_writecommand(0xa1);	//--set segment re-map 0 to 127
	oled_writecommand(0xa6);	//--set normal display
	oled_writecommand(0xa8);	//--set multiplex ratio(1 to 64)
	oled_writecommand(0x3F);	//
	oled_writecommand(0xa4);	//0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	oled_writecommand(0xd3);	//-set display offset
	oled_writecommand(0x00);	//-not offset
	oled_writecommand(0xd5);	//--set display clock divide ratio/oscillator frequency
	oled_writecommand(0xf0);	//--set divide ratio
	oled_writecommand(0xd9);	//--set pre-charge period
	oled_writecommand(0x22); 	//
	oled_writecommand(0xda);	//--set com pins hardware configuration
	oled_writecommand(0x12);
	oled_writecommand(0xdb);	//--set vcomh
	oled_writecommand(0x20);	//0x20,0.77xVcc
	oled_writecommand(0x8d);	//--set DC-DC enable
	oled_writecommand(0x14);	//
	oled_writecommand(0xaf);	//--turn on oled panel 
	
	oled_fill(0);
}

void oled_setposition(uint8_t x, uint8_t y)
{
	oled_writecommand( (uint8_t)(0xb0+y) );
  	oled_writecommand( (uint8_t)( ((x&0xf0)>>4)|0x10 ) );
  	oled_writecommand( (uint8_t)( (x&0x0f)|0x01 ) ) ;
}

void oled_showchar(uint8_t x, uint8_t y, uint8_t tr)
 {
   uint8_t i = 0;
   
   tr -= 32;
   
   oled_setposition(x, y);
   ascii = &Str16[tr][0];
   for(i=0; i<8; i++)
   {
      oled_writedata(*ascii++);
   }
   
   oled_setposition( x, (uint8_t)(y+1) );
   ascii = &Str16[tr][8];
   for(i=0;i<8;i++)
   {
      oled_writedata(*ascii++);
   }
 }

void oled_fill(uint8_t filldata)
{
	uint8_t m, n;
	for(m=0; m<8; m++)
	{
		oled_writecommand((uint8_t)(0xb0 + m) );		//page0-page1
		oled_writecommand(0x00);		//low column start address
		oled_writecommand(0x10);		//high column start address
		for(n=0; n<128; n++)
		{
			oled_writedata(filldata);
		}
	}
}



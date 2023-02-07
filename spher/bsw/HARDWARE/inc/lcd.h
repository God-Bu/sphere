#ifndef __LCD_H
#define __LCD_H

#include "stm32f4xx.h"

void LCD_Init(void); 
void clear_screen(void);
void clear_screen_half(void);
void Transfer_command(u8 data1);
void Transfer_data(u8 data1);
void Delay_LCD(int i);
void Delay_LCD1(int i);
void lcd_address(u8 page,u8 column);
void display_128x64(u8 *dp);
void display_graphic_16x16(u8 page,u8 column,u8 *dp);
void display_graphic_5x8(u8 page,u8 column,u8 *dp);
void send_command_to_ROM(u8 datu);
static u8 get_data_from_ROM(void);
void get_and_write_16x16(u32 fontaddr,u8 page,u8 column);
void get_and_write_8x16(u32 fontaddr,u8 page,u8 column);
void get_and_write_5x8(u32 fontaddr,u8 page,u8 column);
void display_GB2312_string(u8 page,u8 column,u8 *text);
void display_string_5x8(u8 page,u8 column,u8 *text);

#endif


/**************************************************/
/*                                                */
/*         LCD模块型号为JLX2864G-086（含字库）     */
/*                                                */
/**************************************************/
#include "gpio.h"
#include "lcd.h"

/*=========延时=====================*/
void Delay_LCD(int i)
{
	int j,k;
	for(j=0;j<i;j++)
		for(k=0;k<400;k++);
}

/*=========延时=====================*/
void Delay_LCD1(int i)
{
	int j,k;
	for(j=0;j<i;j++)
		for(k=0;k<3;k++); 
} 

void LCD_Init(void) 
{
	lcd_reset()=0;                 //Reset the chip when reset=0
	Delay_LCD(100);
	lcd_reset()=1;
	Delay_LCD(100);
	Transfer_command(0xe2);  /*软复位*/
	Delay_LCD(5);
  Transfer_command(0x2c);  /*升压步聚 1*/
	Delay_LCD(50);
	Transfer_command(0x2e);  /*升压步聚 2*/
	Delay_LCD(50);
	Transfer_command(0x2f);  /*升压步聚 3*/
	Delay_LCD(5);
	Transfer_command(0x23);  /*粗调对比度，可设置范围 20～27*/ 
	Transfer_command(0x81);  /*微调对比度*/ 
	Transfer_command(0x28);  /*微调对比度的值，可设置范围 0～63*/
	Transfer_command(0xa2);  /*1/9 偏压比（bias）*/
	Transfer_command(0xc8);  /*行扫描顺序：从上到下*/
	Transfer_command(0xa0);  /*列扫描顺序：从左到右*/
	Transfer_command(0xaf);  /*开显示*/

  clear_screen();	
}  

void clear_screen(void)
{
	unsigned char i,j;
	for(i=0;i<9;i++)
	{
		Transfer_command(0xb0+i);
		Transfer_command(0x10);
		Transfer_command(0x00);
		for(j=0;j<132;j++)
		{
			Transfer_data(0x00);
		}
	}
} 

void clear_screen_half(void)
{
	unsigned char i,j;
	for(i=4;i<9;i++)
	{
		Transfer_command(0xb0+i);
		Transfer_command(0x10);
		Transfer_command(0x00);
		for(j=0;j<132;j++)
		{
			Transfer_data(0x00);
		}
	}
} 

/*=======写指令========*/
void Transfer_command(u8 data1)
{
  u8 i;
  lcd_cs()=0;
  lcd_rs()=0;
  for(i=0;i<8;i++)
	{
		lcd_clk()=0;
		if(data1&0x80)
			lcd_data()=1;
		else
			lcd_data()=0;
//		Delay_LCD1(2);
		lcd_clk()=1;
//		Delay_LCD1(2);
		data1<<=1;
	}
	lcd_cs()=1;
}

/*--------写数据------------*/
void Transfer_data(u8 data1)
{
  u8 i;
  lcd_cs()=0;
  lcd_rs()=1;
  for(i=0;i<8;i++)
	{
		lcd_clk()=0;
		if(data1&0x80)
			lcd_data()=1;
		else
			lcd_data()=0;
		lcd_clk()=1;
		data1<<=1;
	}
	lcd_cs()=1;
}

void lcd_address(u8 page,u8 column)
{
  column=column-0x01;
  Transfer_command(0xb0+page-1);   //设置页地址，每 8 行为一页，全屏共 64 行，被分成 8 页
  Transfer_command(0x10+(column>>4&0x0f)); //设置列地址的高 4 位
  Transfer_command(column&0x0f); //设置列地址的低 4 位 
}

//显示 128x64 点阵图像 
void display_128x64(u8 *dp)
{
  u8 i,j;
	for(j=0;j<8;j++)
  {
		lcd_address(j+1,1);
		for (i=0;i<128;i++)
		{ 
			Transfer_data(*dp);     //写数据到 LCD,每写完一个 8 位的数据后列地址自动加 1 
			dp++;
		}
	}
}

//显示 16x16 点阵图像、汉字、生僻字或 16x16 点阵的其他图标
void display_graphic_16x16(u8 page,u8 column,u8 *dp)
{  
	u8 i,j;
  for(j=0;j<2;j++)
  {
		lcd_address(page+j,column);
		for (i=0;i<16;i++)
		{
			Transfer_data(*dp);     //写数据到 LCD,每写完一个 8 位的数据后列地址自动加 1 
			dp++;
		}
	}
}

//显示 5X8 点阵图像、ASCII, 或 5x8 点阵的自造字符、其他图标
void display_graphic_5x8(u8 page,u8 column,u8 *dp) 
{
	u8 i;
	lcd_address(page,column);
	for (i=0;i<6;i++) 
	{
		Transfer_data(*dp);
		dp++;
	}
}  

//送指令到晶联讯字库 IC 
void send_command_to_ROM(u8 datu)
{
	u8 i;
	for(i=0;i<8;i++ )
	{
		lcd_sclk()=0;
		Delay_LCD1(10);
		if(datu&0x80)
			lcd_si()=1;
		else
			lcd_si()=0;
		datu = datu<<1;
		lcd_sclk()=1;
		Delay_LCD1(10);
	}
}  	

//从晶联讯字库 IC 中取汉字或字符数据（1 个字节）
static u8 get_data_from_ROM(void) 
{  
	u8 i;
  u8 ret_data=0;
  for(i=0;i<8;i++)
  {
		lcd_so()=1;
		lcd_sclk()=0;
		//delay_us(1);
		ret_data=ret_data<<1;
		if(lcd_so())
			ret_data=ret_data+1;
		else
			ret_data=ret_data+0;
		lcd_sclk()=1;
		//delay_us(1);
	}
	return(ret_data);
}

//从指定地址读出数据写到液晶屏指定（page,column)座标中
void get_and_write_16x16(u32 fontaddr,u8 page,u8 column)
{  
	u8 i,j,disp_data;
  lcd_scs() = 0;
  send_command_to_ROM(0x03);
  send_command_to_ROM((fontaddr&0xff0000)>>16);  //地址的高 8 位,共 24 位
  send_command_to_ROM((fontaddr&0xff00)>>8);   //地址的中 8 位,共 24 位
  send_command_to_ROM(fontaddr&0xff);     //地址的低 8 位,共 24 位
  for(j=0;j<2;j++)
  {
		lcd_address(page+j,column);
		for(i=0;i<16;i++)
		{ 
			disp_data=get_data_from_ROM();
			Transfer_data(disp_data); //写数据到 LCD,每写完 1 字节的数据后列地址自动加 1
		}
	}
	lcd_scs()=1;
}

//从指定地址读出数据写到液晶屏指定（page,column)座标中 
void get_and_write_8x16(u32 fontaddr,u8 page,u8 column) 
{
  u8 i,j,disp_data;
  lcd_scs()=0;
  send_command_to_ROM(0x03);
  send_command_to_ROM((fontaddr&0xff0000)>>16);  //地址的高 8 位,共 24 位 
	send_command_to_ROM((fontaddr&0xff00)>>8);   //地址的中 8 位,共 24 位 
	send_command_to_ROM(fontaddr&0xff);     //地址的低 8 位,共 24 位 
	for(j=0;j<2;j++) 
	{ 
		lcd_address(page+j,column); 
		for(i=0;i<8;i++)
		{  
			disp_data=get_data_from_ROM();
			Transfer_data(disp_data); //写数据到 LCD,每写完 1 字节的数据后列地址自动加 1   
		}
	}
	lcd_scs()=1;
}

//从指定地址读出数据写到液晶屏指定（page,column)座标中
void get_and_write_5x8(u32 fontaddr,u8 page,u8 column)
{
  u8 i,disp_data;
  lcd_scs()=0;
  send_command_to_ROM(0x03);
  send_command_to_ROM((fontaddr&0xff0000)>>16);  //地址的高 8 位,共 24 位
  send_command_to_ROM((fontaddr&0xff00)>>8);   //地址的中 8 位,共 24 位
  send_command_to_ROM(fontaddr&0xff);     //地址的低 8 位,共 24 位
  lcd_address(page,column);
  for(i=0; i<5; i++ )
  {
		disp_data=get_data_from_ROM();
		Transfer_data(disp_data); //写数据到 LCD,每写完 1 字节的数据后列地址自动加 1
	}
	lcd_scs()=1;
}

//****************************************************************   
u32  fontaddr=0;
void display_GB2312_string(u8 page,u8 column,u8 *text)
{
  u8 i=0;
	while((text[i]>0x00))
	{
		if(((text[i]>=0xb0) &&(text[i]<=0xf7))&&(text[i+1]>=0xa1)) 
		{
			//国标简体（GB2312）汉字在晶联讯字库 IC 中的地址由以下公式来计算： 
			//Address = ((MSB - 0xB0) * 94 + (LSB - 0xA1)+ 846)*32+ BaseAdd;BaseAdd=0
			//由于担心 8 位单片机有乘法溢出问题，所以分三部取地址  
			fontaddr = (text[i]- 0xb0)*94;
			fontaddr += (text[i+1]-0xa1)+846;
			fontaddr = (u32)(fontaddr*32);  
			get_and_write_16x16(fontaddr,page,column);  //从指定地址读出数据写到液晶屏指定（page,column)座标中  
			i+=2; 
			column+=16;
		}
		else if(((text[i]>=0xa1) &&(text[i]<=0xa3))&&(text[i+1]>=0xa1))
		{  
			//国标简体（GB2312）15x16 点的字符在晶联讯字库 IC 中的地址由以下公式来计算：    
			//Address = ((MSB - 0xa1) * 94 + (LSB - 0xA1))*32+ BaseAdd;BaseAdd=0
			//由于担心 8 位单片机有乘法溢出问题，所以分三部取地址    
			fontaddr = (text[i]- 0xa1)*94; 
			fontaddr += (text[i+1]-0xa1);
			fontaddr = (u32)(fontaddr*32);
      get_and_write_16x16(fontaddr,page,column);  //从指定地址读出数据写到液晶屏指定（page,column)座标中 
			i+=2;
			column+=16;
		}
		else if((text[i]>=0x20) &&(text[i]<=0x7e))  
		{
			fontaddr = (text[i]- 0x20);
			fontaddr = (unsigned long)(fontaddr*16);
			fontaddr = (unsigned long)(fontaddr+0x3cf80);     
			get_and_write_8x16(fontaddr,page,column);  //从指定地址读出数据写到液晶屏指定（page,column)座标中
			i+=1;
			column+=8;
		}
		else
			i++;
	}
} 

void display_string_5x8(u8 page,u8 column,u8 *text)
{
  u8 i= 0;
  while((text[i]>0x00))
	{
		if((text[i]>=0x20) &&(text[i]<=0x7e))
		{ 
			fontaddr = (text[i]- 0x20);
			fontaddr = (u32)(fontaddr*8);
			fontaddr = (u32)(fontaddr+0x3bfc0);      
			get_and_write_5x8(fontaddr,page,column);  //从指定地址读出数据写到液晶屏指定（page,column)座标中  
			i+=1;
			column+=6;
		}
		else
			i++;
	}
} 

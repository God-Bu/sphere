/**************************************************/
/*                                                */
/*         LCDģ���ͺ�ΪJLX2864G-086�����ֿ⣩     */
/*                                                */
/**************************************************/
#include "gpio.h"
#include "lcd.h"

/*=========��ʱ=====================*/
void Delay_LCD(int i)
{
	int j,k;
	for(j=0;j<i;j++)
		for(k=0;k<400;k++);
}

/*=========��ʱ=====================*/
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
	Transfer_command(0xe2);  /*��λ*/
	Delay_LCD(5);
  Transfer_command(0x2c);  /*��ѹ���� 1*/
	Delay_LCD(50);
	Transfer_command(0x2e);  /*��ѹ���� 2*/
	Delay_LCD(50);
	Transfer_command(0x2f);  /*��ѹ���� 3*/
	Delay_LCD(5);
	Transfer_command(0x23);  /*�ֵ��Աȶȣ������÷�Χ 20��27*/ 
	Transfer_command(0x81);  /*΢���Աȶ�*/ 
	Transfer_command(0x28);  /*΢���Աȶȵ�ֵ�������÷�Χ 0��63*/
	Transfer_command(0xa2);  /*1/9 ƫѹ�ȣ�bias��*/
	Transfer_command(0xc8);  /*��ɨ��˳�򣺴��ϵ���*/
	Transfer_command(0xa0);  /*��ɨ��˳�򣺴�����*/
	Transfer_command(0xaf);  /*����ʾ*/

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

/*=======дָ��========*/
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

/*--------д����------------*/
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
  Transfer_command(0xb0+page-1);   //����ҳ��ַ��ÿ 8 ��Ϊһҳ��ȫ���� 64 �У����ֳ� 8 ҳ
  Transfer_command(0x10+(column>>4&0x0f)); //�����е�ַ�ĸ� 4 λ
  Transfer_command(column&0x0f); //�����е�ַ�ĵ� 4 λ 
}

//��ʾ 128x64 ����ͼ�� 
void display_128x64(u8 *dp)
{
  u8 i,j;
	for(j=0;j<8;j++)
  {
		lcd_address(j+1,1);
		for (i=0;i<128;i++)
		{ 
			Transfer_data(*dp);     //д���ݵ� LCD,ÿд��һ�� 8 λ�����ݺ��е�ַ�Զ��� 1 
			dp++;
		}
	}
}

//��ʾ 16x16 ����ͼ�񡢺��֡���Ƨ�ֻ� 16x16 ���������ͼ��
void display_graphic_16x16(u8 page,u8 column,u8 *dp)
{  
	u8 i,j;
  for(j=0;j<2;j++)
  {
		lcd_address(page+j,column);
		for (i=0;i<16;i++)
		{
			Transfer_data(*dp);     //д���ݵ� LCD,ÿд��һ�� 8 λ�����ݺ��е�ַ�Զ��� 1 
			dp++;
		}
	}
}

//��ʾ 5X8 ����ͼ��ASCII, �� 5x8 ����������ַ�������ͼ��
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

//��ָ�����Ѷ�ֿ� IC 
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

//�Ӿ���Ѷ�ֿ� IC ��ȡ���ֻ��ַ����ݣ�1 ���ֽڣ�
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

//��ָ����ַ��������д��Һ����ָ����page,column)������
void get_and_write_16x16(u32 fontaddr,u8 page,u8 column)
{  
	u8 i,j,disp_data;
  lcd_scs() = 0;
  send_command_to_ROM(0x03);
  send_command_to_ROM((fontaddr&0xff0000)>>16);  //��ַ�ĸ� 8 λ,�� 24 λ
  send_command_to_ROM((fontaddr&0xff00)>>8);   //��ַ���� 8 λ,�� 24 λ
  send_command_to_ROM(fontaddr&0xff);     //��ַ�ĵ� 8 λ,�� 24 λ
  for(j=0;j<2;j++)
  {
		lcd_address(page+j,column);
		for(i=0;i<16;i++)
		{ 
			disp_data=get_data_from_ROM();
			Transfer_data(disp_data); //д���ݵ� LCD,ÿд�� 1 �ֽڵ����ݺ��е�ַ�Զ��� 1
		}
	}
	lcd_scs()=1;
}

//��ָ����ַ��������д��Һ����ָ����page,column)������ 
void get_and_write_8x16(u32 fontaddr,u8 page,u8 column) 
{
  u8 i,j,disp_data;
  lcd_scs()=0;
  send_command_to_ROM(0x03);
  send_command_to_ROM((fontaddr&0xff0000)>>16);  //��ַ�ĸ� 8 λ,�� 24 λ 
	send_command_to_ROM((fontaddr&0xff00)>>8);   //��ַ���� 8 λ,�� 24 λ 
	send_command_to_ROM(fontaddr&0xff);     //��ַ�ĵ� 8 λ,�� 24 λ 
	for(j=0;j<2;j++) 
	{ 
		lcd_address(page+j,column); 
		for(i=0;i<8;i++)
		{  
			disp_data=get_data_from_ROM();
			Transfer_data(disp_data); //д���ݵ� LCD,ÿд�� 1 �ֽڵ����ݺ��е�ַ�Զ��� 1   
		}
	}
	lcd_scs()=1;
}

//��ָ����ַ��������д��Һ����ָ����page,column)������
void get_and_write_5x8(u32 fontaddr,u8 page,u8 column)
{
  u8 i,disp_data;
  lcd_scs()=0;
  send_command_to_ROM(0x03);
  send_command_to_ROM((fontaddr&0xff0000)>>16);  //��ַ�ĸ� 8 λ,�� 24 λ
  send_command_to_ROM((fontaddr&0xff00)>>8);   //��ַ���� 8 λ,�� 24 λ
  send_command_to_ROM(fontaddr&0xff);     //��ַ�ĵ� 8 λ,�� 24 λ
  lcd_address(page,column);
  for(i=0; i<5; i++ )
  {
		disp_data=get_data_from_ROM();
		Transfer_data(disp_data); //д���ݵ� LCD,ÿд�� 1 �ֽڵ����ݺ��е�ַ�Զ��� 1
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
			//������壨GB2312�������ھ���Ѷ�ֿ� IC �еĵ�ַ�����¹�ʽ�����㣺 
			//Address = ((MSB - 0xB0) * 94 + (LSB - 0xA1)+ 846)*32+ BaseAdd;BaseAdd=0
			//���ڵ��� 8 λ��Ƭ���г˷�������⣬���Է�����ȡ��ַ  
			fontaddr = (text[i]- 0xb0)*94;
			fontaddr += (text[i+1]-0xa1)+846;
			fontaddr = (u32)(fontaddr*32);  
			get_and_write_16x16(fontaddr,page,column);  //��ָ����ַ��������д��Һ����ָ����page,column)������  
			i+=2; 
			column+=16;
		}
		else if(((text[i]>=0xa1) &&(text[i]<=0xa3))&&(text[i+1]>=0xa1))
		{  
			//������壨GB2312��15x16 ����ַ��ھ���Ѷ�ֿ� IC �еĵ�ַ�����¹�ʽ�����㣺    
			//Address = ((MSB - 0xa1) * 94 + (LSB - 0xA1))*32+ BaseAdd;BaseAdd=0
			//���ڵ��� 8 λ��Ƭ���г˷�������⣬���Է�����ȡ��ַ    
			fontaddr = (text[i]- 0xa1)*94; 
			fontaddr += (text[i+1]-0xa1);
			fontaddr = (u32)(fontaddr*32);
      get_and_write_16x16(fontaddr,page,column);  //��ָ����ַ��������д��Һ����ָ����page,column)������ 
			i+=2;
			column+=16;
		}
		else if((text[i]>=0x20) &&(text[i]<=0x7e))  
		{
			fontaddr = (text[i]- 0x20);
			fontaddr = (unsigned long)(fontaddr*16);
			fontaddr = (unsigned long)(fontaddr+0x3cf80);     
			get_and_write_8x16(fontaddr,page,column);  //��ָ����ַ��������д��Һ����ָ����page,column)������
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
			get_and_write_5x8(fontaddr,page,column);  //��ָ����ַ��������д��Һ����ָ����page,column)������  
			i+=1;
			column+=6;
		}
		else
			i++;
	}
} 

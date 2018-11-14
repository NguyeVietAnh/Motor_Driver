#include "stm32f1xx_hal_lcd1602.h"


void LCD_Enable(void)
{
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
HAL_Delay(1);
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);	
HAL_Delay(1);	
}

void LCD_Send4Bit(unsigned char Data)
{
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,Data&0x01);
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,(Data>>1)&0x01);
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,(Data>>2)&0x01);
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,(Data>>3)&0x01);	
}

void LCD_SendCommand(unsigned char command)
{
	LCD_Send4Bit(command >>4);/* Gui 4 bit cao */
	LCD_Enable();
	LCD_Send4Bit(command);	/* Gui 4 bit thap*/
	LCD_Enable();
}
///* USER CODE END 0 */
void LCD_Clear(void)
{
 	LCD_SendCommand(0x01);  
  HAL_Delay(1);	
}

void LCD_Init(void)
{
	LCD_Send4Bit(0x00);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
	LCD_Send4Bit(0x03);
	LCD_Enable();
	LCD_Enable();
	LCD_Enable();
	LCD_Send4Bit(0x02);
	LCD_Enable();
	LCD_SendCommand(0x28); // giao thuc 4 bit, hien thi 2 hang, ki tu 5x8
	LCD_SendCommand(0x0C); // cho phep hien thi man hinh
	LCD_SendCommand(0x06); // tang ID, khong dich khung hinh
	LCD_SendCommand(0x01); // xoa toan bo khung hinh
}

void LCD_Gotoxy(unsigned char x, unsigned char y)
{
	unsigned char address;
  	if(!y)address=(0x80+x);
  	else address=(0xC0+x);
  	LCD_SendCommand(address);

}

void LCD_PutChar(unsigned char Data)
{
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
 	LCD_SendCommand(Data);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
}

void LCD_Puts(char *s)
{
   	while (*s){
      	LCD_PutChar(*s);
     	s++;
   	}
}
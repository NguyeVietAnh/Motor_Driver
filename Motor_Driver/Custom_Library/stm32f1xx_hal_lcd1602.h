/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F1XX_HAL_LCD1602_H
#define __STM32F1XX_HAL_LCD1602_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
 extern "C" {
#endif 

void LCD_Enable(void);
void LCD_Send4Bit(unsigned char Data);
void LCD_SendCommand(unsigned char command);
void LCD_Clear(void);
void LCD_Init(void);
void LCD_Gotoxy(unsigned char x, unsigned char y);
void LCD_PutChar(unsigned char Data);
void LCD_Puts(char *s);

#ifdef __cplusplus
}
#endif


#endif /* __LCD_H */



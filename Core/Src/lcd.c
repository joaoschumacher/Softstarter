/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdio.h"


/*  PINO D7 DO LCD -> Pino PA6
 *  PINO D6 DO LCD -> Pino PA7
 *  PINO D5 DO LCD -> Pino PB6
 *  PINO D4 DO LCD -> Pino PC7
 *
 *  PINO LCD_EN ->  Pino PB4
 *  PINO LCD_RS ->  Pino PB5
 *
 */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "main.h"
#include "lcd.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


void tempo_us(uint16_t t)
{
	for(uint16_t i=0; i<=t*15; i++)
	{
		__NOP();
		__NOP();
	}
}

//**** PULSO DE ENABLE  *****

void enable() {
	HAL_GPIO_WritePin(LCD_EN_GPIO_Port,LCD_EN_Pin,1);
	tempo_us(4);
	HAL_GPIO_WritePin(LCD_EN_GPIO_Port,LCD_EN_Pin,0);
}

//****  ESCRITA NO LCD EM 04 BITS  ******

void lcd_write(unsigned char c) {
	unsigned char d = (c >> 4) & 0x0F;
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, (d & 0x01));
	HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, (d & 0x02));
	HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, (d & 0x04));
	HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, (d & 0x08));
	enable();

	d = (c & 0x0F);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, (d & 0x01));
	HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, (d & 0x02));
	HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, (d & 0x04));
	HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, (d & 0x08));
	enable();

	tempo_us(40);
}

//**** ESCRITA DE UMA STRING (NOME)  ****

void lcd_puts(const char *s) {
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port,LCD_RS_Pin,1);
	while (*s) {
		lcd_write(*s++);
	}
}

//**** ESCRITA DE UMA VARI�VEL NUM�RICA DECIMAL (N�mero)  ****

void lcd_printd(uint16_t c){
	char buf[10];
	sprintf(buf,"%d",c);
	lcd_puts(buf);
}

/*
void lcd_printf(float c){
	char buf[10];
	sprintf(buf,"%f",c);
	lcd_puts(buf);
}
*/

//*** ESCRITA DE UM CARACTERE  ****

void lcd_putc(char c) {
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port,LCD_RS_Pin,1);
	lcd_write(c);
}

//**** POSICIONAMENTO DO CURSOR  *****

void lcd_goto(unsigned char pos) {
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port,LCD_RS_Pin,0);
	lcd_write(pos);
}

//*** LIMPEZA DA MEM�RIA DDRAM (Tela) *****

void lcd_clear() {
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port,LCD_RS_Pin,0);
	lcd_write(0x01);
	HAL_Delay(2);
}
void lcd_init() {

	HAL_Delay(15);
	HAL_GPIO_WritePin(D4_GPIO_Port,
			D4_Pin | D5_Pin | D6_Pin | D7_Pin | LCD_RS_Pin | LCD_EN_Pin,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin | D5_Pin, GPIO_PIN_SET);

	enable();
	HAL_Delay(5);

	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin | D5_Pin, GPIO_PIN_SET);
	enable();
	tempo_us(100);

	HAL_GPIO_WritePin(D4_GPIO_Port,
			D4_Pin | D5_Pin | D6_Pin | D7_Pin | LCD_RS_Pin | LCD_EN_Pin,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, 1);
	enable();
	tempo_us(40);

	lcd_write(0x28);
	lcd_write(0x06);
	lcd_write(0x0C);
	lcd_clear();
}


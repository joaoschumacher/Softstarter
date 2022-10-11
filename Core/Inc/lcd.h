/*
 *	LCD interface header file
*/

#ifndef LCD_H_
#define LCD_H_

extern void tempo_us(uint16_t t);

extern void enable();

/* write a byte to the LCD in 4 bit mode */

extern void lcd_write(unsigned char c);

extern void lcd_printd(uint16_t c);

extern void lcd_printf(float c);

/* Clear and home the LCD */

extern void lcd_clear(void);

/* write a string of characters to the LCD */

extern void lcd_puts(const char *s);

/* Go to the specified position */

extern void lcd_goto(unsigned char pos);
	
/* intialize the LCD - call before anything else */

extern void lcd_init(void);

extern void lcd_putch(char c);

extern void lcd_putc(char c);


/*	Set the cursor position */

//#define	lcd_cursor(x)	lcd_write(((x)&0x7F)|0x80)

#endif

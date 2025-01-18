
#ifndef _LCD_H_
#define _LCD_H_

void lcd_init();

void lcd_set_latitude(float x);
void lcd_set_longitude(float y);
void lcd_set_accel(float x, float y, float z);
void lcd_set_gyro(float x, float y, float z);

#endif
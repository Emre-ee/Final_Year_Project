/*
  size is 1*16
  if do not need to read busy, then you can tie R/W=ground
  ground = pin 1    Vss
  power  = pin 2    Vdd   +3.3V or +5V depending on the device
  ground = pin 3    Vlc   grounded for highest contrast
  PE1    = pin 4    RS    (1 for data, 0 for control/status)
  ground = pin 5    R/W   (1 for read, 0 for write)
  PE0    = pin 6    E     (enable)
  -    	 = pin 7    DB0   (8-bit data)
  -	     = pin 8    DB1
  -   	 = pin 9    DB2
  -      = pin 10   DB3
  PD0    = pin 11   DB4
  PD1    = pin 12   DB5
  PD2    = pin 13   DB6
  PD3    = pin 14   DB7
16 characters are configured as 1 row of 16
addr  00 01 02 03 04 05 ... 0F
*/

#include<stdint.h>
// Initialize LCD
// Inputs: none
// Outputs: none
void LCD_Init(void);

// Output a character to the LCD
// Inputs: letter is ASCII character, 0 to 0x7F
// Outputs: none
void LCD_OutChar(char letter);

// Clear the LCD
// Inputs: none
// Outputs: none
void LCD_Clear(void);

//------------LCD_OutString------------
// Output String (NULL termination)
// Input: pt   -> pointer to a NULL-terminated string to be transferred
//        line -> which line you want to write your data         
// Output: none
void LCD_OutString(char *pt,unsigned char line);

//-----------------------LCD_OutUDec-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1-10 digits with no space before or after
void LCD_OutUDec(uint32_t n);

//--------------------------LCD_OutUHex----------------------------
// Output a 32-bit number in unsigned hexadecimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1 to 8 digits with no space before or after
void LCD_OutUHex(uint32_t number);

void lcd_setcursor(void);
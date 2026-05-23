#ifndef _LCD_H
#define _LCD_H

#include <stdint.h>
#include <driver/i2c.h>

#define LCD_I2C_PORT    I2C_NUM_0

// HD44780 command set
#define LCD_CLEARDISPLAY    0x01
#define LCD_RETURNHOME      0x02
#define LCD_ENTRYMODESET    0x04
#define LCD_DISPLAYCONTROL  0x08
#define LCD_CURSORSHIFT     0x10
#define LCD_FUNCTIONSET     0x20
#define LCD_SETCGRAMADDR    0x40
#define LCD_SETDDRAMADDR    0x80

// Entry mode flags
#define LCD_ENTRYLEFT           0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// Display control flags
#define LCD_DISPLAYON   0x04
#define LCD_CURSORON    0x02
#define LCD_BLINKON     0x01

// Cursor/display shift flags
#define LCD_DISPLAYMOVE 0x08
#define LCD_MOVERIGHT   0x04
#define LCD_MOVELEFT    0x00

// Function set flags
#define LCD_2LINE    0x08
#define LCD_1LINE    0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS  0x00

// PCF8574 backpack pin mapping
#define LCD_RS_BIT  0x01  // Register select
#define LCD_RW_BIT  0x02  // Read/Write (always 0 — write only)
#define LCD_EN_BIT  0x04  // Enable
#define LCD_BL_BIT  0x08  // Backlight

class LCD
{
public:
    LCD();
    ~LCD();

    static void scan(int sda_pin, int scl_pin);
    bool setup(int sda_pin, int scl_pin, uint8_t address = 0x27, uint8_t cols = 16, uint8_t rows = 2);

    // Basic display control
    void clear();
    void home();
    void setCursor(uint8_t col, uint8_t row);

    // Display on/off
    void display();
    void noDisplay();

    // Cursor style
    void cursor();
    void noCursor();
    void blink();
    void noBlink();

    // Scrolling
    void scrollDisplayLeft();
    void scrollDisplayRight();

    // Text direction
    void leftToRight();
    void rightToLeft();
    void autoscroll();
    void noAutoscroll();

    // Backlight
    void backlight();
    void noBacklight();
    bool getBacklight();

    // Custom characters (8 slots, 0–7)
    void createChar(uint8_t location, uint8_t charmap[]);

    // Writing
    void writeCommand(uint8_t command);
    void print(char character);
    void printStr(const char* str);

private:
    void writeI2C(uint8_t data);
    void pulseEnable(uint8_t data);
    void sendNibble(uint8_t nibble, bool isData);
    void sendByte(uint8_t value, bool isData);

    uint8_t _address;
    uint8_t _cols;
    uint8_t _rows;
    uint8_t _displayfunction;
    uint8_t _displaycontrol;
    uint8_t _displaymode;
    uint8_t _backlightval;
};

#endif // _LCD_H

#include "lcd.h"
#include <stdio.h>
#include <esp_rom_sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static void i2c_install(int sda_pin, int scl_pin)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = { .clk_speed = 100000 },
        .clk_flags = 0,
    };
    i2c_param_config(LCD_I2C_PORT, &conf);
    i2c_driver_install(LCD_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

void LCD::scan(int sda_pin, int scl_pin)
{
    i2c_install(sda_pin, scl_pin);
    printf("[LCD scan] Scanning addresses 0x08 to 0x77...\n");
    bool found = false;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(LCD_I2C_PORT, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            printf("[LCD scan] Device found at 0x%02X\n", addr);
            found = true;
        }
    }
    if (!found)
        printf("[LCD scan] No devices found — check wiring and power\n");
    i2c_driver_delete(LCD_I2C_PORT);
}

LCD::LCD()
{
    _address         = 0;
    _cols            = 16;
    _rows            = 2;
    _backlightval    = LCD_BL_BIT;
    _displayfunction = 0;
    _displaycontrol  = 0;
    _displaymode     = 0;
}

LCD::~LCD()
{
    i2c_driver_delete(LCD_I2C_PORT);
}

bool LCD::setup(int sda_pin, int scl_pin, uint8_t address, uint8_t cols, uint8_t rows)
{
    _address      = address;
    _cols         = cols;
    _rows         = rows;
    _backlightval = LCD_BL_BIT;

    i2c_install(sda_pin, scl_pin);

    // Verify device is present
    i2c_cmd_handle_t probe = i2c_cmd_link_create();
    i2c_master_start(probe);
    i2c_master_write_byte(probe, (_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(probe);
    esp_err_t ret = i2c_master_cmd_begin(LCD_I2C_PORT, probe, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(probe);

    if (ret != ESP_OK) {
        printf("[LCD] Device not found at 0x%02X — check wiring\n", address);
        return false;
    }

    // HD44780 datasheet: wait >40ms after power-on
    vTaskDelay(pdMS_TO_TICKS(50));

    _displayfunction = LCD_5x8DOTS;
    _displayfunction |= (_rows > 1) ? LCD_2LINE : LCD_1LINE;

    // 4-bit init sequence (HD44780 datasheet fig. 24, pg. 46)
    sendNibble(0x03, false); vTaskDelay(pdMS_TO_TICKS(5));
    sendNibble(0x03, false); vTaskDelay(pdMS_TO_TICKS(5));
    sendNibble(0x03, false); esp_rom_delay_us(150);
    sendNibble(0x02, false); // switch to 4-bit mode

    writeCommand(LCD_FUNCTIONSET | _displayfunction);

    _displaycontrol = LCD_DISPLAYON;
    writeCommand(LCD_DISPLAYCONTROL | _displaycontrol);

    clear();

    _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    writeCommand(LCD_ENTRYMODESET | _displaymode);

    home();

    printf("[LCD] Initialized at 0x%02X (%dx%d)\n", address, cols, rows);
    return true;
}

// ── Low-level I2C + pulse ─────────────────────────────────────────────────────

void LCD::writeI2C(uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(LCD_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
}

void LCD::pulseEnable(uint8_t data)
{
    writeI2C(data & ~LCD_EN_BIT);
    esp_rom_delay_us(1);
    writeI2C(data | LCD_EN_BIT);
    esp_rom_delay_us(1);
    writeI2C(data & ~LCD_EN_BIT);
    esp_rom_delay_us(50);
}

void LCD::sendNibble(uint8_t nibble, bool isData)
{
    uint8_t data = (nibble << 4) | _backlightval;
    if (isData)
        data |= LCD_RS_BIT;
    pulseEnable(data);
}

void LCD::sendByte(uint8_t value, bool isData)
{
    sendNibble(value >> 4,   isData);
    sendNibble(value & 0x0F, isData);
}

void LCD::writeCommand(uint8_t command)
{
    sendByte(command, false);
    esp_rom_delay_us(37); // most commands settle in <37µs
}

// ── Basic display control ─────────────────────────────────────────────────────

void LCD::clear()
{
    writeCommand(LCD_CLEARDISPLAY);
    esp_rom_delay_us(1500); // clear takes up to 1.52ms
}

void LCD::home()
{
    writeCommand(LCD_RETURNHOME);
    esp_rom_delay_us(1500);
}

void LCD::setCursor(uint8_t col, uint8_t row)
{
    static const uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
    if (row >= _rows) row = _rows - 1;
    if (col >= _cols) col = _cols - 1;
    writeCommand(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

// ── Display on/off ────────────────────────────────────────────────────────────

void LCD::display()
{
    _displaycontrol |= LCD_DISPLAYON;
    writeCommand(LCD_DISPLAYCONTROL | _displaycontrol);
}

void LCD::noDisplay()
{
    _displaycontrol &= ~LCD_DISPLAYON;
    writeCommand(LCD_DISPLAYCONTROL | _displaycontrol);
}

// ── Cursor style ──────────────────────────────────────────────────────────────

void LCD::cursor()
{
    _displaycontrol |= LCD_CURSORON;
    writeCommand(LCD_DISPLAYCONTROL | _displaycontrol);
}

void LCD::noCursor()
{
    _displaycontrol &= ~LCD_CURSORON;
    writeCommand(LCD_DISPLAYCONTROL | _displaycontrol);
}

void LCD::blink()
{
    _displaycontrol |= LCD_BLINKON;
    writeCommand(LCD_DISPLAYCONTROL | _displaycontrol);
}

void LCD::noBlink()
{
    _displaycontrol &= ~LCD_BLINKON;
    writeCommand(LCD_DISPLAYCONTROL | _displaycontrol);
}

// ── Scrolling ─────────────────────────────────────────────────────────────────

void LCD::scrollDisplayLeft()
{
    writeCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

void LCD::scrollDisplayRight()
{
    writeCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// ── Text direction ────────────────────────────────────────────────────────────

void LCD::leftToRight()
{
    _displaymode |= LCD_ENTRYLEFT;
    writeCommand(LCD_ENTRYMODESET | _displaymode);
}

void LCD::rightToLeft()
{
    _displaymode &= ~LCD_ENTRYLEFT;
    writeCommand(LCD_ENTRYMODESET | _displaymode);
}

void LCD::autoscroll()
{
    _displaymode |= LCD_ENTRYSHIFTINCREMENT;
    writeCommand(LCD_ENTRYMODESET | _displaymode);
}

void LCD::noAutoscroll()
{
    _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
    writeCommand(LCD_ENTRYMODESET | _displaymode);
}

// ── Backlight ─────────────────────────────────────────────────────────────────

void LCD::backlight()
{
    _backlightval = LCD_BL_BIT;
    writeI2C(_backlightval);
}

void LCD::noBacklight()
{
    _backlightval = 0x00;
    writeI2C(_backlightval);
}

bool LCD::getBacklight()
{
    return _backlightval == LCD_BL_BIT;
}

// ── Custom characters ─────────────────────────────────────────────────────────

void LCD::createChar(uint8_t location, uint8_t charmap[])
{
    location &= 0x07; // only 8 CGRAM slots (0–7)
    writeCommand(LCD_SETCGRAMADDR | (location << 3));
    for (int i = 0; i < 8; i++)
        sendByte(charmap[i], true);
}

// ── Writing ───────────────────────────────────────────────────────────────────

void LCD::print(char character)
{
    sendByte((uint8_t)character, true);
}

void LCD::printStr(const char* str)
{
    clear();
    uint8_t row = 0;
    setCursor(0, 0);
    while (*str) {
        if (*str == '\n') {
            row++;
            setCursor(0, row);
        } else {
            print(*str);
        }
        str++;
    }
}

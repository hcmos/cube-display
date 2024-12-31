/*
 * LCDDisplay.h
 *
 *  Created on: Dec 22, 2024
 *      Author: cmos
 */

#ifndef INC_LCD_DISPLAY_H_
#define INC_LCD_DISPLAY_H_

#include "i2c.h"
#include <unordered_map>
#include <string>

class LCD_Display {
public:
    LCD_Display(I2C_HandleTypeDef &hi2c):i2c(hi2c){};

    int setup();
    void cycle();
    void write(const uint32_t key);

private:
    I2C_HandleTypeDef &i2c;

    void write_command(const uint8_t command);
    void write_data(const uint8_t data);

    void contrast_max();
    void init_oled();

    uint32_t current_key = 0;
    bool is_written = false;

    std::unordered_map<uint32_t, std::string> lib = {
        {0x000, "EMEGENCY ON!"},
        {0x001, "EMEGENCY OFF!"},
        {0x002, "CNT ON"},
        {0x003, "CNT OFF"},
        {0x501, "TELEOP"},
        {0x502, "AUTO"},
        {0x510, "RESTART"}
    };
};

#endif /* INC_LCD_DISPLAY_H_ */

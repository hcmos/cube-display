/*
 * LCDDisplay.cpp
 *
 *  Created on: Dec 22, 2024
 *      Author: cmos
 */

#include <lcd_display.h>
#include "utils.h"

using namespace utils;
using namespace std;

#define OLED_ADRS (0x3C << 1) // OLEDのI2Cアドレス（7ビット左詰め）
#define DISPLAY_ON 0x0F
#define CLEAR_DISPLAY 0x01
#define RETURN_HOME 0x02

int LCD_Display::setup(){
    init_oled();
    contrast_max();
    is_written = true;
    return 0;
}
void LCD_Display::cycle(){
    if(!is_written){
        write_command(CLEAR_DISPLAY);

        const uint32_t second_digit = (current_key >> 4) & 0xF;

        string category = {0x20};
        switch(second_digit){
            case 0:
                category = {0xB7, 0xB6, 0xDD};
                break;
            case 1:
                category = {0xBE, 0xB2, 0xB7, 0xF1, 0xAE};
                break;
            default:
                break;
        }
        if(((current_key >> 8) & 0xF) == 0x0){    // 000番台であれば緊急として扱う
            category = {0xB7, 0xDD, 0xB7, 0xAD, 0xB3};
        }

        for (int i = 0; i < category.length(); ++i){
            const char c = category[i];
            write_data(c);
        }

        write_command(0x20 + 0x80); // 2行目の先頭に移動

        if(lib.find(current_key) == lib.end()) return;
        const string name = lib[current_key];
        for (int i = 0; i < name.length(); ++i){
            const char c = name[i];
            write_data(c);
        }

        is_written = true;
    }
}

void LCD_Display::write(const uint32_t key){
    this->current_key = key;
    is_written = false;
}

void LCD_Display::write_data(const uint8_t data){
    uint8_t buffer[2] = {0x40, data}; // コントロールバイト 0x40 + データ
    HAL_I2C_Master_Transmit(&i2c, OLED_ADRS, buffer, 2, HAL_MAX_DELAY);
    HAL_Delay(1);
}
void LCD_Display::write_command(const uint8_t command){
    uint8_t buffer[2] = {0x00, command}; // コントロールバイト 0x00 + コマンド
    HAL_I2C_Master_Transmit(&i2c, OLED_ADRS, buffer, 2, HAL_MAX_DELAY);
    HAL_Delay(10);
}

void LCD_Display::contrast_max(){
    write_command(0x2A); // RE=1
    write_command(0x79); // SD=1
    write_command(0x81); // コントラスト設定
    write_command(0xFF); // 輝度MAX
    write_command(0x78); // SDを0に戻す
    write_command(0x28); // 28=ノーマル
    HAL_Delay(100);
}
void LCD_Display::init_oled(){
    HAL_Delay(100);
    write_command(CLEAR_DISPLAY);
    HAL_Delay(20);
    write_command(RETURN_HOME);
    HAL_Delay(2);
    write_command(DISPLAY_ON);
    HAL_Delay(2);
    write_command(CLEAR_DISPLAY);
    HAL_Delay(20);
}

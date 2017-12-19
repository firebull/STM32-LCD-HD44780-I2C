/**
 * Copyright Nikita Bulaev 2017
 *
 * STM32 HAL libriary for LCD display based on HITACHI HD44780U chip.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
    Here are some cyrillic symbols that you can use in your code

    uint8_t symD[8]   = { 0x07, 0x09, 0x09, 0x09, 0x09, 0x1F, 0x11 }; // Д
    uint8_t symZH[8]  = { 0x11, 0x15, 0x15, 0x0E, 0x15, 0x15, 0x11 }; // Ж
    uint8_t symI[8]   = { 0x11, 0x11, 0x13, 0x15, 0x19, 0x11, 0x11 }; // И
    uint8_t symL[8]   = { 0x0F, 0x09, 0x09, 0x09, 0x09, 0x11, 0x11 }; // Л
    uint8_t symP[8]   = { 0x1F, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11 }; // П
    uint8_t symSHi[8] = { 0x10, 0x15, 0x15, 0x15, 0x15, 0x1F, 0x03 }; // Щ
    uint8_t symJU[8]  = { 0x12, 0x15, 0x15, 0x1D, 0x15, 0x15, 0x12 }; // Ю
    uint8_t symJA[8]  = { 0x0F, 0x11, 0x11, 0x0F, 0x05, 0x09, 0x11 }; // Я


 */

#include "stdlib.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lcd_hd44780_i2c.h"

uint8_t lcdCommandBuffer[6] = {0x00};

static LCDParams lcdParams;

static bool lcdWriteByte(uint8_t rsRwBits, uint8_t * data);

/**
 * @brief  Turn display on and init it params
 * @note   We gonna make init steps according to datasheep page 46.
 *         There are 4 steps to turn 4-bits mode on,
 *         then we send initial params.
 * @param  hi2c    I2C struct to which display is connected
 * @param  address Display I2C 7-bit address
 * @param  lines   Number of lines of display
 * @param  columns Number of colums
 * @return         true if success
 */
bool lcdInit(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t lines, uint8_t columns) {

    TickType_t xLastWakeTime;

    uint8_t lcdData = LCD_BIT_5x8DOTS;

    lcdParams.hi2c      = hi2c;
    lcdParams.address   = address << 1;
    lcdParams.lines     = lines;
    lcdParams.columns   = columns;
    lcdParams.backlight = LCD_BIT_BACKIGHT_ON;

    lcdCommandBuffer[0] = LCD_BIT_E | (0x03 << 4);
    lcdCommandBuffer[1] = lcdCommandBuffer[0];
    lcdCommandBuffer[2] = (0x03 << 4);

    /* First 3 steps of init cycles. They are the same. */
    for (uint8_t i = 0; i < 3; ++i) {
        if (HAL_I2C_Master_Transmit_DMA(lcdParams.hi2c, lcdParams.address, (uint8_t*)lcdCommandBuffer, 3) != HAL_OK) {
            return false;
        }

        xLastWakeTime = xTaskGetTickCount();

        while (HAL_I2C_GetState(lcdParams.hi2c) != HAL_I2C_STATE_READY) {
            vTaskDelay(1);
        }

        if (i == 2) {
            // For the last cycle delay is less then 1 ms (100us by datasheet)
            vTaskDelayUntil(&xLastWakeTime, (TickType_t)1);
        } else {
            // For first 2 cycles delay is less then 5ms (4100us by datasheet)
            vTaskDelayUntil(&xLastWakeTime, (TickType_t)5);
        }
    }

    /* Lets turn to 4-bit at least */
    lcdCommandBuffer[0] = LCD_BIT_BACKIGHT_ON | LCD_BIT_E | (LCD_MODE_4BITS << 4);
    lcdCommandBuffer[1] = lcdCommandBuffer[0];
    lcdCommandBuffer[2] = LCD_BIT_BACKIGHT_ON | (LCD_MODE_4BITS << 4);

    if (HAL_I2C_Master_Transmit_DMA(lcdParams.hi2c, lcdParams.address, (uint8_t*)lcdCommandBuffer, 3) != HAL_OK) {
        return false;
    }

    while (HAL_I2C_GetState(lcdParams.hi2c) != HAL_I2C_STATE_READY) {
        vTaskDelay(1);
    }

    /* Lets set display params */
    /* First of all lets set display size */
    lcdData |= LCD_MODE_4BITS;

    if (lcdParams.lines > 1) {
        lcdData |= LCD_BIT_2LINE;
    }

    lcdWriteByte((uint8_t)0x00, &lcdData);  // TODO: Make 5x10 dots font usable for some 1-line display

    /* Now lets set display, cursor and blink all on */
    lcdDisplayOn();

    /* Set cursor moving to the right */
    lcdCursorDirToRight();

    /* Clear display and Set cursor at Home */
    lcdDisplayClear();
    lcdCursorHome();

    return true;
}

/**
 * @brief  Send command to display
 * @param  command  One of listed in LCDCommands enum
 * @param  action   LCD_PARAM_SET or LCD_PARAM_UNSET
 * @return          true if success
 */
bool lcdCommand(LCDCommands command, LCDParamsActions action) {
    uint8_t lcdData = 0x00;

    /* First of all lest store the command */
    switch (action) {
        case LCD_PARAM_SET:
            switch (command) {
                case LCD_DISPLAY:
                    lcdParams.modeBits |=  LCD_BIT_DISPLAY_ON;
                    break;

                case LCD_CURSOR:
                    lcdParams.modeBits |= LCD_BIT_CURSOR_ON;
                    break;

                case LCD_CURSOR_BLINK:
                    lcdParams.modeBits |= LCD_BIT_BLINK_ON;
                    break;

                case LCD_CLEAR:
                    lcdData = LCD_BIT_DISP_CLEAR;

                    if (lcdWriteByte((uint8_t)0x00, &lcdData) == false) {
                        return false;
                    } else {
                        vTaskDelay(2);
                        return true;
                    }

                case LCD_CURSOR_HOME:
                    lcdData = LCD_BIT_CURSOR_HOME;

                    if (lcdWriteByte((uint8_t)0x00, &lcdData) == false) {
                        return false;
                    } else {
                        vTaskDelay(2);
                        return true;
                    }

                case LCD_CURSOR_DIR_RIGHT:
                    lcdParams.entryBits |= LCD_BIT_CURSOR_DIR_RIGHT;
                    break;

                case LCD_CURSOR_DIR_LEFT:
                    lcdParams.entryBits |= LCD_BIT_CURSOR_DIR_LEFT;
                    break;

                case LCD_DISPLAY_SHIFT:
                    lcdParams.entryBits |= LCD_BIT_DISPLAY_SHIFT;
                    break;

                default:
                    return false;
            }

            break;

        case LCD_PARAM_UNSET:
            switch (command) {
                case LCD_DISPLAY:
                    lcdParams.modeBits &= ~LCD_BIT_DISPLAY_ON;
                    break;

                case LCD_CURSOR:
                    lcdParams.modeBits &= ~LCD_BIT_CURSOR_ON;
                    break;

                case LCD_CURSOR_BLINK:
                    lcdParams.modeBits &= ~LCD_BIT_BLINK_ON;
                    break;

                case LCD_CURSOR_DIR_RIGHT:
                    lcdParams.entryBits &= ~LCD_BIT_CURSOR_DIR_RIGHT;
                    break;

                case LCD_CURSOR_DIR_LEFT:
                    lcdParams.entryBits &= ~LCD_BIT_CURSOR_DIR_LEFT;
                    break;

                case LCD_DISPLAY_SHIFT:
                    lcdParams.entryBits &= ~LCD_BIT_DISPLAY_SHIFT;
                    break;

                default:
                    return false;
            }

            break;

        default:
            return false;
    }

    /* Now lets send the command */
    switch (command) {
        case LCD_DISPLAY:
        case LCD_CURSOR:
        case LCD_CURSOR_BLINK:
            lcdData = LCD_BIT_DISPLAY_CONTROL | lcdParams.modeBits;
            break;

        case LCD_CURSOR_DIR_RIGHT:
        case LCD_CURSOR_DIR_LEFT:
        case LCD_DISPLAY_SHIFT:
            lcdData = LCD_BIT_ENTRY_MODE | lcdParams.entryBits;
            break;

        default:
            break;
    }

    return lcdWriteByte((uint8_t)0x00, &lcdData);
}

/**
 * @brief  Turn display's Backlight On or Off
 * @param  command LCD_BIT_BACKIGHT_ON to turn display On
 *                 LCD_BIT_BACKIGHT_OFF (or 0x00) to turn display Off
 * @return         true if success
 */
bool lcdBacklight(uint8_t command) {
    lcdParams.backlight = command;

    if (HAL_I2C_Master_Transmit_DMA(lcdParams.hi2c, lcdParams.address, &lcdParams.backlight, 1) != HAL_OK) {
        return false;
    }

    while (HAL_I2C_GetState(lcdParams.hi2c) != HAL_I2C_STATE_READY) {
        vTaskDelay(1);
    }

    return true;
}

/**
 * @brief  Set cursor position on the display
 * @param  column counting from 0
 * @param  line   counting from 0
 * @return        true if success
 */
bool lcdSetCursorPosition(uint8_t column, uint8_t line) {
    // We will setup offsets for 4 lines maximum
    static const uint8_t lineOffsets[4] = { 0x00, 0x40, 0x14, 0x54 };

    if ( line >= lcdParams.lines ) {
        line = lcdParams.lines - 1;
    }

    uint8_t lcdCommand = LCD_BIT_SETDDRAMADDR | (column + lineOffsets[line]);

    return lcdWriteByte(0x00, &lcdCommand);
}

/**
 * @brief  Print string from cursor position
 * @param  data   Pointer to string
 * @param  length Number of symbols to print
 * @return        true if success
 */
bool lcdPrintStr(uint8_t * data, uint8_t length) {
    for (uint8_t i = 0; i < length; ++i) {
        if (lcdWriteByte(LCD_BIT_RS, &data[i]) == false) {
            return false;
        }
    }

    return true;
}

/**
 * @brief  Print single char at cursor position
 * @param  data Symbol to print
 * @return      true if success
 */
bool lcdPrintChar(uint8_t data) {
    return lcdWriteByte(LCD_BIT_RS, &data);
}


/**
 * @brief Loading custom Chars to one of the 8 cells in CGRAM
 * @note  You can create your custom chars according to
 *        documentation page 15.
 *        It consists of array of 8 bytes.
 *        Each byte is line of dots. Lower bits are dots.
 * @param  cell     Number of cell from 0 to 7 where to upload
 * @param  charMap  Pointer to Array of dots
 *                  Example: { 0x07, 0x09, 0x09, 0x09, 0x09, 0x1F, 0x11 }
 * @return          true if success
 */
bool lcdLoadCustomChar(uint8_t cell, uint8_t * charMap) {

    // Stop, if trying to load to incorrect cell
    if (cell > 7) {
        return false;
    }

    uint8_t lcdCommand = LCD_BIT_SETCGRAMADDR | (cell << 3);

    if (lcdWriteByte((uint8_t)0x00, &lcdCommand) == false) {
        return false;
    }

    for (uint8_t i = 0; i < 8; ++i) {
        if (lcdWriteByte(LCD_BIT_RS, &charMap[i]) == false) {
            return false;
        }
    }

    return true;
}

/**
 * @brief  Local function to send data to display
 * @param  rsRwBits State of RS and R/W bits
 * @param  data     Pointer to byte to send
 * @return          true if success
 */
static bool lcdWriteByte(uint8_t rsRwBits, uint8_t * data) {

    /* Higher 4 bits*/
    lcdCommandBuffer[0] = rsRwBits | LCD_BIT_E | lcdParams.backlight | (*data & 0xF0);  // Send data and set strobe
    lcdCommandBuffer[1] = lcdCommandBuffer[0];                                          // Strobe turned on
    lcdCommandBuffer[2] = rsRwBits | lcdParams.backlight | (*data & 0xF0);              // Turning strobe off

    /* Lower 4 bits*/
    lcdCommandBuffer[3] = rsRwBits | LCD_BIT_E | lcdParams.backlight | ((*data << 4) & 0xF0);  // Send data and set strobe
    lcdCommandBuffer[4] = lcdCommandBuffer[3];                                                 // Strobe turned on
    lcdCommandBuffer[5] = rsRwBits | lcdParams.backlight | ((*data << 4) & 0xF0);              // Turning strobe off


    if (HAL_I2C_Master_Transmit_DMA(lcdParams.hi2c, lcdParams.address, (uint8_t*)lcdCommandBuffer, 6) != HAL_OK) {
        return false;
    }

    while (HAL_I2C_GetState(lcdParams.hi2c) != HAL_I2C_STATE_READY) {
        vTaskDelay(1);
    }

    return true;
}

Language: **English** [Русский](http://blog.bulki.me/STM32-LCD-HD44780-I2C/README.RU)

# STM32 HAL Libriary for LCD Based on Hitachi HD44780
## What is it for?
I wrote this libriary as I'm sick and tired of other libs based on Arduino lib [LiquidCrystal_I2C](https://github.com/marcoschwartz/LiquidCrystal_I2C). This one is for STM32 but can be adopted for every chip using C99.

Anyway it is written for I2C expander based on [PCF8574](https://www.nxp.com/docs/en/data-sheet/PCF8574_PCF8574A.pdf?pspll=1).

## Why is it better?
There are several reasons:
1. This lib **does not** use any hardware delays. 
2. This lib uses DMA to send data to display
3. This lib uses some tricks using I2C timings
4. This lib is written on clean C99

## How it works?
I will add text later


## How to use it?
This libriary is for STM32 HAL and uses its functions and FreeRTOS. So first of all you need to configure STM32CubeMX project.

#### Turn on I2C
PCF8574 supports only Standart 100kHz mode:

![Screenshot](http://blog.bulki.me/assets/img/stm32-lcd-hitachi/i2c-params.png)

Turn on DMA:

![Screenshot](http://blog.bulki.me/assets/img/stm32-lcd-hitachi/i2c-dma.png)

Turn on interrupts:

![Screenshot](http://blog.bulki.me/assets/img/stm32-lcd-hitachi/i2c-nvic.png)

#### FreeRTOS
Turn on FreeRTOS globally and include ```vTaskDelayUntil()``` function:

![Screenshot](http://blog.bulki.me/assets/img/stm32-lcd-hitachi/freertos_include_params.png)

#### Include correct HAL Libriary
Fix path in *Inc/lcd_hd44780_i2c.h*:
```c
#include "stm32f3xx_hal.h"
```

> You can see correct name in top of your *Src/main.c* created by STM32CubeMX.

#### Connect display
Connect I2C expander to I2C pins you defined in STM32CubeMX and don't forget about PullUP resitors to 3.3V! Better results from 1,5K to 3.3K. It depends on wire length between STM32 and display module. Also better connect power of display module to 5V.

#### Init display
Put such code to your FreeRTOS task. Let us connect to module with address **0x27** and I2C **hi2c1**. We will connect to LCD2004 with 4 lines and 20 columns:

```c
void StartDefaultTask(void const * argument) {
    /* USER CODE BEGIN StartDefaultTask */
    lcdInit(&hi2c1, (uint8_t)0x27, (uint8_t)4, (uint8_t)20);
    
    // Print text and home position 0,0
    lcdPrintStr((uint8_t*)"Hello,", 6);
    
    // Set cursor at zero position of line 3
    lcdSetCursorPosition(0, 2);

    // Print text at cursor position
    lcdPrintStr((uint8_t*)"World!", 6);

    for (;;) {
        vTaskDelay(1000);
    }

    /* USER CODE END StartDefaultTask */
}
```

**Documentation is not finished yet!**

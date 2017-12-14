Language: [English](http://blog.bulki.me/STM32-LCD-HD44780-I2C/) **Русский**

# Библиотека для STM32 HAL для LCD-дисплеев на МК Hitachi HD44780 и I2C
## Зачем она нужна?
Я написал эту библиотеку, т.к. мне надоело всё время натыкаться на готовые библиотеки, которые основаны на Ардуиновской  [LiquidCrystal_I2C](https://github.com/marcoschwartz/LiquidCrystal_I2C). 
Библиотека написана для STM32, но может быть адаптирована для любого МК, который программируется на C99.

Библиотека написана для классического 8-битного расширителя [PCF8574](https://www.nxp.com/docs/en/data-sheet/PCF8574_PCF8574A.pdf?pspll=1).

## Чем она лучше?
Есть несколько причин:
1. Я  **не** использую никакие аппаратные задержки;
2. Я отправляю данные на дисплей через DMA;
3. Я использую некоторые трюки с таймингами, основанные на особенностях I2C и PCF8574;
4. Библиотека написана на чистом C99.

## Как она работает?
Добавлю описание позже

## Как ей пользоваться?
Библиотека использует функции STM32 HAL и FreeRTOS. Поэтому первым делом необходимо добавить параметры в проект STM32CubeMX.

#### Включить I2C
PCF8574 supports only Standart 100kHz mode:
![Screenshot](http://blog.bulki.me/assets/img/stm32-lcd-hitachi/i2c-params.png)

Включить DMA:
![Screenshot](http://blog.bulki.me/assets/img/stm32-lcd-hitachi/i2c-dma.png)

Включить прерывания:
![Screenshot](http://blog.bulki.me/assets/img/stm32-lcd-hitachi/i2c-nvic.png)

#### FreeRTOS
Turn on FreeRTOS globally and include ```vTaskDelayUntil()``` function:
![Screenshot](http://blog.bulki.me/assets/img/stm32-lcd-hitachi/freertos_include_params.png)

#### Подключить HAL Libriary
Пропишите правильную библиотеку HAL в *Inc/lcd_hd44780_i2c.h*:
```c
#include "stm32f3xx_hal.h"
```

> Правильное название можете найти в вашем *Src/main.c*, который был создан STM32CubeMX.

#### Подключить дисплей
Connect I2C expander to I2C pins you defined in STM32CubeMX and don't forget about PullUP resitors to 3.3V! Better results from 1,5K to 3.3K. It depends on wire length between STM32 and display module. Also better connect power of display module to 5V.

#### Инициализация дисплея
Put such code to your FreeRTOS task. Let us connect to module with address **0x27** and I2C **hi2c1**. We will connect to LCD2004 with 4 lines and 20 columns:

```c
void StartDefaultTask(void const * argument) {
    /* USER CODE BEGIN StartDefaultTask */
    lcdInit(&hi2c1, (uint8_t)0x27, (uint8_t)4, (uint8_t)20);
    
    // Вывести текст на начальной позиции 0,0
    lcdPrintStr((uint8_t*)"Hello,", 6);
    
    // Выставить курсор в начало строки №3
    lcdSetCursorPosition(0, 2);

    // Вывести текст с позиции курсора
    lcdPrintStr((uint8_t*)"World!", 6);

    for (;;) {
        vTaskDelay(1000);
    }

    /* USER CODE END StartDefaultTask */
}
```

**Документация ещё не закончена!**

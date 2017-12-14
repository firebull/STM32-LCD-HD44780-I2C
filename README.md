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

## How it works and how to use it?
Read full description and documentation [here](http://blog.bulki.me/STM32-LCD-HD44780-I2C/).

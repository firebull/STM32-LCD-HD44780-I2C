# Библиотека для STM32 HAL Libriary для LCD-дисплеев на МК Hitachi HD44780
## Зачем она нужна?
I wrote this libriary as I'm sick and tired of other libs based on Arduino lib [LiquidCrystal_I2C](https://github.com/marcoschwartz/LiquidCrystal_I2C). This one is for STM32 but can be adopted for every chip using C99.

## Чем оня лучше?
There are several reasons:
1. This lib **does not** use any hardware delays. 
2. This lib using DMA to send data to display
3. This lib uses some tricks using I2C timings
4. This lib is written on clean C99

## Как она работает?

## Как ей пользоваться?

[![Build Status](https://img.shields.io/badge/USEFUL%20ELECTRONICS-YOUTUBE-red)](https://www.youtube.com/channel/UC5zdou2_vz6rjpqMJ23UnQg)
# STM32 Access Multiple Buttons Through One Pin With Software Debounce
A micro-controller can read the status of several buttons using only one pin using ADC with the help of a simple voltage divider circuit. The most effective way of doing digital read is to perform it using DMA, which will let the CPU not get busy while sampling the read voltage level continuously. This piece of code shows uses one of the algorithms used to debouce buttons by the software while trying to read them.

![Circuit Diagram](https://github.com/UsefulElectronics/stm32-soft-debounce-adc-buttons-read/blob/main/Circuit%20diagram/voltage%20divider%20circuit.jpg)
Alteracoes feitas em cima do repositorio base da Navio2 para raspberry, tendo como objetivo a verificacao de tempo de resposta 
dos sensores atraves do novo programa teste para possivel utilizacao no projeto ProVant.

Realizado por:
Iago de Oliveira Silvestre

Universidade Federal de Santa Catarina - Engenharia de Controle e Automacao







Navio 2
=====

Collection of drivers and examples for Navio 2 - autopilot shield for Raspberry Pi.

## Repository structure

### C++

#### Examples

Basic examples showing how to work with Navio's onboard devices using C++.

* AccelGyroMag 
* ADC
* AHRS
* Barometer
* GPS
* LED 2
* RCInput
* Servo

#### Navio 2

C++ drivers for Navio 2's onboard devices and peripheral interfaces.

* MPU9250 SPI
* LSM9DS1 SPI
* U-blox SPI
* MS5611 I2C
* I2C driver
* SPI driver

### Python

Basic examples showing how to work with Navio's onboard devices using Python.

* AccelGyroMag
* ADC
* Barometer
* GPS
* LED
* RCInput
* Servo


### Utilities 

Applications and utilities for Navio.

* 3D IMU visualizer
* U-blox SPI to PTY bridge utility
* U-blox SPI to TCP bridge utility

### Problem with pigpio.h

If you have got error: `fatal error: pigpio.h: No such file or directory`, install `pigpio` by commands:

    sudo apt-get update
    sudo apt-get install pigpio

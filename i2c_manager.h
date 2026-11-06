#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H
#include <Arduino.h>
#include <Wire.h>

//Declarar funciones de gestión de los buses I2C
void iniciabuses();
void escaneobusI2C();
void liberarbusI2C();

//bus 0=Wire, bus 1=Wire1
void writeRegister(int bus, uint8_t sensor, uint8_t reg, uint8_t data);
uint8_t readRegister(int bus, uint8_t sensor, uint8_t reg);
void requestFromBus(int bus, uint8_t sensor, uint8_t bytes);

//Usadas al comienzo, llaman directamente al bus0
void writeRegister(uint8_t sensor, uint8_t reg, uint8_t data);
uint8_t readRegister(uint8_t sensor, uint8_t reg);

#endif // I2C_MANAGER_H
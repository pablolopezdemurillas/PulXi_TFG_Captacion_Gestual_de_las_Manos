#ifndef SENSOR_LOGIC_H
#define SENSOR_LOGIC_H
#include <Arduino.h>

//Funciones de configuración
void seleccionar_dedo(uint8_t dedo);      //Función para determinar en el multiplexor i2c que canal abrimos
void reiniciar_multiplexores();
void configurar_dedo(uint8_t dedo);
void configuracion_sensor(int F);
void configurar_magnetometro_via_bypass(int bus);
void configurar_DLPF(int bus, uint8_t config_byte);

//Funciones de Lectura
void leerfalange(int dedo,int falange);
void leer_magnetometro(int dedo,int falange);
void leerdedo(int dedo, float deltat_real);



//Funciones de impresión por terminal
void imprimir_falange_por_pantalla(int dedo, int falange);              //Funcion auxiliar para imprimir cada falange de cada dedo
void imprimir_dedo_por_pantalla(int dedo);                              //          Imprimir dedo por pantalla
void imprimir_falange_por_teleplot(int dedo, int falange);              //Funcion auxiliar para visualizar las graficas de cada falange de cada dedo
void imprimir_dedo_por_teleplot(int dedo);                              //Función para visualizar las graficas de cada dedo
void imprimir_dedo_para_matlab(int dedo);

void calibrar_magnetometro_mano_completa(const uint8_t *dedos, int num_dedos, int duracion_ms);
void calibrar_sesgo_giroscopio(const uint8_t *dedos, int num_dedos, int duracion_ms);


#endif              //SENSOR_LOGIC_H
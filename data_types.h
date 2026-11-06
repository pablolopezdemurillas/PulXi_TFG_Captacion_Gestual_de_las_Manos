#ifndef DATA_TYPES_H
#define DATA_TYPES_H
#include "filtro_madgwick.h"

//Tabla para guardar los datos de ax,gx,ay,gy,az,gz de cada falange en cada dedo
//                 |___ax___|___ay___|___az___|___gx___|___gy___|___gz___|___mx___|___my___|___mz___|
//Pulgar  _Falange0|________|________|________|________|________|________|________|________|________|
//        _Falange1|________|________|________|________|________|________|________|________|________|
//        _Falange2|________|________|________|________|________|________|________|________|________|
//Meñique _Falange0|________|________|________|________|________|________|________|________|________|
//        _Falange1|________|________|________|________|________|________|________|________|________|
//        _Falange2|________|________|________|________|________|________|________|________|________|

//Anular  _Falange0|________|________|________|________|________|________|________|________|________|
//        _Falange1|________|________|________|________|________|________|________|________|________|
//        _Falange2|________|________|________|________|________|________|________|________|________|

//Corazón _Falange0|________|________|________|________|________|________|________|________|________|
//        _Falange1|________|________|________|________|________|________|________|________|________|
//        _Falange2|________|________|________|________|________|________|________|________|________|

//Indice  _Falange0|________|________|________|________|________|________|________|________|________|
//        _Falange1|________|________|________|________|________|________|________|________|________|
//        _Falange2|________|________|________|________|________|________|________|________|________|

//Palma   _Falange0|________|________|________|________|________|________|________|________|________|


//Ruteo a la salida de los multiplexores
//Canal físico              Dedo ruteado
//------------              ------------
//D0                        Pulgar
//D1                        Indice
//D2                        Corazón
//D3                        Anular
//D4                        Meñique

enum {Ax,Ay,Az,Gx,Gy,Gz,Mx,My,Mz};
enum{Pulgar,Indice, Corazon, Anular, Menique, Palma};
enum {F0,F1,F2};  //las falanges de cada dedo

//Declaración de la matriz
extern float datos[6][3][9];   //dedo|falange|sensor

//-------------------------
//  FILTRO MADGWICK
//-------------------------
extern MadgwickState filtros[6][3];     //estado del filtro, uno por falange
extern float cuaterniones[6][3][4];     //[dedo][falange][w,x,y,z]-->salida
extern SemaphoreHandle_t semaforoCuaterniones;

//Instante (millis()) en el que arranca F0 - se fija una sola vez en setup().
//Mientras (millis() - tiempoArranqueFiltros) sea menor que DURACION_SEMBRADO_MS,
//F0 usa madgwick_9_ejes (con magnetómetro); pasado ese tiempo,
//pasa a madgwick_6_ejes (sin magnetómetro) para siempre.
//F1 y F2 usan madgwick_6_ejes desde el primer ciclo, nunca leen el magnetómetro.
extern unsigned long tiempoArranqueFiltros;
#define DURACION_SEMBRADO_MS 5000UL   //5s: tiempo para que el filtro de Madgwick de 9 ejes converja
#define MEDIR_TIEMPOS 0   // 1 = una línea [CICLO] por ciclo. Si vale 1 se imprime por serie el tiempo de cada bus (para el primer ensayo)
#endif // DATA_TYPES_H
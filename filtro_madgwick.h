#ifndef FILTRO_MADGWICK_H
#define FILTRO_MADGWICK_H

#include <Arduino.h>
//Como el filtro es solo para una falange, hay que instanciar para que podamos usar el filtro en cada una de las falanges
//siguiendo la misma estructura que datos [6][3][9] en data_types.h
typedef struct{
    //Cuaternión estimado: SEq_1=0w, SEq_2=0x, SEq_3=0y, SEq4=0z
    float SEq_1, SEq_2, SEq_3, SEq_4;
    //Dirección de referencia del campo magnético terrestre, recalculada en cada iteración (3.4 Compensación de distorsión)
    float b_x, b_z;
    //Bias estiamdo del giroscopio (3.5)
    float w_bx, w_by, w_bz;
    //Ganancias del filtro (apartado 5)
    float beta;     //ganancia del error de medida del giroscopio
    float zeta;     //ganancia de convergencia del bias del giroscopio
    //Periodo de muestreo Real en segundos (1/f). Debe coincidir con la frecuencia real de madgwick_9_ejes()
    float deltat;
} MadgwickState;

//Llamar una vez por sensor en setup antes de usar el filtro
//si mi tarea corre a 100Hz debo configurar sampleFreqHz a 100.0f para el madgwick_9_ejes()
//gyroMeasErrorRadS: error de medida esperado del giro, en rad/s
//gyroMeasDriftRadS2: deriva del bias esperada del giro, en rad/s^2
void madgwick_init(MadgwickState *st, float sampleFreqHz, float gyroMeasErrorRadS, float gyroMeasDriftRadS2);

//una iteración del filtro MARG (acelerómetro+giroscopio+magnetómetro)
//w_x, w_y, w_z -->rad/s (mi IMU viene en grados/s por lo que hay que convertirlos)
void madgwick_9_ejes(MadgwickState *st, float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z);
//Filtro de 6 ejes
//Madgwick 2010, la variante sin magnetómetro que describe junto a la MARG
void madgwick_6_ejes(MadgwickState *st, float w_x, float w_y, float w_z, float a_x, float a_y, float a_z);

void madgwick_set_deltat(MadgwickState *st, float deltat);

#endif //FILTRO_MADGWICK_H
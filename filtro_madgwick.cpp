/*Fuente: Madgwick, S.O.H. (2010) "An efficient orientation filter for inertial
and inertial/magnetic sensor arrays". Código fuente: Apéndice B (MARG, pág. 30-32
del PDF)*/

#include "filtro_madgwick.h"
#include <math.h>

void madgwick_init(MadgwickState *st, float sampleFreqHz, float gyroMeasErrorRadS, float gyroMeasDriftRadS2)
{
    st->SEq_1 = 1.0f; st->SEq_2 = 0.0f; st->SEq_3 = 0.0f; st->SEq_4 = 0.0f;
    st->b_x = 1.0f;   st->b_z = 0.0f;
    st->w_bx = 0.0f;  st->w_by = 0.0f; st->w_bz = 0.0f;

    // beta y zeta según Ecuaciones (50) y (51) del paper:
    // beta = sqrt(3/4) * error_medio_giroscopio
    // zeta = sqrt(3/4) * deriva_bias_giroscopio
    st->beta = sqrtf(3.0f / 4.0f) * gyroMeasErrorRadS;
    st->zeta = sqrtf(3.0f / 4.0f) * gyroMeasDriftRadS2;

    st->deltat = 1.0f / sampleFreqHz;
}
void madgwick_set_deltat(MadgwickState *st, float deltat){
    st->deltat = deltat;
}

void madgwick_9_ejes(MadgwickState *st, float w_x, float w_y, float w_z, float a_x, float a_y, float a_z,float m_x, float m_y, float m_z)
{
    float norm;
    float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4;
    float f_1, f_2, f_3, f_4, f_5, f_6;
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64;
    float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4;
    float w_err_x, w_err_y, w_err_z;
    float h_x, h_y, h_z;

    // --- Auxiliares para no repetir cálculos (igual que el paper) ---
    float halfSEq_1 = 0.5f * st->SEq_1;
    float halfSEq_2 = 0.5f * st->SEq_2;
    float halfSEq_3 = 0.5f * st->SEq_3;
    float halfSEq_4 = 0.5f * st->SEq_4;
    float twoSEq_1  = 2.0f * st->SEq_1;
    float twoSEq_2  = 2.0f * st->SEq_2;
    float twoSEq_3  = 2.0f * st->SEq_3;
    float twoSEq_4  = 2.0f * st->SEq_4;
    float twob_x = 2.0f * st->b_x;
    float twob_z = 2.0f * st->b_z;
    float twob_xSEq_1 = 2.0f * st->b_x * st->SEq_1;
    float twob_xSEq_2 = 2.0f * st->b_x * st->SEq_2;
    float twob_xSEq_3 = 2.0f * st->b_x * st->SEq_3;
    float twob_xSEq_4 = 2.0f * st->b_x * st->SEq_4;
    float twob_zSEq_1 = 2.0f * st->b_z * st->SEq_1;
    float twob_zSEq_2 = 2.0f * st->b_z * st->SEq_2;
    float twob_zSEq_3 = 2.0f * st->b_z * st->SEq_3;
    float twob_zSEq_4 = 2.0f * st->b_z * st->SEq_4;
    float SEq_1SEq_2;
    float SEq_1SEq_3 = st->SEq_1 * st->SEq_3;
    float SEq_1SEq_4;
    float SEq_2SEq_3;
    float SEq_2SEq_4 = st->SEq_2 * st->SEq_4;
    float SEq_3SEq_4;
    float twom_x = 2.0f * m_x;
    float twom_y = 2.0f * m_y;
    float twom_z = 2.0f * m_z;

    // --- Paso 1: normalizar acelerómetro y magnetómetro ---
    // (el filtro solo usa la dirección de estos vectores, no su magnitud)
    norm = sqrtf(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x /= norm; a_y /= norm; a_z /= norm;

    norm = sqrtf(m_x * m_x + m_y * m_y + m_z * m_z);
    m_x /= norm; m_y /= norm; m_z /= norm;

    // --- Paso 2: función objetivo f y su Jacobiano J ---
    // Ecuaciones (25) y (29) del paper (gravedad y campo magnético)
    f_1 = twoSEq_2 * st->SEq_4 - twoSEq_1 * st->SEq_3 - a_x;
    f_2 = twoSEq_1 * st->SEq_2 + twoSEq_3 * st->SEq_4 - a_y;
    f_3 = 1.0f - twoSEq_2 * st->SEq_2 - twoSEq_3 * st->SEq_3 - a_z;
    f_4 = twob_x * (0.5f - st->SEq_3 * st->SEq_3 - st->SEq_4 * st->SEq_4)
          + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
    f_5 = twob_x * (st->SEq_2 * st->SEq_3 - st->SEq_1 * st->SEq_4)
          + twob_z * (st->SEq_1 * st->SEq_2 + st->SEq_3 * st->SEq_4) - m_y;
    f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4)
          + twob_z * (0.5f - st->SEq_2 * st->SEq_2 - st->SEq_3 * st->SEq_3) - m_z;

    // Ecuación (30) del paper (Jacobiano)
    J_11or24 = twoSEq_3;
    J_12or23 = 2.0f * st->SEq_4;
    J_13or22 = twoSEq_1;
    J_14or21 = twoSEq_2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;
    J_41 = twob_zSEq_3;
    J_42 = twob_zSEq_4;
    J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1;
    J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2;
    J_51 = twob_xSEq_4 - twob_zSEq_2;
    J_52 = twob_xSEq_3 + twob_zSEq_1;
    J_53 = twob_xSEq_2 + twob_zSEq_4;
    J_54 = twob_xSEq_1 - twob_zSEq_3;
    J_61 = twob_xSEq_3;
    J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
    J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
    J_64 = twob_xSEq_2;

    // --- Paso 3: gradiente (J^T * f), Ecuación (20)/(32) del paper ---
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;

    // Normalizar el gradiente (Ecuación (44) del paper: direccion del error)
    norm = sqrtf(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2
                + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot_1 /= norm; SEqHatDot_2 /= norm; SEqHatDot_3 /= norm; SEqHatDot_4 /= norm;

    // --- Paso 4: compensación de bias del giroscopio (Sección 3.5) ---
    // Ecuación (47): error angular por eje, a partir del error del cuaternión
    w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
    w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
    w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;

    // Ecuación (48): integrar el bias ponderado por zeta
    st->w_bx += w_err_x * st->deltat * st->zeta;
    st->w_by += w_err_y * st->deltat * st->zeta;
    st->w_bz += w_err_z * st->deltat * st->zeta;

    // Ecuación (49): restar el bias estimado de la medida del giroscopio
    w_x -= st->w_bx;
    w_y -= st->w_by;
    w_z -= st->w_bz;

    // --- Paso 5: velocidad de cambio del cuaternión a partir del giroscopio ---
    // Ecuación (12) del paper
    SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
    SEqDot_omega_2 =  halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
    SEqDot_omega_3 =  halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
    SEqDot_omega_4 =  halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;

    // --- Paso 6: fusión e integración, Ecuación (42)/(43) del artículo ---
    st->SEq_1 += (SEqDot_omega_1 - (st->beta * SEqHatDot_1)) * st->deltat;
    st->SEq_2 += (SEqDot_omega_2 - (st->beta * SEqHatDot_2)) * st->deltat;
    st->SEq_3 += (SEqDot_omega_3 - (st->beta * SEqHatDot_3)) * st->deltat;
    st->SEq_4 += (SEqDot_omega_4 - (st->beta * SEqHatDot_4)) * st->deltat;

    // Normalizar el cuaternión resultante (obligatorio: debe ser unitario)
    norm = sqrtf(st->SEq_1 * st->SEq_1 + st->SEq_2 * st->SEq_2
               + st->SEq_3 * st->SEq_3 + st->SEq_4 * st->SEq_4);
    st->SEq_1 /= norm; st->SEq_2 /= norm; st->SEq_3 /= norm; st->SEq_4 /= norm;

    // --- Paso 7: recalcular la referencia magnética para la siguiente vuelta ---
    // Sección 3.4 del artículo: esto es lo que limita el efecto de una distorsión
    // magnética al RUMBO (heading) en vez de contaminar cabeceo/alabeo.
    SEq_1SEq_2 = st->SEq_1 * st->SEq_2;
    SEq_1SEq_3 = st->SEq_1 * st->SEq_3;
    SEq_1SEq_4 = st->SEq_1 * st->SEq_4;
    SEq_3SEq_4 = st->SEq_3 * st->SEq_4;
    SEq_2SEq_3 = st->SEq_2 * st->SEq_3;
    SEq_2SEq_4 = st->SEq_2 * st->SEq_4;

    h_x = twom_x * (0.5f - st->SEq_3 * st->SEq_3 - st->SEq_4 * st->SEq_4)
        + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
    h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4)
        + twom_y * (0.5f - st->SEq_2 * st->SEq_2 - st->SEq_4 * st->SEq_4)
        + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
    h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2)
        + twom_z * (0.5f - st->SEq_2 * st->SEq_2 - st->SEq_3 * st->SEq_3);

    // Ecuación (46): la referencia solo tiene componentes en x y z (no en y)
    st->b_x = sqrtf((h_x * h_x) + (h_y * h_y));
    st->b_z = h_z;
}

//Variante de 6 ejes (SIN MAGNETÓMETRO) - mismo filtro que el de 9 ejes, pero solo tiene acel+giro
void madgwick_6_ejes(MadgwickState *st, float w_x, float w_y, float w_z, float a_x, float a_y, float a_z)
{
    float norm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;


    // --- Auxiliares para no repetir cálculos (igual que el artículo) ---
    float _2q0 = 2.0f * st->SEq_1;
    float _2q1 = 2.0f * st->SEq_2;
    float _2q2 = 2.0f * st->SEq_3;
    float _2q3 = 2.0f * st->SEq_4;
    float _4q0 = 4.0f * st->SEq_1;
    float _4q1 = 4.0f * st->SEq_2;
    float _4q2 = 4.0f * st->SEq_3;
    float _8q1 = 8.0f * st->SEq_2;
    float _8q2 = 8.0f * st->SEq_3;
    float q0q0 = st->SEq_1 * st->SEq_1;
    float q1q1 = st->SEq_2 * st->SEq_2;
    float q2q2 = st->SEq_3 * st->SEq_3;
    float q3q3 = st->SEq_4 * st->SEq_4;

    //--- Paso 1: velocidad de cambio del cuaternión a partir del giroscopio ---
    qDot1 = 0.5f * (-st->SEq_2 * w_x - st->SEq_3 * w_y - st->SEq_4 * w_z);
    qDot2 = 0.5f * ( st->SEq_1 * w_x + st->SEq_3 * w_z - st->SEq_4 * w_y);
    qDot3 = 0.5f * ( st->SEq_1 * w_y - st->SEq_2 * w_z + st->SEq_4 * w_x);
    qDot4 = 0.5f * ( st->SEq_1 * w_z + st->SEq_2 * w_y - st->SEq_3 * w_x);

    //--- Paso 2: si hay lectura de acelerómetro válida, aplicar la corrección ---
    norm = sqrtf(a_x * a_x + a_y * a_y + a_z * a_z);
    if (norm > 0.0f) {
        a_x /= norm; a_y /= norm; a_z /= norm;
        //Gradiente de la función objetivo (solo término de gravedad, Ecuación (25)
        //del artículo, sin los términos de magnetómetro f_4/f_5/f_6)
        s0 = _4q0 * q2q2 + _2q2 * a_x + _4q0 * q1q1 - _2q1 * a_y;
        s1 = _4q1 * q3q3 - _2q3 * a_x + 4.0f * q0q0 * st->SEq_2 - _2q0 * a_y - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * a_z;
        s2 = 4.0f * q0q0 * st->SEq_3 + _2q0 * a_x + _4q2 * q3q3 - _2q3 * a_y - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * a_z;
        s3 = 4.0f * q1q1 * st->SEq_4 - _2q1 * a_x + 4.0f * q2q2 * st->SEq_4 - _2q2 * a_y;

        norm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        if (norm > 0.0f) {
            s0 /= norm; s1 /= norm; s2 /= norm; s3 /= norm;
        }

        //Aplicar la corrección (beta es el MISMO que usa madgwick_9_ejes,
        //ya calculado en madgwick_init())
        qDot1 -= st->beta * s0;
        qDot2 -= st->beta * s1;
        qDot3 -= st->beta * s2;
        qDot4 -= st->beta * s3;
    }

    //--- Paso 3: integrar para obtener el nuevo cuaternión ---
    st->SEq_1 += qDot1 * st->deltat;
    st->SEq_2 += qDot2 * st->deltat;
    st->SEq_3 += qDot3 * st->deltat;
    st->SEq_4 += qDot4 * st->deltat;

    //--- Paso 4: normalizar (obligatorio, igual que en la versión MARG) ---
    norm = sqrtf(st->SEq_1 * st->SEq_1 + st->SEq_2 * st->SEq_2 + st->SEq_3 * st->SEq_3 + st->SEq_4 * st->SEq_4);
    st->SEq_1 /= norm; st->SEq_2 /= norm; st->SEq_3 /= norm; st->SEq_4 /= norm;
}

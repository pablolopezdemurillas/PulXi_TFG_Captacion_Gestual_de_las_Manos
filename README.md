# PulXi: Sistema de captación gestual de las manos basado en una red de sensores inerciales

**Autor:** Pablo López de Murillas García

**Trabajo Fin de Grado - Ingeniería Eléctrica y Electrónica Industrial**
**Escuela Politécnica Superior, Universidad de Sevilla**

Este repositorio contiene el código fuente y la documentación técnica de **PulXi**, un sistema embebido portable diseñado para la captura del movimiento de las manos y su digitalización en tiempo real. 

## 🎥 Vídeo de demostración
* [Funcionamiento del guante y la mano en Unity] -> (BORRA ESTO Y PEGA AQUÍ TU ENLACE)

## ⚙️ Arquitectura del Proyecto
* **Hardware:** 15 sensores inerciales (ICM-20948) distribuidos en 5 circuitos impresos flexibles (FPC), centralizados en una placa rígida (PCB) con multiplexores I2C (TCA9548A) y un microcontrolador ESP32.
* **Firmware (C++):** Sistema basado en FreeRTOS para el ESP32, con filtro de Madgwick para fusión sensorial y transmisión de cuaterniones por Bluetooth.
* **Software (C# / Unity):** Aplicación de visualización 3D que recibe los datos espaciales y aplica cinemática directa sobre un modelo articulado de la mano.

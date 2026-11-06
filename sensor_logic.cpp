#include "sensor_logic.h"
#include "icm20948_registers.h"
#include "data_types.h"
#include "i2c_manager.h"
#include "filtro_madgwick.h"
#include <SoftWire.h>
extern SoftWire Wire2;
//--VARIABLES PARA DEPURAR TIEMPOS I2C
unsigned  long tiempo_bus0=0;
unsigned  long tiempo_bus1=0;
unsigned  long tiempo_bus2=0;

//Variables
byte direccion;               //utilizada para almacenar temporalmente la dirección del sensor

//Offsets de calibración de los magnetómetros se autorellena al hacer la calibración, si no se asigna un 0
//Se obtuvieron con calibrar_magnetometro_mano_completa() y copiados directamente
//float offsets_magnetometro[6][3][3]={0};
float offsets_magnetometro[6][3][3]={

  {{-5.32f, -3.37f, 29.77f},{14.40f, -10.13f, 24.52f},{-15.82f, -24.30f, 0.90f}},
  //DEDO 1
  {{-4.35f, 7.57f, 13.35f},{-15.90f, 17.70f, 16.87f},{-6.07f, 12.45f, 1.50f}},
  //DEDO 2
  {{-1.80f, 1.35f, 3.98f},{0.37f, 11.02f, -3.90f},{2.10f, -42.52f, 40.58f}},
  //DEDO 3
  {{-5.25f, 15.97f, 29.85f},{0.75f, 0.75f, 34.42f},{15.52f, -46.28f, 45.97f}},
  //DEDO 4
  {{-12.07f, 23.02f, 1.27f},{-9.07f, 18.75f, -2.92f},{50.17f, 4.73f, -5.33f}}
};

//float offsets_giroscopio[6][3][3]={0};
float offsets_giroscopio[6][3][3]={
  //Dedo 0
  {{-1.46f, -0.47f, -0.81f},{-1.59f, -1.18f, 0.08f},{-1.75f, -1.26f, 0.05f}},
  //Dedo 1
  {{-3.03f, -1.30f, -0.16f},{2.07f, 0.46f, -0.60f},{-2.29f, 0.60f, -0.00f}},
  //Dedo 2
  {{-2.35f, -0.55f, 0.05f},{-1.37f, -0.51f, 0.05f},{-1.92f, -0.04f, -0.19f}},
  //Dedo 3
  {{-3.70f, -1.21f, 0.24f},{0.71f, -0.62f, 0.51f},{0.08f, 0.46f, -0.56f}},
  //Dedo 4
  {{-2.07f, -0.32f, 0.01f},{-3.43f, -1.08f, -0.57f},{-0.98f, -0.27f, -0.30f}}
};


//---------------------------------------------------
//    FALANGE DESCONECTADA (Cable FLEX ROTO) !!!
//---------------------------------------------------
//    Debido al uso, se ha roto la conexión I2C con F2-dedo índice
//    Se debe evitar la lectura para:
//        - No perder tiempo de lectura I2c
//        - Evitar error en I2C bit-banging por error de lectura
//        - Evitar división entre 0 en madgwick_6_ejes()
const int DEDO_DESCONECTADO = Menique;
const int FALANGE_DESCONECTADA = F2;




//__________________________________________________________
//Función para seleccionar dedo a leer, necesaria para abrir los canales indicados en el multiplexor
//__________________________________________________________
void seleccionar_dedo(uint8_t dedo)
{
  if (dedo>7){
    return;     //Protección hardware
  }
  //Indice=0    Corazon=1   Anular=2    Menique=3   Pulgar=4    Palma=5
  uint8_t canal=1<<dedo;   //así indice=00001

  //Escribimos en Multiplexor del Bus 0
  Wire.beginTransmission(0x70);   //Es la dirección del multiplexor i2c
  Wire.write(canal);
  Wire.endTransmission();
  
  //Escribimos en Multiplexor del Bus 1
  Wire1.beginTransmission(0x70);   //Es la dirección del multiplexor i2c
  Wire1.write(canal);
  Wire1.endTransmission();

  //Escribimos en Multiplexor del Bus 2
  Wire2.beginTransmission(0x70);   //Es la dirección del multiplexor i2c
  Wire2.write(canal);
  Wire2.endTransmission();
  

  delayMicroseconds(500);         //Se deja margen para que estabilice el bus (podríamos reducir los tiempos de los ciclos reduciendo este valor)
}
//______________________________________________________
//Función para apagar los multiplexores
//______________________________________________________
void reiniciar_multiplexores()
{
  
  //Escribimos en Multiplexor del Bus 0
  Wire.beginTransmission(0x70);   //Es la dirección del multiplexor i2c
  Wire.write(0x00);
  Wire.endTransmission();

  //Escribimos en Multiplexor del Bus 1
  Wire1.beginTransmission(0x70);   //Es la dirección del multiplexor i2c
  Wire1.write(0x00);
  Wire1.endTransmission();

  //Escribimos en Multiplexor del Bus 2
  Wire2.beginTransmission(0x70);   //Es la dirección del multiplexor i2c
  Wire2.write(0x00);
  Wire2.endTransmission();
  
  delay(10);                       //Tiempo de espera para que el MUX cierre todos los canales
}
//______________________________________________________
//Función para configurar un dedo completo (F0-F1-F2)
//______________________________________________________
void configurar_dedo(uint8_t dedo)
{
  Serial.print("\n===INICIANDO CONFIGURACIÓN DEL DEDO");
  Serial.print(dedo);
  Serial.println("===");
  //1. Abrimos el canal correspondiente en los tres multiplexores
  seleccionar_dedo(dedo);
  //2.Configuramos los IMUS de las tres falanges (F0-F1-F2)
  configuracion_sensor(F0);
  configurar_DLPF(F0,DLPF_CFG_MEDIO_1000DPS);
  configurar_magnetometro_via_bypass(F0);

  configuracion_sensor(F1);
  configurar_DLPF(F1,DLPF_CFG_MEDIO_1000DPS);
  //configurar_magnetometro_via_bypass(F1);   //En la estructura anterior se utilizaba el bypass, pero ahora ya no para así conseguir una mejor visualización

  configuracion_sensor(F2);
  configurar_DLPF(F2,DLPF_CFG_MEDIO_1000DPS);
  //configurar_magnetometro_via_bypass(F2);
  
  Serial.println("=======CONFIGURACIÓN DEL DEDO COMPLETADA========\n");
}
//_____________________________________________________________
//          CONFIGURACIÓN INICIAL DE CADA UNO DE LOS SENSORES
//_____________________________________________________________
void configuracion_sensor(int falange)
{
    int bus=falange; //F0 = BUS 0, F1 = BUS 1 y F2 = BUS 2
    uint8_t dir=0x69;
    Serial.print("------------CONFIGURACIÓN INICIAL DEL ICM-20948 (Falange F");
    Serial.print(falange);
    Serial.println(")-------");

    //1. Seleccionar USER BANK 0
    writeRegister(bus,dir,REG_BANK_SEL,USER_BANK_0);
    writeRegister(bus,dir,REG_PWR_MGMT_1,0x80);          //limpiamos el error de estado previo con 0x80
    delay(100);                                         //damos tiempo para estabilizar
    
    //2. Verificar WHO_AM_I --> Se espera el valor 0xEA
    uint8_t whoAmI = readRegister(bus,dir,REG_WHO_AM_I);
    Serial.print("VALOR DE WHO_AM_I: 0x");
    Serial.println(whoAmI,HEX);
    
    if (whoAmI == 0xEA)
    {
        Serial.print("[Exito] ¡Sensor detectado correctamente en 0x");
        Serial.println(dir, HEX);
    } else{
        Serial.print("[Error] no se reconoce el dispositivo 0x");
        Serial.println(dir, HEX);
        //while(1);   //detiene la ejecución si hay un error crítico
    }
    
    //3. Despertamos el sensor y configuramo el reloj
    //escribimos en el bit 6 un 0 y en los bits 2:0 =001 para seleccionar el mejor reloj disponible
    writeRegister(bus,dir,REG_PWR_MGMT_1,0x01);
    delay(100); //tiempo para establizar
    
    //4. Habilitamos los acelerómetros y giroscopios
    writeRegister(bus,dir,REG_PWR_MGMT_2, 0x00);
    Serial.println("Configuración completada: Sensor despierto y habilitado.");
}

//
void configurar_magnetometro_via_bypass(int bus){
  uint8_t dir_icm=0x69;
  uint8_t dir_manget=0x0C;
  Serial.print("----Configurando AK09916 vía Bypass en F"); Serial.print(bus); Serial.println("------");

  //1. Aseguramos el Banco 0
  writeRegister(bus, dir_icm, REG_BANK_SEL,USER_BANK_0);

  //2. Abrimos el Bypass
  writeRegister(bus, dir_icm, INT_PIN_CFG, 0x02);   //BYPASS_EN=1 en INT_PIN_CFG
  delay(10);    //Tiempo de estabilización

  //3. Verificación: Leer el ID del AK09916, debe ser 0x09
  uint8_t id_mag=readRegister(bus,AK09916,0x01);
  Serial.print("[DIAG] ID del Magnetómetro en F");Serial.print(bus);
  Serial.print(":0x");Serial.println(id_mag,HEX);
  if(id_mag==0x09)
  {
    Serial.println("[EXITO] Magentómetro detectado correctamente a través del bypass");
  } else{
    Serial.println("[ERROR] No se reconoce el magnetómetro. Revisar hardware/cableado");
  
  }

  //4. Soft-Reset para limpiar bloqueos internos antes de configurarlo
  writeRegister(bus, AK09916, 0x32,0x01);   //Registro CNTL3(0x32)=0x01
  delay(20);

  //5.Configurar magnetómetro a través del bypass
  //Modo contínuo (MODO 4 -->0x08) --> 100Hz
  writeRegister(bus, AK09916, CNTL2,0x08);
  delay(10);

  // Verificación
  uint8_t verificacion_cntl2=readRegister(bus,AK09916,CNTL2);
  Serial.print("[DIAG] CNTL2 del Magnetómetro en F");Serial.print(bus);
  Serial.print(":0x");Serial.println(verificacion_cntl2,HEX);
  if(verificacion_cntl2==0x08)
  {
    Serial.print("[EXITO] Modo contínuo confirmado en F");Serial.println(bus);
  } else{
    Serial.print("[ERROR] CNTL2 no se quedó a 0x08 en F");Serial.print(bus);
    Serial.print(", valor leído: 0x");Serial.println(verificacion_cntl2,HEX);
  
  }
  //No cerramos ByPass: F0 necesita acceso directo al magnetómetro durante la lectura de los primeros 5 segundos para calibrar
  delay(10);
}

//------------------------------------------------
//------------------------------------------------
//________________________________________________
//Función para leer las falanges de cada dedo
//________________________________________________
void leerfalange(int dedo,int falange)
{
  uint8_t dir=0x69;   //Todos los sensores tienen esta dirección, configurada vía Hardware
  int available_bytes = 0;

  //INICIO DEL CRONÓMETRO
  unsigned long t_inicio=micros();

  //int16_t ax, ay,az, gx, gy, gz;

  //1. Solicitamos los 22 bytes seguidos según el BUS que corresponda
  //12bytes de Aceleracion + Giroscopio
  //2bytes temporales
  //8 bytes magnetómetro
  if(falange==F0)
  {
    //1. Solicitamos leer del ICM20948 el registro de
    //0x2D --> ACCEL_X_H  |  0x2E --> ACCEL_X_L   |   0x2F --> ACCEL_Y_H    |   0x30 --> ACCEL_Y_L    |   0x31 --> ACCEL_Z_H    |   0x32 --> ACCEL_Z_L
    //0x33 --> GYRO_X_H   |  0x34 --> GYRO_X_L    |   0x35 --> GYRO_Y_H     |   0x36 --> GYRO_Y_L     |   0x37 --> GYRO_Z_H     |   0x38 --> GYRO_Z_L
    //Indicar leer 9bytes del magnetómetro--->0x09
    //0x3B --> EXT_SLV_SENS_DATA_00
    //0x3C --> EXT_SLV_SENS_DATA_01
    //0x3D --> EXT_SLV_SENS_DATA_02
    //0x3E --> EXT_SLV_SENS_DATA_03
    //0x3F --> EXT_SLV_SENS_DATA_04
    //0x40 --> EXT_SLV_SENS_DATA_05
    //0x41 --> EXT_SLV_SENS_DATA_06
    //0x42 --> EXT_SLV_SENS_DATA_07
    //0x43 --> EXT_SLV_SENS_DATA_08
    Wire.beginTransmission(dir);
    //comenzamos a leer desde la posición 0x2D
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);  //Reinicio para la lectura
    //2. El micro genera los pulsos de reloj para recibir 12 bytes seguidos
    // el sensor, sigue enviando paquetes de datos hasta que no se reciba una señal de STOP
    //automáticamente aumenta el puntero interno a la siguiente dirección de memoria
    //0x2D-->0x2E-->0x2F-->...-->0x38
    //Cada eje (X,Y,Z) tiene un byte alto(H) y uno bajo(L)
    //
    //solicitamos 12 bytes y tras ello mandamos la señal
    //NACK y el bit de STOP (todo lo hace automático la siguiente función)
    Wire.requestFrom(dir, (uint8_t)12);
    available_bytes=Wire.available();
    tiempo_bus0 +=(micros()-t_inicio);  //Acumulación tiempo BUS0
  }
  else if (falange==F1)  
  {
    Wire1.beginTransmission(dir);
    Wire1.write(ACCEL_XOUT_H);
    Wire1.endTransmission(false);
    Wire1.requestFrom(dir, (uint8_t)12);
    available_bytes=Wire1.available();
    tiempo_bus1 +=(micros()-t_inicio);  //Acumulación tiempo BUS1
  }
  else if (falange==F2)  
  {
    Wire2.beginTransmission(dir);
    Wire2.write(ACCEL_XOUT_H);
    Wire2.endTransmission(false);
    Wire2.requestFrom(dir, (uint8_t)12);
    available_bytes=Wire2.available();
    tiempo_bus2 +=(micros()-t_inicio);  //Acumulación tiempo BUS2
  }
  else
  {
    return; //falange no válida
  }

  //2. Comprobamos cuantos bytes han llegado y están en el buffer
  if(available_bytes>=12)    //uso un >= porque se puede llenar antes o llegar en dos tandas
  {
    int16_t ax, ay,az, gx, gy, gz;

    //Leemos y reconstruimos los 2 bytes o 16 bits (HIGH <<8|LOW)
    //los registros del sensor son de 8 bits
    //el dato de salida es de 16 bits en Ca2
    //Para el valor alto:
    //el primer byte lo desplazamos a la izq 8 pos
    //y lo combinamos con el valor bajo haciendo un
    //OR |
    if (falange==F0){
      ax=(int16_t)((Wire.read()<<8) | Wire.read());
      ay=(int16_t)((Wire.read()<<8) | Wire.read());
      az=(int16_t)((Wire.read()<<8) | Wire.read());
      gx=((int16_t)(Wire.read()<<8) | Wire.read());
      gy=(int16_t)((Wire.read()<<8) | Wire.read());
      gz=(int16_t)((Wire.read()<<8) | Wire.read());
    }
    else if (falange==F1){
      ax=(int16_t)((Wire1.read()<<8) | Wire1.read());
      ay=(int16_t)((Wire1.read()<<8) | Wire1.read());
      az=(int16_t)((Wire1.read()<<8) | Wire1.read());
      gx=((int16_t)(Wire1.read()<<8) | Wire1.read());
      gy=(int16_t)((Wire1.read()<<8) | Wire1.read());
      gz=(int16_t)((Wire1.read()<<8) | Wire1.read());
    }
    else if (falange==F2){
      ax=(int16_t)((Wire2.read()<<8) | Wire2.read());
      ay=(int16_t)((Wire2.read()<<8) | Wire2.read());
      az=(int16_t)((Wire2.read()<<8) | Wire2.read());
      gx=((int16_t)(Wire2.read()<<8) | Wire2.read());
      gy=(int16_t)((Wire2.read()<<8) | Wire2.read());
      gz=(int16_t)((Wire2.read()<<8) | Wire2.read());
    }


   
    // 3. Conversión a unidades físicas (Factores de escala por defecto)
    // Acelerómetro (+-2g): 16384 LSB/g [Pág. 12 y 83]
    // Giroscopio (+-250 dps): 131 LSB/dps [Pág. 11 y 79]
    datos[dedo][falange][Ax]=ax/16384.0;
    datos[dedo][falange][Ay]=ay/16384.0;
    datos[dedo][falange][Az]=az/16384.0;
    datos[dedo][falange][Gx]=gx/32.8;    //Correspondientes +-1000dps
    datos[dedo][falange][Gy]=gy/32.8;
    datos[dedo][falange][Gz]=gz/ 32.8;
  }
}
//-------------------------------------------------
//-------------------------Leer magnetometro-------
//-------------------------------------------------
//-------------------------------------------------
void leer_magnetometro(int dedo,int falange){
  
  int available_bytes=0;

  //INICIO DEL CRONÓMETRO
  unsigned long t_inicio=micros();

  if(falange==F0)
  {
    Wire.beginTransmission(AK09916);
    Wire.write(0x10);   //ST1
    Wire.endTransmission(false);  //Reinicio para la lectura
    Wire.requestFrom(AK09916, (uint8_t)9);
    available_bytes=Wire.available();
    tiempo_bus0 +=(micros()-t_inicio);
  }
  else if (falange==F1)  
  {
    Wire1.beginTransmission(AK09916);
    Wire1.write(0x10);
    Wire1.endTransmission(false);
    Wire1.requestFrom(AK09916, (uint8_t)9);
    available_bytes=Wire1.available();
    tiempo_bus1 +=(micros()-t_inicio);
  }
  else if (falange==F2)  
  {
    Wire2.beginTransmission(AK09916);
    Wire2.write(0x10);
    Wire2.endTransmission(false);
    Wire2.requestFrom(AK09916, (uint8_t)9);
    available_bytes=Wire2.available();
    tiempo_bus2 +=(micros()-t_inicio);
  }
  else
  {
    return; //falange no válida
  }

  //2. Comprobamos cuantos bytes han llegado y están en el buffer
  if(available_bytes>=9)   
  {
    int16_t mx, my, mz;
    uint8_t mlx, mhx, mly, mhy, mlz, mhz;


    if (falange==F0){
      Wire.read();    //ST1
      mlx=Wire.read(); mhx=Wire.read();   //Magnetómetro, LSB primero
      mly=Wire.read(); mhy=Wire.read(); 
      mlz=Wire.read(); mhz=Wire.read();
      Wire.read();Wire.read();    //BASURA (TMPS y ST2 del magnetómetro, libera la lectura)
    }
    else if (falange==F1){
      Wire1.read();   //ST1
      mlx=Wire1.read(); mhx=Wire1.read();   //Magnetómetro, LSB primero
      mly=Wire1.read(); mhy=Wire1.read(); 
      mlz=Wire1.read(); mhz=Wire1.read();
      Wire1.read();Wire1.read();    //BASURA (TMPS y ST2 del magnetómetro)
    }
    else if (falange==F2){
      Wire2.read();    //ST1
      mlx=Wire2.read(); mhx=Wire2.read();   //Magnetómetro, LSB primero
      mly=Wire2.read(); mhy=Wire2.read(); 
      mlz=Wire2.read(); mhz=Wire2.read();
      Wire2.read();Wire2.read();    //BASURA (TMPS y ST2 del magnetómetro)
    }

    //Reconstruimos el magnetómetro
    mx=(int16_t)((mhx<<8)|mlx);
    my=(int16_t)((mhy<<8)|mly);
    mz=(int16_t)((mhz<<8)|mlz);
   
    // 3. Conversión a unidades físicas (Factores de escala por defecto)


    datos[dedo][falange][Mx]=mx*0.15;
    datos[dedo][falange][My]=my*0.15;
    datos[dedo][falange][Mz]=mz*0.15;
  }
  
}

void leerdedo(int dedo, float deltat_real)
{
  seleccionar_dedo(dedo);

  //Actualizar el reloj de filtros para el dedo
  madgwick_set_deltat(&filtros[dedo][F0],deltat_real);
  madgwick_set_deltat(&filtros[dedo][F1],deltat_real);
  madgwick_set_deltat(&filtros[dedo][F2],deltat_real);
  float mx_corregido, my_corregido, mz_corregido;

  //========================================
  //      LECTURA Y FUSIÓN DE LA FALANGE F0
  //   Los primeros ciclos se usa acel+giro+magnet
  //   Luego solo acel+giro para evitar el drift(deriva) progresiva debido a la influencia
  //  de campos magnéticos variables al mover la mano
  //========================================
  if(!(dedo==DEDO_DESCONECTADO && FALANGE_DESCONECTADA==F0))
  {
    leerfalange(dedo, F0);
    

    //1.Convertimos grados/s a rad/s para el giroscopio
    float gx_rad_F0=(datos[dedo][F0][Gx]-offsets_giroscopio[dedo][F0][0])*0.0174533f;
    float gy_rad_F0=(datos[dedo][F0][Gy]-offsets_giroscopio[dedo][F0][1])*0.0174533f;
    float gz_rad_F0=(datos[dedo][F0][Gz]-offsets_giroscopio[dedo][F0][2])*0.0174533f;

    bool usoF0 =(millis()-tiempoArranqueFiltros) < DURACION_SEMBRADO_MS;
    if (usoF0){
      leer_magnetometro(dedo, F0);
      //Restamos los offset hard-iron al magnetómetro
      mx_corregido=datos[dedo][F0][Mx] - offsets_magnetometro[dedo][F0][0];
      my_corregido=datos[dedo][F0][My] - offsets_magnetometro[dedo][F0][1];
      mz_corregido=datos[dedo][F0][Mz] - offsets_magnetometro[dedo][F0][2];

      //Aplicamos el filtro de Madgwick de 9 ejes
      madgwick_9_ejes(&filtros[dedo][F0],
        gx_rad_F0, gy_rad_F0, gz_rad_F0,
        datos[dedo][F0][Ax], datos[dedo][F0][Ay], datos[dedo][F0][Az],
        mx_corregido, my_corregido, mz_corregido
      );
    }else{
      madgwick_6_ejes(&filtros[dedo][F0],
        gx_rad_F0, gy_rad_F0, gz_rad_F0,
        datos[dedo][F0][Ax], datos[dedo][F0][Ay], datos[dedo][F0][Az]
      );
    }  
    //3.Guardamos el resultado en la matriz global de cuaterniones
    xSemaphoreTake(semaforoCuaterniones, portMAX_DELAY);
    cuaterniones[dedo][F0][0]= filtros[dedo][F0].SEq_1;   //Qw
    cuaterniones[dedo][F0][1]= filtros[dedo][F0].SEq_2;   //Qx
    cuaterniones[dedo][F0][2]= filtros[dedo][F0].SEq_3;   //Qy
    cuaterniones[dedo][F0][3]= filtros[dedo][F0].SEq_4;   //Qz
    xSemaphoreGive(semaforoCuaterniones);
  }



  //========================================
  //      LECTURA Y FUSIÓN DE LA FALANGE F1
  //    Siempre sin magnetómetro, transmite relativo a F0 así que no afecta la pos.absoluta
  //========================================
  if(!(dedo==DEDO_DESCONECTADO && FALANGE_DESCONECTADA==F1))
  {
    leerfalange(dedo, F1);

    //1.Convertimos grados/s a rad/s para el giroscopio
    float gx_rad_F1=(datos[dedo][F1][Gx]-offsets_giroscopio[dedo][F1][0])*0.0174533f;
    float gy_rad_F1=(datos[dedo][F1][Gy]-offsets_giroscopio[dedo][F1][1])*0.0174533f;
    float gz_rad_F1=(datos[dedo][F1][Gz]-offsets_giroscopio[dedo][F1][2])*0.0174533f;

    
    madgwick_6_ejes(&filtros[dedo][F1],
      gx_rad_F1, gy_rad_F1 , gz_rad_F1,
      datos[dedo][F1][Ax], datos[dedo][F1][Ay], datos[dedo][F1][Az]
    );
    //3.Guardamos el resultado en la matriz global de cuaterniones
    xSemaphoreTake(semaforoCuaterniones, portMAX_DELAY);
    cuaterniones[dedo][F1][0]= filtros[dedo][F1].SEq_1;   //Qw
    cuaterniones[dedo][F1][1]= filtros[dedo][F1].SEq_2;   //Qx
    cuaterniones[dedo][F1][2]= filtros[dedo][F1].SEq_3;   //Qy
    cuaterniones[dedo][F1][3]= filtros[dedo][F1].SEq_4;   //Qz
    xSemaphoreGive(semaforoCuaterniones);
  }


  //========================================
  //      LECTURA Y FUSIÓN DE LA FALANGE F2
  //   Para hacer más rápida la lectura, no se lee el magnetómetro
  //  Así se reduce el cuello de botella
  //  Porque además, no se necesita el magnetómetro
  //========================================
    if(!(dedo==DEDO_DESCONECTADO && FALANGE_DESCONECTADA==F2))
  {


    leerfalange(dedo, F2);

    //1.Convertimos grados/s a rad/s para el giroscopio
    float gx_rad_F2=(datos[dedo][F2][Gx]-offsets_giroscopio[dedo][F2][0])*0.0174533f;
    float gy_rad_F2=(datos[dedo][F2][Gy]-offsets_giroscopio[dedo][F2][1])*0.0174533f;
    float gz_rad_F2=(datos[dedo][F2][Gz]-offsets_giroscopio[dedo][F2][2])*0.0174533f;

    
    madgwick_6_ejes(&filtros[dedo][F2],
      gx_rad_F2, gy_rad_F2, gz_rad_F2,
      datos[dedo][F2][Ax], datos[dedo][F2][Ay], datos[dedo][F2][Az]
    );
    //3.Guardamos el resultado en la matriz global de cuaterniones
    xSemaphoreTake(semaforoCuaterniones, portMAX_DELAY);
    cuaterniones[dedo][F2][0]= filtros[dedo][F2].SEq_1;   //Qw
    cuaterniones[dedo][F2][1]= filtros[dedo][F2].SEq_2;   //Qx
    cuaterniones[dedo][F2][2]= filtros[dedo][F2].SEq_3;   //Qy
    cuaterniones[dedo][F2][3]= filtros[dedo][F2].SEq_4;   //Qz
    xSemaphoreGive(semaforoCuaterniones);
  }

  // AL FINAL DE LA FUNCIÓN, mostramos los tiempos medidos (1 vez por segundo = 100 ciclos)
  if (dedo == Indice) {
     static int contador_tiempo = 0;
     contador_tiempo++;
     if (contador_tiempo >= 100) { 
        Serial.printf("[TIEMPO I2C] Dedo Indice -> Bus 0 (HW): %lu us | Bus 1 (HW): %lu us | Bus 2 (SoftWire): %lu us\n", 
                      tiempo_bus0, tiempo_bus1, tiempo_bus2);
        contador_tiempo = 0;
     }
  }
}


//_______________________________________________________
//        FUNCIÓN PARA CONFIGURAR EL FILTRADO PASO BAJA
//_______________________________________________________

void configurar_DLPF(int bus, uint8_t config_byte)
{
  direccion=ICM20948_ADDR_H;  //0x69
  Serial.print("---Activando FILTRO en F");Serial.print(bus);Serial.println("---");
  //1. Asegurar Banco 2
  writeRegister(bus,direccion,REG_BANK_SEL,USER_BANK_2);

  //2.Aplica DLPF+FS_SEL+FCHOICE al giroscopio.
  writeRegister(bus,direccion,GYRO_CONFIG_1, config_byte);
  //3.Aplica DLPF+FS_SEL+FCHOICE al acelerometro
  writeRegister(bus,direccion,ACCEL_CONFIG, config_byte);
  //4. Verificacion: Leemos para ver qué se ha guardado
    //RESETEAR BUS I2C interno por si el AK09916 se quedó colgado
  uint8_t verificacion_giro=readRegister(bus, direccion, GYRO_CONFIG_1);
  uint8_t verificacion_acel=readRegister(bus, direccion, ACCEL_CONFIG);
  if (verificacion_giro ==config_byte && verificacion_acel==config_byte)
  {
    Serial.println("[EXITO] Filtro aplicado correctamente.");
  }else{
    Serial.println("[ERROR] Fallo al aplicar el filtro.");
  }
  //5. Volver al banco 0 antes de salir, ya que consideramos que está activo por defecto
  writeRegister(bus, direccion,REG_BANK_SEL, USER_BANK_0);
}

//________________________________________________________
//Funcion auxiliar para imprimir cada falange de cada dedo
//________________________________________________________
void imprimir_falange_por_pantalla(int dedo, int falange)
{
  Serial.print(datos[dedo][falange][Ax]); Serial.print(",");
  Serial.print(datos[dedo][falange][Ay]); Serial.print(",");
  Serial.print(datos[dedo][falange][Az]); Serial.print(",");
  Serial.print(datos[dedo][falange][Gx]); Serial.print(",");
  Serial.print(datos[dedo][falange][Gy]); Serial.print(",");
  Serial.print(datos[dedo][falange][Gz]); Serial.print(",");
  Serial.print(datos[dedo][falange][Mx]); Serial.print(",");
  Serial.print(datos[dedo][falange][My]); Serial.print(",");
  if(falange==F2){
  Serial.println(datos[dedo][falange][Mz]);
  }else{
    Serial.print(datos[dedo][falange][Mz]); Serial.print(",");
  }
}
//----------------------------------------------------------
//          Imprimir dedo por pantalla
//----------------------------------------------------------
void imprimir_dedo_por_pantalla(int dedo){
  imprimir_falange_por_pantalla(dedo,F0);
  imprimir_falange_por_pantalla(dedo,F2);
}

//__________________________________________________________
//Funcion auxiliar para visualizar las graficas de cada falange de cada dedo
//__________________________________________________________
void imprimir_falange_por_teleplot(int dedo, int falange)
{
  Serial.print(">Ax_F");Serial.print(falange);Serial.print(":");Serial.println(datos[dedo][falange][Ax]);
  Serial.print(">Ay_F");Serial.print(falange);Serial.print(":");Serial.println(datos[dedo][falange][Ay]);
  Serial.print(">Az_F");Serial.print(falange);Serial.print(":");Serial.println(datos[dedo][falange][Az]);

  Serial.print(">Gx_F");Serial.print(falange);Serial.print(":");Serial.println(datos[dedo][falange][Gx]);
  Serial.print(">Gy_F");Serial.print(falange);Serial.print(":");Serial.println(datos[dedo][falange][Gy]);
  Serial.print(">Gz_F");Serial.print(falange);Serial.print(":");Serial.println(datos[dedo][falange][Gz]);
  
  
  Serial.print(">Mx_F");Serial.print(falange);Serial.print(":");Serial.println(datos[dedo][falange][Mx]);
  Serial.print(">My_F");Serial.print(falange);Serial.print(":");Serial.println(datos[dedo][falange][My]);
  Serial.print(">Mz_F");Serial.print(falange);Serial.print(":");Serial.println(datos[dedo][falange][Mz]);
  
}

//_______________________________________________________
//Función para visualizar las graficas de cada dedo
//_______________________________________________________
void imprimir_dedo_por_teleplot(int dedo)
{ 
  imprimir_falange_por_teleplot(dedo,F0);
  imprimir_falange_por_teleplot(dedo,F1);
  imprimir_falange_por_teleplot(dedo,F2);
}

//________________________________________________________
// Función para imprimir datos en formato CSV para MATLAB
// Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz
//________________________________________________________
void imprimir_dedo_para_matlab(int dedo)
{
  Serial.print(datos[Indice][F2][Ax],4); Serial.print(",");
  Serial.print(datos[Indice][F2][Ay],4); Serial.print(",");
  Serial.print(datos[Indice][F2][Az],4); Serial.print(",");
  Serial.print(datos[Indice][F2][Gx],4); Serial.print(",");
  Serial.print(datos[Indice][F2][Gy],4); Serial.print(",");
  Serial.print(datos[Indice][F2][Gz],4); Serial.print(",");
  Serial.print(datos[Indice][F2][Mx],4); Serial.print(",");
  Serial.print(datos[Indice][F2][My],4); Serial.print(",");
  Serial.println(datos[Indice][F2][Mz],4);
  delay(10);    //+- 100 Hz
}


void calibrar_magnetometro_mano_completa(const uint8_t *dedos, int num_dedos, int duracion_ms) {
  Serial.println("=================================================");
  Serial.println("¡ATENCIÓN! Mueve la mano completa en forma de 8 (todas direcciones),");
  Serial.println(" Y además,flexiona los dedos mientras tanto.");
  Serial.println("=================================================");
    
  // Se usa float porque tus datos ya están en microTeslas (uT)
  float min_x[6][3];
  float max_x[6][3];
  float min_y[6][3];
  float max_y[6][3];
  float min_z[6][3];
  float max_z[6][3];

  //1. Lectura inicial de cada sensor para tener un punto de partida
  for(int d=0; d<num_dedos; d++)
  {
    uint8_t dedo=dedos[d];
    seleccionar_dedo(dedo);
    for(int f=F0; f<=F2; f++)
    {
      leer_magnetometro(dedo,f);
      min_x[dedo][f]=max_x[dedo][f]=datos[dedo][f][Mx];
      min_y[dedo][f]=max_y[dedo][f]=datos[dedo][f][My];
      min_z[dedo][f]=max_z[dedo][f]=datos[dedo][f][Mz];
    }
  }

  
    
  // 2. Bucle de captura durante el tiempo indicado
  //Recorre todos los sensores a la vez, para que así un único 8 de la mano
  //permite obtener máximos y mínimos de cada falange independiente
  unsigned long tiempo_inicio = millis();
  while (millis() - tiempo_inicio < (unsigned long)duracion_ms) {
    for (int d=0; d<num_dedos; d++)
    {
      uint8_t dedo=dedos[d];
      seleccionar_dedo(dedo);
      for(int f=F0; f<=F2; f++)
      {
        leer_magnetometro(dedo,f);
        float mx = datos[dedo][f][Mx];
        float my = datos[dedo][f][My];
        float mz = datos[dedo][f][Mz];
        // Actualizamos los extremos si encontramos un valor mayor o menor
        if (mx < min_x[dedo][f]) min_x[dedo][f] = mx;
        if (mx > max_x[dedo][f]) max_x[dedo][f] = mx;
          
        if (my < min_y[dedo][f]) min_y[dedo][f] = my;
        if (my > max_y[dedo][f]) max_y[dedo][f] = my;
          
        if (mz < min_z[dedo][f]) min_z[dedo][f] = mz;
        if (mz > max_z[dedo][f]) max_z[dedo][f] = mz;
      }
    }
  }

  // 3. Calcular el punto central del offset (Hard Iron)
  Serial.println("===CALIBRACIÓN COMPLETADA===");
  for(int d=0; d<num_dedos; d++)
  {
    uint8_t dedo=dedos[d];
    Serial.print("DEDO "); Serial.println(dedo);
    Serial.print("{");
    for (int f=F0; f<=F2; f++)
    {
      offsets_magnetometro[dedo][f][0] = (max_x[dedo][f] + min_x[dedo][f]) / 2.0f;
      offsets_magnetometro[dedo][f][1] = (max_y[dedo][f] + min_y[dedo][f]) / 2.0f;
      offsets_magnetometro[dedo][f][2] = (max_z[dedo][f] + min_z[dedo][f]) / 2.0f;

      Serial.print("{"); Serial.print(offsets_magnetometro[dedo][f][0]);
      Serial.print("f, "); Serial.print(offsets_magnetometro[dedo][f][1]);
      Serial.print("f, "); Serial.print(offsets_magnetometro[dedo][f][2]);
      Serial.print("f},");
    }
    Serial.print("}");
  }
  Serial.println("=================================================");
}

void calibrar_sesgo_giroscopio(const uint8_t *dedos, int num_dedos, int duracion_ms){
  Serial.println("=================================================");
  Serial.println("CALIBRANDO SESGO DEL GIROSCOPIO - NO MOVER LA MANO");
  Serial.println("=================================================");
  float suma_x[6][3]={0}, suma_y[6][3]={0}, suma_z[6][3]={0};
  int muestras=0;

  unsigned long tiempo_inicio=millis();
  while (millis()-tiempo_inicio < (unsigned long) duracion_ms){
    for(int d=0; d<num_dedos; d++){
      uint8_t dedo=dedos[d];
      seleccionar_dedo(dedo);
      for(int f=F0; f<=F2; f++){
        leerfalange(dedo,f);
        suma_x[dedo][f] +=datos[dedo][f][Gx];
        suma_y[dedo][f] +=datos[dedo][f][Gy];
        suma_z[dedo][f] +=datos[dedo][f][Gz];
      }
    }
    muestras++;
  }
  Serial.println("===SESGO DE GIROSCOPIO CALCULADO===");
  for(int d=0; d<num_dedos; d++){
    uint8_t dedo=dedos[d];
    Serial.print("Dedo "); Serial.println(dedo);
    Serial.print("{");
    for (int f=F0; f<=F2; f++){
      offsets_giroscopio[dedo][f][0]=suma_x[dedo][f]/muestras;
      offsets_giroscopio[dedo][f][1]=suma_y[dedo][f]/muestras;
      offsets_giroscopio[dedo][f][2]=suma_z[dedo][f]/muestras;
      Serial.print("{"); Serial.print(offsets_giroscopio[dedo][f][0]);
      Serial.print("f, "); Serial.print(offsets_giroscopio[dedo][f][1]);
      Serial.print("f, "); Serial.print(offsets_giroscopio[dedo][f][2]);
      Serial.print("f}");

    }
    Serial.println("}");
  }
  Serial.println("=============================================================");
}
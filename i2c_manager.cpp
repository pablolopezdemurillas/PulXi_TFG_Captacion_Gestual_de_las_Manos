#include "i2c_manager.h"
#include "data_types.h"             //necesario para el enum 'Indice'
#include <SoftWire.h>           //Librería del Bit-Banging

//Variables de pines I2C del Bus 0 (F0) --> Hardware
int SDA_F0=21;
int SCL_F0=22;

//Variables de pines I2C del Bus 1 (F1) --> Hardware
int SDA_F1=32;
int SCL_F1=33;

//Variables de pines I2C del BUS 2 (F2) --> Software
int SDA_F2=25;
int SCL_F2=26;

int interr=34;              //Pin comunitario de todas las interrupciones de los ICM-20948 (no usado en esta versión)

//Buffers para SoftWire
uint8_t txBuffer_F2[32];    //32 bytes es el máximo de una transacción I2C estándar
uint8_t rxBuffer_F2[32];

//Iniciamos el Bit-Banging para que ya esté vinculado
SoftWire Wire2(SDA_F2,SCL_F2);


//Función auxiliar para escribir en un registro
void writeRegister(int bus, uint8_t sensor, uint8_t reg, uint8_t data)
{
    if(bus==1){
      Wire1.beginTransmission(sensor);
      Wire1.write(reg);
      Wire1.write(data);
      Wire1.endTransmission();
    } else if (bus==2){
      Wire2.beginTransmission(sensor);
      Wire2.write(reg);
      Wire2.write(data);
      Wire2.endTransmission();
    }else {
      //Por defecto BUS 0
      Wire.beginTransmission(sensor);
      Wire.write(reg);
      Wire.write(data);
      Wire.endTransmission();
    }
  
}

//Función auxiliar para leer un registro
uint8_t readRegister(int bus, uint8_t sensor, uint8_t reg)
{
  uint8_t data=0;                       //Nunca se alcanza: todas las ramas lo llaman bien. Se mantiene para evitar el warning al compilar  
  if (bus==1){
    Wire1.beginTransmission(sensor);
    Wire1.write(reg);
    Wire1.endTransmission(false);
    Wire1.requestFrom(sensor,(uint8_t)1);
    return Wire1.read();
  } else if (bus==2){
    Wire2.beginTransmission(sensor);
    Wire2.write(reg);
    Wire2.endTransmission(false);
    Wire2.requestFrom(sensor,(uint8_t)1);
    return Wire2.read();
  } else{
    Wire.beginTransmission(sensor);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(sensor,(uint8_t)1);
    return Wire.read();
  }
  return data;
}

//Solicitar múltiples bytes según el bus
void requestFromBus(int bus, uint8_t sensor, uint8_t bytes){
  if (bus==1)
  {
    Wire1.requestFrom(sensor,bytes);
  }else if (bus==2)
  {
    Wire2.requestFrom(sensor,bytes);
  } else
  {
    Wire.requestFrom(sensor,bytes);
  }
}
// Funciones de compatibilidad con las llamadas anteriores (bus 0)
void writeRegister(uint8_t sensor, uint8_t reg, uint8_t data) {
  writeRegister(0, sensor, reg, data);
}
uint8_t readRegister(uint8_t sensor, uint8_t reg) {
  return readRegister(0, sensor, reg);
}

//Función inicializa bus I2C hardware SDA=21, SCL=22
void iniciabuses()
{
  delay(500);                         //tiempo de asentamiento del bus I2C
  //Iniciamos BUS 0
  Wire.begin(SDA_F0,SCL_F0);
  Wire.setClock(400000);              //100kHz para pruebas iniciales --> después configurado en Fast Mode (400kHz)

  

  //Iniciamos BUS 1
  Wire1.begin(SDA_F1,SCL_F1);
  Wire1.setClock(400000);             //100kHz para pruebas iniciales--> después configurado en Fast Mode (400kHz)

  //--CONFIGURACIÓN DE BUS2 (SOFTWARE)
  Wire2.setTxBuffer(txBuffer_F2,sizeof(txBuffer_F2));
  Wire2.setRxBuffer(rxBuffer_F2,sizeof(rxBuffer_F2));
  //Para 400 kHz el semiperiodo es de 1.25us, pero solo se pueden meter numeros enteros(1-2-3-4-5...)
  //Se ajusta a 2 us (250 kHz) para máxima estabilidad
  Wire2.setDelay_us(2);

  Wire2.begin();
}

//----------------------------------------------------------
//Funcion que escanea el Bus I2C
//----------------------------------------------------------
void escaneobusI2C()
{
  //contamos los módulos que hay escaneando el bus i2c
  int ModDetect_F0=0;
  int ModDetect_F1=0;
  int ModDetect_F2=0;

  Serial.println("Escaneando BUS 0 (Wire)...");
  //según el protocolo i2c las direcciones en su bus se organizan de la siguiente manera:
  // 7bit ADDR | bit R/W  --> como son 7 bits esto hace que sean 127 posibles direcciones
  //ADDR  0         -->   Transmisión
  //ADDR  1-7       -->   Otros fines
  //ADDR  8-119     -->   ÚTILES
  //ADDR  120-127   --> Usos futuros
  for(byte addr = 8; addr < 120; addr++ )
  {
    // Wire.beginTransmission inicia la comunicación con la dirección 'address'
    Wire.beginTransmission(addr);
    //vemos si la transmisión ha tenido éxito--> Si es así, devuelve un 0
    if (Wire.endTransmission()== 0) {
      Serial.print("--->Esclavo encontrado en F0:      0x");
      Serial.println(addr, HEX);
      ModDetect_F0++;
    }   
    delay(5);
  }


  
  Serial.println("Escaneando BUS 1 (Wire 1)...");
  for(byte addr = 8; addr < 120; addr++ )
  {
    Wire1.beginTransmission(addr);
    //vemos si la transmisión ha tenido éxito--> Si es así, devuelve un 0
    if (Wire1.endTransmission()== 0) {
      Serial.print("--->Esclavo encontrado en F1:      0x");
      Serial.println(addr, HEX);
      ModDetect_F1++;
    }   
    delay(5);
  }

  Serial.println("Escaneando BUS 2 (Software BitBanging)...");
  for(byte addr = 8; addr < 120; addr++ )
  {
    Wire2.beginTransmission(addr);
    //vemos si la transmisión ha tenido éxito--> Si es así, devuelve un 0
    if (Wire2.endTransmission()== 0) {
      Serial.print("--->Esclavo encontrado en F2:      0x");
      Serial.println(addr, HEX);
      ModDetect_F2++;
    }   
    delay(5);
  }
  
  
  
  Serial.print("[INFO] Fin de búsqueda. Encontrados -> BUS 0:");
  Serial.print(ModDetect_F0);

  
  Serial.print("    | BUS 1:");
  Serial.print(ModDetect_F1);
  Serial.print("    | BUS 2:");
  Serial.println(ModDetect_F2);
  
}

//_________________________________________________________
//Función para desatascar sensores colgados tras un reinicio
//_________________________________________________________
void liberarbusI2C(){
  Serial.begin(115200);
  // Esperar un poco a que el puerto serie arranque
  //while (!Serial); 
  Serial.println("======================INICIANDO SISTEMA======================");

  //Liberamos el BUS 0
  pinMode(SDA_F0, INPUT_PULLUP);
  pinMode(SCL_F0, INPUT_PULLUP);
  delay (10);
  //Si la linea SDA está baja, un sensor ha quedado "hablando"
  if(digitalRead(SDA_F0)==LOW)
  {
    Serial.println("[ALERTA] Bus 0 atascado. Forzando liberación...................");
    //Mandamos 9 pulsos de reloj manuales para obligar al esclavo a soltar el envío de información
    for (int i=0;i<9;i++)
    {
      pinMode(SCL_F0, OUTPUT);
      digitalWrite (SCL_F0, LOW);
      delayMicroseconds(10);
      pinMode(SCL_F0, INPUT_PULLUP);
      delayMicroseconds(10);
    }
  }
  //Liberamos el BUS 1
  pinMode(SDA_F1, INPUT_PULLUP);
  pinMode(SCL_F1, INPUT_PULLUP);
  delay (10);
  //Si la linea SDA está baja, un sensor ha quedado "hablando"
  if(digitalRead(SDA_F1)==LOW)
  {
    Serial.println("[ALERTA] Bus 1 atascado. Forzando liberación...................");
    //Mandamos 9 pulsos de reloj manuales para obligar al esclavo a soltar el envío de información
    for (int i=0;i<9;i++)
    {
      pinMode(SCL_F1, OUTPUT);
      digitalWrite (SCL_F1, LOW);
      delayMicroseconds(10);
      pinMode(SCL_F1, INPUT_PULLUP);
      delayMicroseconds(10);
    }
  }
}
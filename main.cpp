#include <Arduino.h>
#include <Wire.h>
#include "BluetoothSerial.h"
#include "icm20948_registers.h"
#include "data_types.h"
#include "i2c_manager.h"
#include "sensor_logic.h"

extern unsigned long tiempo_bus0, tiempo_bus1, tiempo_bus2;   // sensor_logic.cpp


//Variables globales
float datos[6][3][9];                   //[6 dedos][3 falanges][9 canales: Ax,Ay,Az,Gx,Gy,Gz,Mx,My,Mz]"
MadgwickState filtros[6][3];            //un filtro de Madgwick para cada falange
float cuaterniones[6][3][4];            //Resultado de la fusión: [dedo][falange][Qw,Qx,Qy,Qz]
BluetoothSerial SerialBT;
TaskHandle_t handleComunicacion= NULL;  //notificación escrita por la tarea de fusión para que se haga la tarea de comunicación
constexpr uint8_t dedosActivos[]={Pulgar, Indice, Corazon, Anular, Menique};      //Los 5 dedos están activos
constexpr int NUM_DEDOS_ACTIVOS = sizeof(dedosActivos) / sizeof(dedosActivos[0]);

//Como los cuaterniones son los datos compartidos entre ambos núcleos
//Usaremos un Semáforo para indicar si alguno de los dos está escribiendo/leyendo
SemaphoreHandle_t semaforoCuaterniones;

unsigned long tiempoArranqueFiltros;

//===============================================================
// TAREA NÚCLEO 1: LECTURA I2C Y FILTRO DE MADGWICK A (100HZ)
//===============================================================
void TareaSensorFusion(void *pvParameters){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);  //Metrónomo a 100 Hz (10ms)
  int ciclos=0;
  unsigned long ultimoPrint=millis();

  for(;;){
    //Medir tiempo real transcurrido (deltat) para luego vincular la lectura de cada dedo y falange al mismo instante de tiempo
    static unsigned long anterior=micros();
    unsigned long ahora=micros();
    unsigned long periodo_us=ahora-anterior;    //periodo real del ciclo anterior
    float deltat_real=(ahora-anterior)/1000000.0f;
    anterior=ahora;
    //Para evitar que el deltat meta un salto grande en el filtro tras un reset debemos meterlo dentro de un rango
    if (deltat_real < 0.001f) deltat_real=0.001f;       //Límite inferior 1ms (evita que sea inestable el filtro)
    if (deltat_real > 0.15f) deltat_real=0.15f;         //salvaguardar casos extremos (reset)
    
    

    for(int d=0; d<NUM_DEDOS_ACTIVOS; d++)
    {
      leerdedo(dedosActivos[d],deltat_real);    //Leemos los sensores (internamente ya hacen el filtro de Madgwick)
    }
    #if MEDIR_TIEMPOS
         {
           static unsigned long p0=0, p1=0, p2=0;
           unsigned long cuerpo_us=micros()-ahora;       // lectura y fusión de los cinco dedos
           Serial.printf("[CICLO];%lu;%lu;%lu;%lu;%lu\n", periodo_us, cuerpo_us,
                         tiempo_bus0-p0, tiempo_bus1-p1, tiempo_bus2-p2);
           p0=tiempo_bus0; p1=tiempo_bus1; p2=tiempo_bus2;
         }
    #endif

    ciclos++;
    if(millis()-ultimoPrint>=1000)
    {
      Serial.printf("[DIAG] Fusion real: %d Hz (objetivo: 100 Hz)\n",ciclos);
      ciclos=0;
      ultimoPrint=millis();
    }   
    //Avisar a Notificación de que hay un ciclo completo nuevo listo para ser enviado
    if(handleComunicacion !=NULL)
    {
      xTaskNotifyGive(handleComunicacion);
    } 
    //La tarea duerme hasta la siguiente ejecución (10ms)
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

//===============================================================
// TAREA NÚCLEO 0: COMUNICACIONES MEDIANTE SERIAL/BLUETOOTH (50 HZ)
//===============================================================
void TareaComunicacion(void  *pvParameters){

  //Buffer para paquete binario
  // Byte 0:      0xAA          <- marca de sincronismo
      // Byte 1:      dedo          (0-5, enum de data_types.h)
      // Byte 2:      falange       (0-2, enum F0/F1/F2)
      // Bytes 3-6:   Qw (float, 4 bytes)
      // Bytes 7-10:  Qx (float, 4 bytes)
      // Bytes 11-14: Qy (float, 4 bytes)
      // Bytes 15-18: Qz (float, 4 bytes)
      // Byte 19:     checksum      (XOR de los bytes 1 a 18)
  //Total de 20 bytes por falange
  //Total por dedo de 60 bytes
  //Total de mano 300 bytes
  uint8_t bufferBinario[NUM_DEDOS_ACTIVOS*3*20];

  for(;;){
    //1.MODO ESPERA: Si no hay Bluetooth, no gastamos recursos
    if(!SerialBT.hasClient()){
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;   //Volvemos al inicio del bucle
    }
    //Comprobar si se ha pulsado en Unity el botón de HOMING
    if (SerialBT.available())
    {
      char comando_unity=SerialBT.read();
      if (comando_unity == 'H' || comando_unity == 'h')
      {
        tiempoArranqueFiltros = millis();   //tenemos que volver a medir magnetómetros de F0 para tener la posicion abs
        Serial.println("[HOMING] Vuelta a la posición inicial.");
      }
    }

    //2.Hay cliente conectado
    //Bloqueado aquí hasta que TareaSensorFusion notifique un ciclo completo nuevo para así solo enviar la información actualizada
    //Damos 200ms para volver a comprobar hasClient() por si el cliente se desconecta mientras esperamos
    uint32_t notificado=ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));
    if(notificado ==0)
    {continue;
    }
    //Bloqueo del semáforo (o esperamos a que este quede libre para ejecutar)
    xSemaphoreTake(semaforoCuaterniones, portMAX_DELAY);
    //Hacemos una copia rápida local para soltar el semáforo cuanto antes
    float copiaCuaterniones[NUM_DEDOS_ACTIVOS][3][4];
    for(int d=0; d<NUM_DEDOS_ACTIVOS; d++)
    {
      for(int f=0;f<3; f++){
        for(int i=0;i<4;i++){
        copiaCuaterniones[d][f][i]=cuaterniones[dedosActivos[d]][f][i];
        }
      }
    }
    //Liberamos el semáforo
    xSemaphoreGive(semaforoCuaterniones);


    int offset=0;

    //3. Construccion del paquete binario
    for(int d=0;d<NUM_DEDOS_ACTIVOS;d++){
      for(int f=0; f<3; f++){
        bufferBinario[offset++]=0xAA;    //Byte 0:Sincronismo
        bufferBinario[offset++]=dedosActivos[d];    //Byte 1: Dedo
        bufferBinario[offset++]=(uint8_t)f;         //Byte 2: Falange

        //Bytes 3-18: los 4 floats copiados a nivel de bytes
        memcpy(&bufferBinario[offset],&copiaCuaterniones[d][f][0],16);
        offset+=16;
        //Byte 19: Checksum (XOR de los bytes 1 a 18)
        uint8_t checksum=0;
        for(int k=offset -18; k<offset; k++){
          checksum^=bufferBinario[k];
        }
        bufferBinario[offset++]=checksum;
      }
    }
    //==================================
    //ENVÍO POR BLUETOOTH --> TRAMA A COMPROBAR
    //==================================
    SerialBT.write(bufferBinario, offset);
    //===================================
    //ENVÍO SERIAL --> TRAMA VERDADEREA
    //===================================
    Serial.printf("[CABLE] Anular F0 -> Qw:%.4f, Qx:%.4f, Qy:%.4f, Qz:%.4f\n",
    copiaCuaterniones[3][0][0], copiaCuaterniones[3][0][1], copiaCuaterniones[3][0][2], copiaCuaterniones[3][0][3]);
    Serial.printf("[CABLE] Anular F1 -> Qw:%.4f, Qx:%.4f, Qy:%.4f, Qz:%.4f\n",
    copiaCuaterniones[3][1][0], copiaCuaterniones[3][1][1], copiaCuaterniones[3][1][2], copiaCuaterniones[3][1][3]);
    Serial.printf("[CABLE] Anular F2 -> Qw:%.4f, Qx:%.4f, Qy:%.4f, Qz:%.4f\n",
    copiaCuaterniones[3][2][0], copiaCuaterniones[3][2][1], copiaCuaterniones[3][2][2], copiaCuaterniones[3][2][3]);
  }
  
}

void setup() {
  delay(200);                     //damos tiempo a que despierte el esp32 y se estabilice
  Serial.begin(115200);
  SerialBT.begin("Guante_TFG_Pablo");   //Nombre visible para emparejar
  Serial.println("Bluetooth Iniciado. Listo para emparejar.");
  liberarbusI2C();
  //1. Iniciar los buses hardware
  iniciabuses();
  delay(500);

  //2.Forzamos el cierre manual de todos los multiplexores por seguridad
  Serial.println("\n---RESETEANDO LOS MULTIPLEXORES----");
  reiniciar_multiplexores();
  
  //3. Escaneo inicial (sólo deberían verse multiplexores 0x70)
  Serial.println("\n---ESCANEO 1 (Buscando Multiplexores 0x70) ---");
  escaneobusI2C();
  delay(1000);

  //4.Configuración de los dedos activos
  for (int d=0; d<NUM_DEDOS_ACTIVOS; d++)
  {
    uint8_t dedo= dedosActivos[d];
    Serial.println("\n =======================================================");
    Serial.print("\n                    CONFIGURANDO DEDO NÚMERO: ");
    Serial.println(dedo);
    Serial.println("\n =======================================================");
    configurar_dedo(dedo);
    //0.0873f rad/s = 5 grados/s, y 0.00349f rad/s^2 = 0.2 grados/s/s: son los
    //valores de partida que usa el propio paper en su Seccion 4.4
    madgwick_init(&filtros[dedo][F0], 100.0f, 0.0873f, 0.00349f);
    madgwick_init(&filtros[dedo][F1], 100.0f, 0.0873f, 0.00349f);
    madgwick_init(&filtros[dedo][F2], 100.0f, 0.0873f, 0.00349f);
  }

  

  //5. Escaneo final: Debemos ver que todo ha quedado abierto
  Serial.println("---ESCANEO 2 (Deben verse 0x70, 0x69 y 0x0C para el último dedo)---");
  escaneobusI2C();
  delay(1000);

  
  /*//PASO 2: CALIBRACIÓN DEL SESGO DEL GIROSCOPIO
  Serial.println("\nDejar mano COMPLETAMENTE QUIETA (apoyada, sin tocarla).");
  Serial.println("Escribe cualquier caracter y pulsa ENTER para calibrar el giroscopio...");
  while (!Serial.available()){
    delay(50);
  }
  while (Serial.available()){
    Serial.read();
  }
  calibrar_sesgo_giroscopio(dedosActivos, NUM_DEDOS_ACTIVOS, 20000);   //20s quieta
  



  //Comentado porque ya está calibrado y puesto de fábrica
  //---PASO 3: CALIBRACIÓN
  Serial.println("\n Escribir cualquier caracter en el monitor serie para comenzar la calibración.");
  while(!Serial.available())
  {
    delay(50);
  }
  while(Serial.available())
  {
    Serial.read();    //vaciamos el buffer de entrada
  }
  calibrar_magnetometro_mano_completa(dedosActivos, NUM_DEDOS_ACTIVOS, 60000);
  */

  
  //Arranca la ventana de sembrado con magnetómetro para F0 - la mano
  //debería estar quieta durante los próximos DURACION_SEMBRADO_MS
  Serial.println("\nSembrando rumbo inicial con magnetómetro (F0 de cada dedo).");
  Serial.println("Deja la mano quieta unos 5 segundos...");
  tiempoArranqueFiltros = millis();


  //---CONFIGURACIÓN FREERTOS ---
  Serial.println("\nIniciando Tareas Dual-Core...");
  semaforoCuaterniones=xSemaphoreCreateMutex();   //Damos estructura de semáforo
  //Creamos la Tarea de Comunicación en el Núcleo 0 (Prioridad 1 - Más Baja)
  
  xTaskCreatePinnedToCore(
    TareaComunicacion,
    "Comunicacion",
    4096,                   //Tamaño de la memoria asignada (Stack)--> 4096 bytes suficientes para los 300 bytes de datos y hueco por si acaso
    NULL,
    1,                      //Prioridad
    &handleComunicacion,    //Necesario para que sea notificada de cuándo ejecutarse
    0                       //Se ejecuta en el núcleo 0
  );
  //Crear tarea de Fusión en el Núcleo 1 (Prioridad 2 - Más alta)
  xTaskCreatePinnedToCore(
    TareaSensorFusion,    //Nombre de la funcion
    "Fusion",             //Nombre identificativo
    4096,                 //Tamaño de la memoria asignada (Stack)
    NULL,                 //Parámetros
    2,                    //Prioridad
    NULL,                 //Handle
    1                     //Núcleo 1
  );
  
  Serial.println("==================SISTEMA DUALCORE LISTO=================\n");
}


void loop() {
  //El loop corre por defecto en el Núcleo 1
  //Como ya existen las tareas, este loop debe desaparecer para liberar recursos
  vTaskDelete(NULL);
}
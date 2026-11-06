#ifndef ICM20948_REGISTERS_H
#define ICM20948_REGISTERS_H

// Direcciones I2C del ICM20948
#define ICM20948_ADDR_L 0x68            //AD0=LOW='0'=GND --> No se va a usar en esta aplicación
#define ICM20948_ADDR_H 0x69            //Dirección I2C AD0=HIGH='1'=1.8V --> Todos están configurados por hardware para que tengan esta dirección
#define AK09916 0x0C                    //Dirección I2C del magnetómetro

// Bancos de registros
#define REG_BANK_SEL  0x7F              //Selección de banco de registro
#define USER_BANK_0 0x00                //Banco 0
#define USER_BANK_2 0x20                //Banco 2
#define USER_BANK_3 0x30                //Banco 3

//Registros del BANCO 0
#define REG_WHO_AM_I  0x00              //Identificación del dispositivo
#define REG_PWR_MGMT_1  0x06            //Gestión del reloj
#define REG_PWR_MGMT_2  0x07            //Habilitación de ejes de aceleración y giro
#define LP_CONFIG   0x05                // Configuración Bajo Consumo (Para I2C_MST_CYCLE) y que no se quede pillado al hacer un reset caliente
#define USER_CTRL 0x03                  //Desactivar el I2C_Master_Mode
#define INT_PIN_CFG 0x0F                //Habilitar el ByPass_Mode
#define I2C_MST_STATUS  0x17            //Para el pollling del SLV4
#define ACCEL_XOUT_H 0x2D               //Inicio de los datos acelerómetro
#define EXT_SLV_SENS_DATA_00  0x3B      //Inicio de los datos esclavos

//Registros del BANCO 2 --> Usado para DLFP (Filtro paso Baja)
#define GYRO_SMPLRT_DIV 0x00            //no usado por ahora
#define GYRO_CONFIG_1   0x01            //DLPFCFG + FS_SEL + FCHOICE (gyro)
#define ACCEL_SMPLRT_DIV_1 0x010        //no usado por ahora
#define ACCEL_SMPLRT_DIV_2 0x11         //no usado por ahora
#define ACCEL_CONFIG    0x14            //DLPFCFG + FS_SEL + FCHOICE (accel)
// Estructura de GYRO_CONFIG_1 y ACCEL_CONFIG
// bit:   7   6   5   4   3   2   1    0
//       [ - | - |  DLPFCFG|  FS_SEL |FCHOICE]
//DLPFCFG=6 (filtro paso baja -3dB en 5.7 Hz)
//F_SEL=00 --> por defecto
//FCHOICE=1 //Permite el filtrado
//0b00110000 | 0b00000000 | 0b00000001  =   0b00110001
//Por lo que GYRO_CONFIG_1=ACCEL_CONFIG=0x31
#define DLPF_CFG_5_7HZ_RANGO_DEFECTO 0x31
//Los 5.7Hz generan retraso,
//Es mejor acelerarlo a 119Hz (DLPDCFG=2) y F_SEL=10 (+-1000dps)
#define DLPF_CFG_RAPIDO_1000DPS 0x15
//Escalón intermedio, filtrado a 51.2Hz pero con +-1000dps
#define DLPF_CFG_MEDIO_1000DPS 0x1D




//Registros del BANCO 3
#define I2C_MST_CTRL  0x01
//Los canales del 0 al 3 son para hacer lectura/escritura de forma cíclica y automática (usado para leer datos)
//el canal SLV0 para leer el magnetómetro 0x0C
#define I2C_SLV0_ADDR  0x03
#define I2C_SLV0_REG  0x04
#define I2C_SLV0_CTRL  0x05
//el canal SLV1 para leer la Falange 1 (0x68)
#define I2C_SLV1_ADDR  0x07
#define I2C_SLV1_REG  0x08
#define I2C_SLV1_CTRL  0x09
//el canal SLV2 para usarlo como canal basura para leer el ST2 del magnetómetro de F0
#define I2C_SLV2_ADDR  0x0B
#define I2C_SLV2_REG  0x0C
#define I2C_SLV2_CTRL  0x0D
//el canal SLV4 es one-shot, usado para configurar inicialmente el SLV0 y SLV1
#define I2C_SLV4_ADDR  0x13
#define I2C_SLV4_REG  0x14
#define I2C_SLV4_CTRL  0x15
#define I2C_SLV4_DO  0x16

//Registro del que empezamos a leer el magnetómetro AK09916
#define HXL 0x11
#define CNTL2 0x31

#endif //ICM20948_REGISTERS_H
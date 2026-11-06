//BluetoothReceiver.cs
//1. Recibe los cuaterniones del ESP32 por Bluetooth
//2. Valida la trama binaria con checksum
//3. Aplica rotaciones al esqueleto de la mano
//Autor: Pablo López de Murillas García
//=================================================
using UnityEngine;
using System.IO.Ports;
using System.Threading;
using System;
using Quaternion = UnityEngine.Quaternion;
using System.Diagnostics;
using Debug = UnityEngine.Debug;
using System.Runtime.CompilerServices;
using TMPro;        //Necesario para el texto

//Estructura vincular dedo-falange-hueso unity
[System.Serializable]
public class MapeoFalange
{
    public int dedo;        //0-Pulgar, 1-Índice, 2-Corazón, 3-Anular, 4-Meñique
    public int falange;     //0-F0  1-F1    2-F2
    public Transform hueso;     //Vínculo de unity: Index 1, Index 2...
}
public class BluetoothReceiver : MonoBehaviour
{
    public string puertoCOM = "COM7";          //Puerto del ESP32 . Cambiar según aparezca vinculado en el PC
    public int baudRate = 115200;
    public float velocidadSuavizado=15f;     //Factor de suavizado para SLERP: Si subimos la velocidad, sigue más rapido al sensor, es decir, menos suavizado

    //Aquí deben aparecer los objetos vinculados
    public MapeoFalange[] mapeoHuesos;
    public bool CALIBRAR_AHORA=false;

    public bool HOMING_AHORA=false;
    public GameObject panelAviso;       //Se debe arrastrar aquí el panel indicador
    public TextMeshProUGUI textoAviso;
    
    private bool homingEnProgreso=false;

    //Variables para el Homing
    private const float TIEMPO_PREPARACION_SEGUNDOS=3f;     //Tiempo para llevar la mano a la posición inicial y evitar calibrar la mano en otra posición distinta
    private const float DURACION_SEMBRADO_SEGUNDOS=5f;      //5 segundos
    private const float MARGEN_SEGURIDAD_SEGUNDOS=1f;       //debido al retardo envío-recepción del Bluetooth

    private SerialPort puertoSerie;
    private Thread hiloLectura;
    private bool leyendo = false;
    private object lockObject = new object();

    //Matrices para almacenar hasta 6 dedos con 3 falanges
    private Quaternion[,] orientacionesRecibidas = new Quaternion[6,3];
    private Quaternion[,] offsetcalibracion = new Quaternion[6,3];
    private bool[,] calibrado = new bool[6,3];

    private bool[,] datosRecibidos= new bool[6,3];

    //Variables para guardar la posición original del modelo
    //Necesario para hacer el homing --> llevar la mano a la posición de origen
    private Quaternion[,] rotacionMundialOriginal=new Quaternion[6,3];
    private Quaternion[,] rotacionLocalOriginal=new Quaternion[6,3];

    private int contadorPaquetes=0;
    private float ultimoPrintHz= 0f;


    void Start()
    {
        //1.Iniciar matrices por seguridad
        for(int i=0;i<6;i++)
        {
            for( int j=0;j<3;j++)
            {
                orientacionesRecibidas [i,j] = Quaternion.identity;
                offsetcalibracion[i,j] = Quaternion.identity;
                calibrado[i,j]=false;
                datosRecibidos[i,j]=false;
            }
        }
        foreach (var m in mapeoHuesos)
        {
            if (m.hueso == null || m.dedo >=6 || m.falange >= 3)
            {
                continue;
            }
            rotacionMundialOriginal[m.dedo , m.falange] = m.hueso.rotation;
            rotacionLocalOriginal[m.dedo , m.falange] = m.hueso.localRotation;
        }
        try
        {
            puertoSerie = new SerialPort(puertoCOM, baudRate);
            puertoSerie.ReadTimeout = 50;                                       //50ms de espera, suficiente para detectar la ausencia de datos
            puertoSerie.Open();
            leyendo = true;
            hiloLectura = new Thread(LeerDatosBinarios);
            hiloLectura.IsBackground = true;
            hiloLectura.Start();
            Debug.Log("Conectado al " + puertoCOM);
        }
        catch (Exception e) { Debug.LogError("Error: " + e.Message); }
    }

    //Remapeo de los ejes para ejes de Unity
    private Quaternion RemapEjes(float qw, float qx, float qy, float qz)
    {
        return new Quaternion(-qy, -qz, qx, qw);                            //Remapeo de ejes, ya que el ICM-20948 produce los cuaterniones (X=Norte, Y=Este, Z=Abajo)
                                                                            //Y en Unity hay que calibrar los ejes a mano hasta que corresponden con los movimientos 
    }

    void LeerDatosBinarios()
    {
        byte[] buffer = new byte[20];

        while (leyendo && puertoSerie != null && puertoSerie.IsOpen)
        {
            try
            {
                if (puertoSerie.ReadByte() == 0xAA)
                {
                    int bytesLeidos = 0;
                    while (bytesLeidos < 19)
                    {
                        bytesLeidos += puertoSerie.Read(buffer, bytesLeidos + 1, 19 - bytesLeidos);
                    }
                    //VALIDACIÓN DEL CHECKSUM
                    byte checksumcalculado=0;
                    for(int k=1;k<19;k++)
                    {
                        checksumcalculado ^=buffer[k];      //XOR
                    }
                    if(checksumcalculado==buffer[19])
                    {
                        int dedo = buffer[1];
                        int falange = buffer[2];
                        
                        //Dejar pasar todos los bytes del ESP32 que correspondan con datos de dedo-falange

                        if (dedo <6 && falange <3)
                        {
                            float qw = BitConverter.ToSingle(buffer, 3);
                            float qx = BitConverter.ToSingle(buffer, 7);
                            float qy = BitConverter.ToSingle(buffer, 11);
                            float qz = BitConverter.ToSingle(buffer, 15);

                            lock (lockObject)
                            {
                                //Si el ESP32 manda todo ceros es que no se ha conectado todavía
                                if(qw!=0 || qx!=0 || qy!=0 || qz!=0)
                                {
                                    //Guardar los cuaterniones recibidos
                                    orientacionesRecibidas[dedo, falange] = RemapEjes(qw, qx, qy, qz);
                                    datosRecibidos[dedo,falange]=true;          //Confirmamos que hemos recibido datos
                                    contadorPaquetes++;
                                }
                                
                            }
                        }
                    }
                    else
                    {
                        //AVISO DE QUE NO SE HAN RECIBIDO LOS DATOS QUE SE HAN ENVIADO (DISTORSIONADOS)
                        Debug.LogWarning($"[BLUETOOTH] Trama corrupta ignorada. Checksum esperado: {checksumcalculado}, Recibido: {buffer[19]}");
                    }
                    
                }
            }
            catch (TimeoutException) { }
            catch (Exception) { }
        }
    }

    void Update()
    {
        if(Time.time-ultimoPrintHz>=1f)
        {
            Debug.Log($"[BLUETOOTH] Paquetes válidos/seg: {contadorPaquetes}");
            contadorPaquetes=0;
            ultimoPrintHz=Time.time;
        }
        //FASE 1 :HOMING
        if (HOMING_AHORA && !homingEnProgreso)
        {
            HOMING_AHORA=false;
            StartCoroutine(RutinaHoming());
        }
        //FASE 2: Capturar calibracion de pose al pulsarcasilla
        if (CALIBRAR_AHORA)
        {        
            lock (lockObject)
            {
                foreach(var m in mapeoHuesos)
                {
                    if(m.hueso == null) continue;
                    if(m.dedo>=6 || m.falange>=3) continue;
                    if(!datosRecibidos[m.dedo, m.falange])
                    {
                        Debug.LogWarning($"Hueso {m.hueso} Ignorado: El ESP32 no ha mandado datos");
                        continue;
                    }
                    if (m.falange == 0)
                    {
                        //F0: Posición absoluta
                        Quaternion qSensorReposo = orientacionesRecibidas[m.dedo, m.falange];
                        Quaternion qHuesoReposo = rotacionMundialOriginal[m.dedo, m.falange];   //Usamos la rotacion mundiol del rig
                        //Calcular el offset (error entre sensor y hueso)
                        offsetcalibracion[m.dedo, m.falange]=qHuesoReposo*Quaternion.Inverse(qSensorReposo);

                    }
                    else
                    {
                        //F1 y F2: Relativo al sensor de la falange anterior
                        //F2----relativo a ---->F1----relativo a---->F0
                        if(!datosRecibidos[m.dedo, m.falange-1])
                        {
                            Debug.LogWarning($"Hueso {m.hueso} Ignorado: falta el dato del sensor maestro (falange{m.falange-1})");
                            continue;
                        }
                        Quaternion qSensorMaestroReposo = orientacionesRecibidas[m.dedo,m.falange-1];
                        Quaternion qSensorReposo = orientacionesRecibidas[m.dedo,m.falange];
                        Quaternion qRelativoReposo = Quaternion.Inverse(qSensorMaestroReposo) * qSensorReposo;
                        Quaternion qHuesoLocalReposo = rotacionLocalOriginal[m.dedo, m.falange];
                        offsetcalibracion[m.dedo, m.falange] = qHuesoLocalReposo * Quaternion.Inverse(qRelativoReposo);
                    }
                    calibrado[m.dedo, m.falange]=true;
                }   
                Debug.Log("Calibracion completada para todos los huesos mapeados");
            }
            CALIBRAR_AHORA=false;
        }
    }
    //Rutina del homing
    private System.Collections.IEnumerator RutinaHoming()
    {
        homingEnProgreso=true;
        //Encendemos el panel
        if (panelAviso != null)
        {
            panelAviso.SetActive(true);
        }
        //0. Preparar (Tiempo para poner la mano plana tras hacer click en homing)
        float tPrep=0;
        while (tPrep < TIEMPO_PREPARACION_SEGUNDOS)
        {
            tPrep+=Time.deltaTime;
            if(textoAviso!=null)
            {
                textoAviso.text= $"Llevar mano a la posición inicial...\n{Mathf.CeilToInt(TIEMPO_PREPARACION_SEGUNDOS - tPrep)}s";
            }
            yield return null;
        }
        //1. Avisar al ESP32 de rearmar F0
        try
        {
            if(puertoSerie!=null && puertoSerie.IsOpen)
            {
                puertoSerie.Write(new byte[]{(byte) 'H'},0,1);
            }
        }
        catch(Exception e)
        {
            Debug.LogWarning("No se pudo enviar el comando de Homing:"+ e.Message);
        }
        //2.Mostrar el panel indicador
        float tiempoEspera = DURACION_SEMBRADO_SEGUNDOS + MARGEN_SEGURIDAD_SEGUNDOS;

        float t=0f;
        while(t < tiempoEspera)
        {
            t+=Time.deltaTime;
            if(textoAviso != null)
            {
                textoAviso.text= $"Mantén la mano quieta ...\n {Mathf.CeilToInt(tiempoEspera-t)}s";
            }
            yield return null;
        }
        //Apagar el panel
        if(panelAviso!=null)
        {
            panelAviso.SetActive(false);
        }
        //3.Resetear el acumulador de rotación de la palma
        //rotacionPalmaAcumulada = Quaternion.identity;

        //4.Saltamos a CALIBRAR_AHORA (para comenzar en esa posición inicial)
        CALIBRAR_AHORA=true;
        homingEnProgreso=false;

    }


    //Usamos lateupdate para aplicar la rotacion despues de que unity dibuje las animaciones base
    void LateUpdate()
    {
        lock (lockObject)
        {
            foreach(var m in mapeoHuesos)
            {
                //Si hay Homing
                if (homingEnProgreso)
                {
                    if (m.falange ==0)
                    {
                        m.hueso.rotation=Quaternion.Slerp(m.hueso.rotation, rotacionMundialOriginal[m.dedo, m.falange], Time.deltaTime * velocidadSuavizado);
                    }
                    else
                    {
                        m.hueso.localRotation = Quaternion.Slerp(m.hueso.localRotation, rotacionLocalOriginal[m.dedo, m.falange], Time.deltaTime * velocidadSuavizado);
                    }
                    continue; 
                }
                //Si no hay homing:
                if (m.hueso == null || !calibrado[m.dedo, m.falange]) continue;
                if(m.falange==0)
                {
                    Quaternion objetivo = offsetcalibracion[m.dedo, m.falange]*orientacionesRecibidas[m.dedo, m.falange];
                    //Por cada frame, el hueso avanza una fracción (proporcional a Time.deltaTime*velocidadSuavizado) para que así no haya saltos
                    //y que vaya de una forma mucho más fluida
                    //Bajarlo implica que es muy rapido y queremos que sea más lento
                    //Subirlo implica que es muy lento y queremos que sea más rapido
                    m.hueso.rotation = Quaternion.Slerp(m.hueso.rotation, objetivo, Time.deltaTime*velocidadSuavizado);
                }
                else
                {
                    if(!datosRecibidos[m.dedo, m.falange-1]) continue;
                    Quaternion qRelativoActual = Quaternion.Inverse(orientacionesRecibidas[m.dedo,m.falange-1]) * orientacionesRecibidas[m.dedo,m.falange];
                    Quaternion objetivo = offsetcalibracion[m.dedo, m.falange] * qRelativoActual;
                    m.hueso.localRotation=Quaternion.Slerp(m.hueso.localRotation, objetivo,Time.deltaTime*velocidadSuavizado);
                
                }
            }
        }
    } 
    //Para ver los botones en scene
    public void OnClickCalibrar()
    {
        CALIBRAR_AHORA=true;
    }
    public void OnClickHoming()
    {
        HOMING_AHORA=true;
    }
    void OnApplicationQuit()
    {
        leyendo = false;
        if (hiloLectura != null) hiloLectura.Join(500);
        if (puertoSerie != null && puertoSerie.IsOpen) puertoSerie.Close();
    }
    
}

/**
 * @file respi_Esp_Mit.ino
 * @author HOMTEC Team
 * @brief 
 * @version 1.0
 * @date 2020-08-26
 * 
 * @copyright GPLv3
 * 
 * 
 * 
 * 
 * \fn void readIncomingMsg(void)
 * \brief  Esta función es la encargada de implementar el protocolo de comunicación con la interfaz gráfica. 
 * \details Permite configurar valores tipicos en los procesos respiratorio y realiza una retransmisión de hacia la interfaz como chequeo de parámetros. Formato: "CONFIG %d %d %d %d %d %d %d %d %d#"
 * \param ninguno 
 * \return ninguno
 * 
 * \fn loop
 * \brief Función de bocle principal del microcontrolador
 * \details A través de valores temporizado de menor prioridad que los timers, realiza actualización grafica y de valores medidos a la interfaz cada 100ms. Además cada 10 minutos actualiza los ciclos de la maquina.
 * \param ninguno
 * \return ninguno
 * 
 * \fn setup
 * \brief Función de inicio principal del microcontrolador
 * \details Configura: los baudios de los dos puertos series de comnunicación, pines digitales de entreda-salida, inicializa sensores, PID (obsoleto), la máquina de estado MechVentilation y sus opciones.
 * Maneja valores grabados en EEPROM y determina el inicio normal con calibración y valores por defectos, o el inicio rápido de emergencia con valores previos seteados.
 * Actualiza valores para la interfaz. Configura temporizadores a 40us y 20ms para el motor paso a paso y para la maquina de estado, respectivamente.
 * \param ninguno
 * \return ninguno
 * 
 * \var FlexyStepper *stepper 
 * \brief Puntero a clase que maneja motor paso a paso.
 * 
 * \var Sensors *sensors
 * \brief Puntero a clase que maneja los sensores de presion, flujo, y otros eventualmente.
 * 
 * \var MechVentilation *ventilation
 * \brief Puntero a clase que maneja la maquina de estado principal del proceso respiratorio y del respirador en si mismo.
 * 
 * \var VentilationOptions_t options
 * \brief Clase que configura las opciones por default de microcontrolador.
 * 
 * \var uint16_t dirEPROM
 * \brief Direcció EEPROM donde se guarda la dirección de la estructura CiclosEPROM.
 * 
 * \var CiclosEPROM st_Ciclos
 * \brief Estructura que contiene la cantidad de ciclos y la cantidad de grabaciones en EEPROM.
 * 
 * \var uint8_t pim 
 * \brief Variable de configuración de Presión inspiratoria máxima.
 * 
 * \var uint8_t peep
 * \brief Variable de configuración de Presión positiva al final de la expiración (en desuso). 
 * 
 * \var uint8_t frecResp
 * \brief Variable de configuración de Frecuencia respitatoria.
 * 
 * \var uint8_t ier
 * \brief Variable de configuración de Relación inspiración expiración.
 * 
  * \var uint8_t vt 
 * \brief Variable de configuración de Volumen Tidal.
 * 
 * \var uint8_t thr 
 * \brief Variable de configuración de Umbral de detección de inspiración.
 * 
 * \var uint8_t pause 
 * \brief Variable de configuración de Pausa respiratoria.
 * 
 * \var uint8_t encend 
 * \brief Variable de configuración de Encendido-Apagado de respirador.
 * 
 * \var uint8_t tins
 * \brief Variable de configuración de Tiempo de inspiración.
 * 
 */

/**
 * Dependencies
 */

#include "defaults.h"
#include "calc.h"
#include "Sensors.h"
#include <EEPROM.h>
#include "MechVentilation.h"
#include "mpx.h"
#include "src/AutoPID/AutoPID.h"
#include "src/FlexyStepper/FlexyStepper.h"
#include "src/TimerOne/TimerOne.h"
#include "src/TimerThree/TimerThree.h"
#include <avr/wdt.h>

//void softwareReset( uint8_t prescaller) {
//  // start watchdog with the provided prescaller
//  wdt_enable( prescaller);
//  // wait for the prescaller time to expire
//  // without sending the reset signal by using
//  // the wdt_reset() method
//  while(1) {}
//}


#if DEBUG_STATE_MACHINE
volatile String debugMsg[15];
volatile byte debugMsgCounter = 0;
#endif

FlexyStepper *stepper = new FlexyStepper();
Sensors *sensors;
AutoPID *pid;
MechVentilation *ventilation;
VentilationOptions_t options;

volatile uint16_t dirEPROM;
CiclosEPROM st_Ciclos;

/**
 * Read commands
 */
uint8_t pim, peep, frecResp, ier, vt, thr, pause, encend, tins;
void readIncomingMsg(void){
    char msg[50];
    Serial2.readStringUntil('#').toCharArray(msg, 50);
    //Serial.println(msg);

  
    if (String(msg).substring(0, 6) == "CONFIG")
    {
            delayMicroseconds(300);
            int rc = sscanf(msg, "CONFIG PEEP %d", &peep); 
            if (rc == 1)
            {
               Serial.print(peep); 
               ventilation->setPeakEspiratoryPressure(peep);
               
            }
            else
            {
            int rc = sscanf(msg, "CONFIG PIP %d", &pim);  
            if (rc == 1)
            {
               EEPROM.put(4, (unsigned char)pim);
               ventilation->setPeakInspiratoryPressure(pim);
            }
            else
            {
                int rc = sscanf(msg, "CONFIG BPM %d", &frecResp);
                if (rc == 1)
                {
                     EEPROM.put(5, (unsigned char)frecResp);
                     ventilation->setRPM(frecResp);
                     
                }
                else
                {
                  int rc = sscanf(msg, "CONFIG TDAL %d", &vt);
                  if (rc == 1)
                  {
                      EEPROM.put(6, (unsigned char)vt);
                      //ventilation->setVolTidalInStep(vt);
                      ventilation->setVolTidalByArray(vt);
                  }
                  else
                  {
                    int rc = sscanf(msg, "CONFIG IER %d", &ier);
                    if (rc == 1)
                    {
                        EEPROM.put(11, (unsigned char)ier);
                        ventilation->setIer(ier);
                    }
                    else
                    {
                      int rc = sscanf(msg, "CONFIG THR %d", &thr);
                      if (rc == 1)
                      {
                        EEPROM.put(8, (unsigned char)thr);
                        ventilation->setTh(thr);
                      }
                      else
                        {
                        int rc = sscanf(msg, "CONFIG PAU %d", &pause);
                        if (rc == 1)
                        {
                          EEPROM.put(9, (unsigned char)pause);
                          ventilation->setPause(pause);
                        }else{
                        
                           int rc = sscanf(msg, "CONFIG TINS %d", &tins);
                          if(rc == 1){
                            EEPROM.put(10, (unsigned char)tins);
                            ventilation->setInsuflationTime(tins);
                        }
                        else{//----------------------------------------------------
                          int rc = sscanf(msg, "CONFIG PWR %d", &encend);
                          if (rc == 1)
                          {
                            EEPROM.put(3, (unsigned char)encend);
                            EEPROM.put(12, (unsigned char)encend);
                            EEPROM.put(13, (unsigned char)encend);
                            if (encend == 1){ 
                              ventilation->start();
                            }
                            else{
                              if(encend == 0)
                                ventilation->stopMech();
                            }
                          }
                      }//--------------------------------------------------------------------
                    }
                    }
                  }
                }
            }
        }
        }
    }
    else if (String(msg).substring(0, 7) == "RECRUIT")
    {
        uint8_t tmp = 255;
        int rc = sscanf(msg, "RECRUIT %d", &tmp);
        switch (tmp)
        {
        case 0:
            //Serial.println("ACK 0");
            ventilation->deactivateRecruitment();
            break;
        case 1:
            //Serial.println("ACK 1");
            ventilation->activateRecruitment();
            break;
        default:
            break;
        }
    }
    
      char string[50];
      sprintf(string, "CONFIG %d %d %d %d %d %d %d %d %d#", ((int)ventilation->getPeakInspiratoryPressure()),
                                        //((int)ventilation->getPeakEspiratoryPressure()), 
                                        (((int)ventilation->getInsuflationTimeApp())/10),
                                        ((int)ventilation->getRPM()), 
                                        ((int)ventilation->getIer()),
                                        //((int)ventilation->getVolTidalPorcen()),
                                        ((int)ventilation->getVolTidalByArray()),
                                        ((int)ventilation->getTh()),
                                        ((int)ventilation->getPause()),
                                        ((int)((st_Ciclos.ciclos)/100)),
                                        ((int)ventilation->getRunning())
                                        );
        delay(1);
        Serial2.print(string); 
        //Serial.print(string); 
         
}



void setup()
{
    // Puertos serie
    Serial.begin(57600);//57600);
    Serial2.begin(57600);//115200);
    //Serial.println(F("Setup"));

    // Zumbador
    pinMode(PIN_BUZZ, OUTPUT);
//    digitalWrite(PIN_BUZZ, HIGH); // test zumbador
//    delay(100);
//    digitalWrite(PIN_BUZZ, LOW);

    // FC con pulsador 
    // pinMode(PIN_STEPPER_ENDSTOP, INPUT_PULLUP); // el sensor tipo pulsador da un 0 cuando detecta
    
    // FC efecto hall
    pinMode(PIN_STEPPER_ENDSTOP, INPUT); // el sensor de efecto hall da un 1 cuando detecta

    // Solenoid
    pinMode(PIN_SOLENOID1, OUTPUT);
    pinMode(PIN_SOLENOID2, OUTPUT);
    
    // Sensores de presión
    sensors = new Sensors();
    int check = sensors->begin();

    // PID obsoleto solo por compatibilidad
    pid = new AutoPID(PID_MIN, PID_MAX, PID_KP, PID_KI, PID_KD);

    // Parte motor
    pinMode(PIN_STEPPER_EN, OUTPUT);
    digitalWrite(PIN_STEPPER_EN, HIGH);

    // TODO: Añadir aquí la configuarcion inicial desde puerto serie
    // Opcion no usada pero necesaria para la clase MechVentilation
    options.height = DEFAULT_HEIGHT;
    options.sex = DEFAULT_SEX;
    options.respiratoryRate = DEFAULT_RPM;
    options.peakInspiratoryPressure = DEFAULT_PEAK_INSPIRATORY_PRESSURE;
    options.peakEspiratoryPressure = DEFAULT_PEAK_ESPIRATORY_PRESSURE;
    options.triggerThreshold = DEFAULT_TRIGGER_THRESHOLD;
    options.hasTrigger = true; // va en false como siempre, agregara a la app opcion

    ventilation = new MechVentilation(
        stepper,
        sensors,
        pid,
        options
    );
    #if DEBUG_UPDATE
    Serial.println("Tiempo del ciclo (mseg):" + String(ventilation->getExsuflationTime() + ventilation->getInsuflationTime()));
    Serial.println("Tiempo inspiratorio (mseg):" + String(ventilation->getInsuflationTime()));
    Serial.println("Tiempo espiratorio (mseg):" + String(ventilation->getExsuflationTime()));
    #endif
    // TODO: Esperar aqui a iniciar el arranque desde el serial
    // No espera nada
/////////////////// Manejo de ciclo de trabajo
    EEPROM.get(0, dirEPROM);
    EEPROM.get(dirEPROM, st_Ciclos);
    if(st_Ciclos.escriturasEPROM>65000){
       dirEPROM += sizeof(st_Ciclos);
       if(dirEPROM > EEPROM.length()){dirEPROM = 20;}
       EEPROM.put(0, dirEPROM);
       st_Ciclos.escriturasEPROM = 0;
       EEPROM.put(dirEPROM, st_Ciclos);
    }

   
///////////////////// Fin Manejo de ciclo de trabajo 
    char string[50];
    sprintf(string, "CONFIG %d %d %d %d %d %d %d %d %d#", ((int)ventilation->getPeakInspiratoryPressure()),
                                      //((int)ventilation->getPeakEspiratoryPressure()), 
                                      (((int)ventilation->getInsuflationTimeApp())/10),
                                      ((int)ventilation->getRPM()), 
                                      ((int)ventilation->getIer()),
                                      //((int)ventilation->getVolTidalPorcen()),
                                      ((int)ventilation->getVolTidalByArray()),
                                      ((int)ventilation->getTh()),
                                      ((int)ventilation->getPause()),
                                      ((int)(st_Ciclos.ciclos/100)),
                                      ((int)ventilation->getRunning())
                                      );
      Serial2.print(string);
      #if DEBUG_UPDATE
      Serial.print(string); 
      #endif


    // Habilita el motor
    digitalWrite(PIN_STEPPER_EN, LOW);

    // configura la ventilación
    ventilation->start();
    ventilation->update();
    
    delay(1000);

    sensors->readPressure();

    Timer3.initialize(40); //50us
    Timer3.attachInterrupt(timer3Isr);

    Timer1.initialize(TIME_BASE*1000);
    Timer1.attachInterrupt(timer1Isr);

//////////////////////////////////////////////////////////////////////////////////////////////////
// Nuevo 13/7 Activo WatchDogTimer en 60ms
    wdt_enable(WDTO_120MS);
//////////////////////////////////////////////////////////////////////////////////////////////////
    uint8_t encendaux = 0;
    EEPROM.get(3, encend);
    if(encend == 1)
      encendaux++;
    EEPROM.get(12, encend);
     if(encend == 1)
      encendaux++;
    EEPROM.get(13, encend);
    if(encend == 1)
      encendaux++;
    
    if (encendaux > 1){ 
      // Serial.println("Quedo on");
      EEPROM.get(4, pim);
      ventilation->setPeakInspiratoryPressure(pim);
      
      EEPROM.get(5, frecResp);
      ventilation->setRPM(frecResp);
      
      EEPROM.get(6, vt);
      ventilation->setVolTidalByArray(vt);
             
      EEPROM.get(11, ier);
      ventilation->setIer(ier);
      
      EEPROM.get(8, thr);
      ventilation->setTh(thr);
      
      EEPROM.get(9, pause);
      ventilation->setPause(pause);
      
      EEPROM.get(10, tins);
      ventilation->setInsuflationTime(tins);
      
      ventilation->start();
    }
    else{
      //if(encend == 0)
        // Serial.println("Quedo off");
        ventilation->stopMech();
        delay(10*1000);
        sensors->rutinaDeAutoajuste();//<---------------------------------  Ejecuto rutina de autoajuste ------------------------------------------
        EEPROM.update(4, (unsigned char)DEFAULT_PEAK_INSPIRATORY_PRESSURE);
        EEPROM.update(5, (unsigned char)DEFAULT_RPM);
        EEPROM.update(6, (unsigned char)DEFAULT_BYARRAY_VOLTILDAL);
        EEPROM.update(11, (unsigned char)(DEFAULT_INSPIRATORY_FRACTION*100));
        EEPROM.update(8, (unsigned char)DEFAULT_TRIGGER_THRESHOLD);
        EEPROM.update(9, (unsigned char)DEFAULT_PAUSE);
        EEPROM.update(10, (unsigned char)DEFAULT_TINS);
    }
    // attachInterrupt(digitalPinToInterrupt(PIN_STEPPER_ALARM), driverAlarmIsr, FALLING);

}

/////////////////////////////////////////////////////////////////////////////////

///////////////////////////////   LOOP    ///////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////

 
void loop()
{
    unsigned long Mytime;
    static unsigned int contms = 0;
    Mytime = millis();
    unsigned long static lastReadSensor = 0;
    unsigned long static lastSendConfiguration = 0;
    State static lastState;

      /* Status update and reset timer, for next time */


    if (Mytime > lastReadSensor + TIME_SENSOR) {
        sensors->readPressure();
        SensorPressureValues_t pressure = sensors->getRelativePressureInCmH2O();

        sensors->PEEPRobust(pressure);
        
        #if ENABLED_SENSOR_VOLUME_STEPPER
        sensors->readVolume(stepper);
        #else
        sensors->readVolume();
        #endif   
        
        SensorVolumeValue_t volume = sensors->getVolume();
        
        SensorLastPressure_t PEEPPIMval = sensors->getLastPressure();
        uint8_t plateau = ventilation->GetPlateau();
        uint8_t frReal = ventilation->getFrReal();
        unsigned long volgraf = stepper->getCurrentPositionInSteps()*(-1);
        volgraf = (volgraf*13/10) - 166;
        int Alarmas=  ventilation->getAlarms();//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        char string[50];
        sprintf(string, "DT %d %d %d %d %d %d %d #",  ((int)(pressure.pressure1)+1),
                                                  //((int)(PEEPPIMval.maxPressure)+1), 
                                                  ((int)(sensors->getPEEPRobust())), //               <--------------- 9/11/2020 3era revision estaba + 1 paso a + 0
                                                  ((int)(PEEPPIMval.minPressure)+1), 
                                                  ((int)volgraf),//(sensors->getVolumeActual())), //->getFlow() * 1000)),
                                                  ((int)(plateau)+1),
                                                  ((int)(frReal)+1),
                                                  ((int)Alarmas)
                                                  );
       Serial2.print(string);
       //Serial.println(string);
       contms++; // n veces 100ms
       if (contms > TIME_EPROM) {
          contms = 0;
          st_Ciclos.escriturasEPROM++;
          st_Ciclos.ciclos +=  ventilation->GetCiclos();
          ventilation->ResetCiclos();
          EEPROM.put(dirEPROM, st_Ciclos);
//          Inestable
//          EEPROM.update(4, (unsigned char)pim);
//          EEPROM.update(5, (unsigned char)frecResp);
//          EEPROM.update(6, (unsigned char)vt);
//          EEPROM.update(11, (unsigned char)ier);
//          EEPROM.update(8, (unsigned char)thr);
//          EEPROM.update(9, (unsigned char)pause);
//          EEPROM.update(3, (unsigned char)tins);
//          EEPROM.update(10, (unsigned char)encend);
       }
        lastReadSensor = Mytime;
    }
    
    
///////////////////////////////////
// Para implementar serial 2 sino consola serial
    if (Serial2.available()) {
        readIncomingMsg();
    }
//////////////////////////////////    
} // fin loop


/////////////////////////////////////////////////////////////////////////////////

/////////////////////////    RUTINAS DE SERVICIO    /////////////////////////////

/////////////////////////////////////////////////////////////////////////////////

void timer1Isr(void)
{
    ventilation->update();
}

void timer3Isr(void)
{
    stepper->processMovement();
}

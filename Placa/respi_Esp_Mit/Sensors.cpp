/** Definiciones de los métodos para utilizar los sensores de presión y volumen
 *
 * @file Sensors.cpp
 * @details Este archivo contiene las defniciones de las funciones declaradas en Sensors.h
 *
 */
#include "Sensors.h"
#include "mpx.h"
#include <EEPROM.h>

//static Honeywell_ABP* abp;

MPX_PRESSURE_SENSOR *mpx = new MPX_PRESSURE_SENSOR(PIN_MPX_DATA);
//#if ENABLED_SENSOR_VOLUME_byPRESSURE
MPX_PRESSURE_SENSOR *mpxVol = new MPX_PRESSURE_SENSOR(PIN_MPX_FLOW);
//#endif

float pressurecmH2O;


/**
 * @brief   Función Dummy
 *
 * @param   ninguno
 * @return  0
 */
unsigned int Sensors::begin(void) {

    return 0;
}


/**
 * @brief   Funcion que comienza el proceso de inicialización de la clase sensores 
 *
 * @param   ninguno
 * @return  ninguno
 */
Sensors::Sensors(void) {
    _init();
}

/**
 * @brief   Funcion que inicializa la clase sensores 
 *
 * @param   ninguno
 * @return  ninguno
 */
void Sensors::_init () {
  //Sensor de presion MPX5050/5010   
#if ENABLED_SENSOR_MPX
    float aux;
    mpx->MPX_SET_ADC();
    EEPROM.get(14,aux);
    mpx->setOffsetValue(aux);
#endif

    _errorCounter = 0;
    _state =  SensorStateOK;

#if ENABLED_SENSOR_VOLUME_SFM3300
    Wire.begin();
    pinMode(20, INPUT_PULLUP);
    pinMode(21, INPUT_PULLUP);
    _sfm3000 = new SFM3000wedo(64);
    _sfm3000->init();
#endif

#if ENABLED_SENSOR_VOLUME
    resetVolumeIntegrator();
#endif
}


/**
 * @brief   Funcion que reestablece el valor del integrador para el sensor de volumen 
 *
 * @param   ninguno
 * @return  ninguno
 */

void Sensors::rebootVolumeSensor() {
#if ENABLED_SENSOR_VOLUME_SFM3300
    _sfm3000->reset();
    _sfm3000->init();
#endif
#if ENABLED_SENSOR_VOLUME_STEPPER

#endif
#if ENABLED_SENSOR_VOLUME_byPRESSURE

#endif
    resetVolumeIntegrator();
}


/**
 * @brief   Función obsoleta. No usar, no borrar.
 *
 * @param   ninguno
 * @return  Bool
 */

bool Sensors::stalledVolumeSensor()
{
    if (_flowRepetitionCounter > 10)
    {
        _flowRepetitionCounter = 0;
        return true;
    }
    return false;
}


/**
 * @brief   Funcion que lee el sensor de presión MPX 50XX 
 *
 * @param   ninguno
 * @return  ninguno
 */

float Sensors::readPressure() {
    float pres1, pres2;

#if ENABLED_SENSOR_MPX 
    pres1 = mpx->MPX_READ_PRESSURE();          //Volts
    
    pres2 = mpx->MPX_CONVERT_PRESSURE(pres1);   //kPa
    _state=SensorStateOK;
    _pressure1 = pres2;


    return(pres2);
#else
    return(-1);
#endif 

}

/**
 * @brief Función que permite obtener el valor absoluto de presión en Pascales.
 * @param ninguno
 * @return SensorValues_t - values: Valor de presión obtenido convertido a Pascales
 */
SensorPressureValues_t Sensors::getAbsolutePressureInPascals() {
  SensorPressureValues_t values;
#if ENABLED_SENSOR_MPX 
    values.state = _state;
    values.pressure1 = _pressure1;
    values.pressure2 = 0;
#endif
    return values;
}



/**
 * @brief Función que permite obtener el valor relativo de presión en CmH20.
 * @param ninguno
 * @return SensorValues_t - values: Valor de presión obtenido convertido a CmH20
 */

SensorPressureValues_t Sensors::getRelativePressureInCmH2O() {
  SensorPressureValues_t values;
#if ENABLED_SENSOR_MPX 
    values.pressure1 =mpx->MPX_CONVERT_cmH20(_pressure1);
    values.pressure2 = 0;
    values.state = SensorStateOK;
    
    if (_minPressure < values.pressure1)        _minPressure = values.pressure1;
    if (_maxPressure > values.pressure1)        _maxPressure = values.pressure1;
    
//    #if DEBUG_UPDATE
//             Serial.println("Pkp");
//             Serial.print(_pressure1,5);
//             Serial.println("cmh2");
//             Serial.print(values.pressure1,5);
//             Serial.println("");
//#endif
    /*values = getAbsolutePressureInPascals();
    values.pressure1 *= DEFAULT_PA_TO_CM_H2O;
    values.pressure2 *= DEFAULT_PA_TO_CM_H2O;*/
    return values;
#endif

}

/**
 * @brief Función que permite obtener el valor absoluto de presión en CmH20.
 * @param ninguno
 * @return SensorValues_t - values: Valor de presión obtenido convertido a CmH20
 */
SensorPressureValues_t Sensors::getAbsolutePressureInCmH2O() {
   SensorPressureValues_t values;
#if ENABLED_SENSOR_MPX 
    values = getAbsolutePressureInPascals();
    values.pressure1 *= DEFAULT_PA_TO_CM_H2O;
    values.pressure2 *= DEFAULT_PA_TO_CM_H2O;
    return values;
#endif

}


 SensorLastPressure_t lastPres;

 /**
 * @brief Función que permite obtener la última presión leida.
 * @param ninguno
 * @return SensorLastPressure_t - lastPres: Último valor de presión registrado
 */
SensorLastPressure_t Sensors::getLastPressure(void) {
     return lastPres;
}


 /**
 * @brief Función que permite obtener una medida de flujo.
 * @param ninguno
 * @return float flow: valor de flujo obtenido
 */
#if ENABLED_SENSOR_VOLUME
float Sensors::getFlow(void) {
    return _flow;
}

 /**
 * @brief Función que guarda el volumen leído como última medida tomada en la estructura correspondiente.
 * @param ninguno
 * @return ninguno
 */
void Sensors::saveVolume(void) {
_lastVolume = _volume_ml;
}
    

 /**
 * @brief Función que lee volumen y lo guarda en la estructura correspondiente.
 * @param ninguno
 * @return ninguno
 */
void Sensors::readVolume(
#if ENABLED_SENSOR_VOLUME_STEPPER
    FlexyStepper *stepper
#else
    void
#endif  
  ) {
    #if ENABLED_SENSOR_VOLUME_SFM3300
        SFM3000_Value_t tmp = _sfm3000->getvalue(); //TODO crc
        if (tmp.crcOK) {
            _volumeState = SensorStateOK;
        } else {
            _volumeState = SensorStateFailed;
        }
        float flow = ((float)tmp.value - SFM3300_OFFSET) / SFM3300_SCALE; //lpm

        // Record flow samples
        if (flow == _flow && flow != 0)
        {
            _flowRepetitionCounter++;
        } else {
            _flowRepetitionCounter = 0;
        }
        _flow = flow;


        unsigned short mseconds = (unsigned short)(millis() - _lastReadFlow);
        float ml = flow * mseconds / 60; // l/min * ms * 1000 (ml) /60000 (ms)
        _volume_ml += ml;
        _lastReadFlow = millis();
    #endif
#if ENABLED_SENSOR_VOLUME_STEPPER
        {
           float positionApriete = stepper->getCurrentPositionInSteps()/STEPPER_MICROSTEPS;
           _volume_ml =  (COEFFS.a * positionApriete * positionApriete) + (COEFFS.b * positionApriete) + COEFFS.c; // volumen ml <-----------------------------------------------------
//           _flow = (_volume_ml-_mylastVolume); // que tiempo????saveVolume();
//           unsigned short mseconds = (unsigned short)(millis() - _lastReadFlow);
//           _flow = _flow / mseconds;
//           _mylastVolume = _volume_ml;
//           _lastReadFlow = millis();
            _flow = -(stepper->getCurrentVelocityInStepsPerSecond()/STEPPER_MICROSTEPS/20);
        }
       
        
#endif

#if ENABLED_SENSOR_VOLUME_byPRESSURE
        float flow;
        flow = mpxVol->MPX_CONVERT_flujo();
        // Record flow samples
        if (flow == _flow && flow != 0)
        {
            _flowRepetitionCounter++;
        } else {
            _flowRepetitionCounter = 0;
        }
        _flow = flow;
        unsigned short mseconds = (unsigned short)(millis() - _lastReadFlow);
        float ml = flow * mseconds / 60; // l/min * ms * 1000 (ml) /60000 (ms)
        _volume_ml += ml;
        _lastReadFlow = millis();
#endif
}


 /**
 * @brief Función que permite resetear los valores de presión de la estructura SensorLastPressure_t
 * @param ninguno
 * @return ninguno
 */


void Sensors::resetPressures(void) {
    lastPres.minPressure = _minPressure;
    lastPres.maxPressure = _maxPressure;
    _minPressure = 0;         //En kPa
    _maxPressure = 100;         //En kPa
    _minLastPressureArray = _minPressureArray[1]; //               <--------------- 9/11/2020 3era revision podria cambiar????
    _minPressureArray[0] = 100;
    _minPressureArray[1] = 100;
    _minPressureArray[2] = 100;
    
}

void Sensors::PEEPRobust(SensorPressureValues_t auxP){
  uint8_t auxPresion = (uint8_t) (auxP.pressure1);
  if(_minPressureArray[2] > auxPresion){
    _minPressureArray[2] = auxPresion;
    if(_minPressureArray[1] > _minPressureArray[2]){
      auxPresion = _minPressureArray[1];
      _minPressureArray[1] = _minPressureArray[2];
      if(_minPressureArray[0] > _minPressureArray[1]){
        _minPressureArray[0] = _minPressureArray[1];
      } 
    }  
  }
}

uint8_t Sensors::getPEEPRobust(void){
  return(_minLastPressureArray);
}


 /**
 * @brief Función que permite obtener el valor de volumen almacenado en la estructura Sensors
 * @param ninguno
 * @return float _volume_ml
 */

float Sensors::getVolumeActual(){
  return (_volume_ml);
}


void Sensors::resetVolumeIntegrator(void) {
    _volume_ml = 0;
    _lastReadFlow = millis();
}
#endif



 /**
 * @brief Función que accede a los datos almacenados en la estructura SensorVolumeValue_t
 * @param ninguno
 * @return _volumeState
 * @return _lastVolume
 */

SensorVolumeValue_t Sensors::getVolume() {
    SensorVolumeValue_t values;
    values.state = _volumeState;
    values.volume = _lastVolume;
    return values;
}

 /**
 * @brief Función para la autocalibración del sensor de presión. Permite calibrar el valor de offset al iniciar el sistema
 * @param   ninguno
 * @return  ninguno
 */
void Sensors::rutinaDeAutoajuste(void){
  const char veces=15;
  float pre_offset=0,aux=0;
  float presion1, presion2;
  
  for (int i=0;i<veces;i++){
    
    pre_offset=mpx->getOffsetValue();
    aux= mpx->autoajuste(pre_offset); 
    mpx->setOffsetValue(aux);  
        
  }
    mpx->getOffsetValue();
//-----------Ejecuto una lectura para verificar que no quede negativo
      presion1 = mpx-> MPX_READ_PRESSURE();          //Volts
      presion2 = mpx->MPX_CONVERT_PRESSURE(presion1);   //kPa
      presion2 = mpx->MPX_CONVERT_cmH20(presion2);   //cmH2O

//si el valor es positivo el offset debe aumentar
//si el valor es negativo el offset debe disminuir 
      if(presion2<1){                //               <--------------- Vuelta atras 9/11/2020 3era revision estaba 1 paso a 0.75
        mpx->setOffsetValue(aux-30);    //               <--------------- Vuelta atras 9/11/2020 3era revision estaba 30 paso a 20
//        Serial.println("");
//        Serial.println("Nos pasamos para abajo, cambiando offset"); 
      }
      EEPROM.put(14,mpx->getOffsetValue());
  }

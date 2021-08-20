/** Estructuras con funciones necesarias para cada sensor
 *
 * @file Sensors.h
 * @details Este archivo contiene las declaraciones de las estructuras con funciones necesarias para cada sensor
 * 
 * \class Sensores
 * \brief Contiene las variables y definiciones de los métodos para utilizar los sensores de presión y volumen
 *
 * \fn Sensors()
 * \brief Constructor del objeto Sensors
 *
 * \fn unsigned int begin(void)
 * \brief Función antigua. No eliminar ni modificar.
 *
 * \fn float readPressure(void)
 * \brief Funcion que lee el sensor de presión MPX 50XX 
 * \param ninguno
 * \return valor de presión leido
 *
 * \fn void Sensors::rutinaDeAutoajuste(void)
 * \brief Función para la autocalibración del sensor de presión. Permite calibrar el valor de offset al iniciar el sistema
 
 *
 * \fn SensorLastPressure_t getLastPressure(void)
 * \brief  Función que permite obtener la última presión leida.
 * \param  ninguno
 * \return Último valor de presión registrado
 
 *
 * \fn SensorPressureValues_t getAbsolutePressureInPascals(void)
 * \brief  Función que permite obtener el valor absoluto de presión en Pascales.
 * \param  ninguno
 * \return Valor de presión obtenido convertido a Pascales
 
 *
 * \fn SensorPressureValues_t getAbsolutePressureInCmH2O(void)
 * \brief  Función que permite obtener el valor absoluto de presión en CmH20.
 * \param  ninguno
 * \return Valor de presión obtenido convertido a CmH20
 
 *
 * \fn SensorPressureValues_t getRelativePressureInCmH2O(void)
 * \brief  Función que permite obtener el valor relativo de presión en CmH20.
 * \param  ninguno
 * \return Valor de presión obtenido convertido a CmH20
 
 *
 * \fn void resetPressures(void)
 * \brief Función que permite resetear los valores de presión de la estructura SensorLastPressure_t
 *
 *
 * \fn void rebootVolumeSensor(void)
 * \brief   Funcion que reestablece el valor del integrador para el sensor de volumen 
 *
 * \fn void readVolume()
 * \brief Función que lee volumen y lo guarda en la estructura correspondiente.
 *
 *
 * \fn void resetVolumeIntegrator(void)
 * \brief Función que resetea el integrador utilizado para calcular el volumen
 *
 * \fn float getFlow(void)
 * \brief  Función que permite obtener una medida de flujo.
 * \param  ninguno
 * \return valor de flujo obtenido
 *
 * \fn float getVolumeActual(void)
 * \brief Función que permite obtener el valor de volumen almacenado en la estructura Sensors
 * \param ninguno
 * \return volumen leido.
 *
 * \fn SensorVolumeValue_t getVolume(void)
 * \brief Función que accede a los datos almacenados en la estructura SensorVolumeValue_t
 * \param ninguno
 * \return _volumeState: Estado del sensor de volumen
 * \return _lastVolume:  Última lectura almacenada.
 *
 * \fn void saveVolume(void)
 * \brief Función que guarda el volumen leído como última medida tomada en la estructura correspondiente.
 * 
 *
 * \fn bool stalledVolumeSensor()
 * \brief   Función obsoleta. No usar, no borrar.
 *
 *
 * \fn void _init(void)
 * \brief  Funcion que inicializa la clase sensores 
 *
 * variables privadas
 *
 * \var uint8_t _minPressure
 * \brief Presión mínima
 *
 * \var uint8_t _maxPressure
 * \brief Presión máxima
 *
 * \var float _pressure1
 * \brief Presión leída 1
 *
 * \var float _pressure2
 * \brief Presión leída 2
 *
 * \var float _pressureSensorsOffset
 * \brief Desviación del sensor de presión
 *
 * \var SensorState _state
 * \brief Estado del sensor
 *
 * \var SensorState _volumeState
 * \brief Estado del sensor de volumen
 *
 * \var byte _errorCounter
 * \brief Contador de errores
 *
 * \var volatile uint8_t _lastMinPressure
 * \brief Última presión mínima leída
 *
 * \var volatile uint8_t _lastMaxPressure
 * \brief Última presión máxima leída
 *
 * \var float _volume_ml
 * \brief Volumen leído en mililitros
 *
 * \var float _mylastVolume
 * \brief Último volumen leído
 *
 * \var float _flow
 * \brief Flujo leido
 *
 * \var volatile float _lastVolume
 * \brief último volumen leída
 *
 * \var unsigned long _lastReadFlow
 * \brief último flujo leído
 *
 * \var unsigned int _flowRepetitionCounter
 * \brief contador de mediciones de flujo
 *
 *
 *
 */



#ifndef _SENSORS_H_
#define _SENSORS_H_

#include <stdint.h>
#include "defaults.h"
#ifdef ENABLED_SENSOR_VOLUME_SFM3300
#include "src/SFM3200/sfm3000wedo.h"
#endif
#if ENABLED_SENSOR_VOLUME_STEPPER
#include "src/FlexyStepper/FlexyStepper.h"
#endif

//#include "src/Adafruit_BME280/Adafruit_BME280.h"
//#include "src/Honeywell_ABP/Honeywell_ABP.h"

#define SENSORS_MAX_ERRORS 5

#if ENABLED_SENSOR_VOLUME_SFM3300
#define SFM3300_OFFSET 32768
#define SFM3300_SCALE   120
#endif



#if ENABLED_SENSOR_VOLUME_STEPPER
#define    SENSOR_VOLUME_STEPPER_KVolumen 1.111F  //depende de la calibraci
#endif


/**
 * \enum SensorState
 * \brief   Estructura de los posibles estados de un sensor
 * \var SensorStateOK
 * \var SensorStateFailed 
 *
 */

enum SensorState {
    SensorStateOK = 0,
    SensorStateFailed = 1
};


/**
 * \struct SensorLastPressure_t
 * \brief   Estructura que almacena las últimas lecturas del sensor de presión (mínima y máxima)
 * 
 * \var minPressure
 * \brief mínima presión registrada 
 * 
 * \var maxPressure
 * \brief máxima presión registrada
 *
 */

typedef struct {
    uint8_t minPressure;
    uint8_t maxPressure;
} SensorLastPressure_t;



 /**
 * \struct SensorLastPressure_t
 * \brief   Estructura que contiene información sobre el sensor de de presión.
 * 
 * \var state
 * \brief Estado del sensor 
 * 
 * \var pressure1
 * \brief Presión 1 
 * 
 * \var pressure2
 * \brief Presión 2
 *
 */
 
typedef struct {
    SensorState state;
    float pressure1;
    float pressure2;
} SensorPressureValues_t;



/**
 * \struct SensorVolumeValue_t
 * \brief   Estructura que contiene información sobre el sensor de volumen.
 * \var state
 * \brief estado del sensor.
 * \var volume
 * \brief volumen almacenado.
 */
typedef struct {
    SensorState state;
    short volume; // ml
} SensorVolumeValue_t;


/**
 * @brief   Clase "Sensores". Contiene las variables y definiciones de los métodos para utilizar los sensores de presión y volumen
 *
 *
 *
 *
 * @brief   Función Dummy
 *
 * @param   ninguno
 * @return  0
 */
 
 
class Sensors
{
    public:
    Sensors();
    unsigned int begin(void);
    float readPressure(void);
    void Sensors::rutinaDeAutoajuste(void);//<<<<<----------------------------------
    SensorLastPressure_t getLastPressure(void);
    SensorPressureValues_t getAbsolutePressureInPascals(void);
    SensorPressureValues_t getAbsolutePressureInCmH2O(void);
    SensorPressureValues_t getRelativePressureInCmH2O(void);
    void resetPressures(void);
    void PEEPRobust(SensorPressureValues_t);
    uint8_t getPEEPRobust(void);
#if ENABLED_SENSOR_VOLUME
    void rebootVolumeSensor(void);
    void readVolume(
#if ENABLED_SENSOR_VOLUME_STEPPER
    FlexyStepper *stepper
#else
    void
#endif 
    );

    void resetVolumeIntegrator(void);
    float getFlow(void);
    float getVolumeActual(void);
    SensorVolumeValue_t getVolume(void);
    void saveVolume(void);
    bool stalledVolumeSensor();
#endif


    private:
    void _init(void);
 //   Adafruit_BME280 _pres1Sensor;
 //   Adafruit_BME280 _pres2Sensor;
#if ENABLED_SENSOR_VOLUME_SFM3300
    SFM3000wedo* _sfm3000;
#endif
    uint8_t _minPressure;
    uint8_t _minPressureArray[3];
    uint8_t _minLastPressureArray = 0;
    uint8_t _maxPressure;
    float _pressure1;
    float _pressure2;
    float _pressureSensorsOffset = 0.0;
    SensorState _state;
    SensorState _volumeState;
    byte _errorCounter;
    volatile uint8_t _lastMinPressure;
    volatile uint8_t _lastMaxPressure;
#if ENABLED_SENSOR_VOLUME
    float _volume_ml;
    float _mylastVolume = 0;
    float _flow;
    volatile float _lastVolume;
    unsigned long _lastReadFlow;
    unsigned int _flowRepetitionCounter = 0;
#endif

};

#endif

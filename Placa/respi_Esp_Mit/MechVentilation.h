/** Ventilación mecánica
*
* @file    MechVentilation.h
* @details Este archivo contiene las declaraciones de los métodos para controlar la ventilación mecánica a través de todos sus componentes.
* 

* \enum State
* \brief   Enumneración de los estados de la máquina de estados utilizada para el control de la ventilación mecánica.
*
*  Init_Insufflation  Comienza la insuflación
* State_Insufflation  Se insufla durante todo el tiempo de inspiración
* Init_Exsufflation   Comienza la exuflación  
* State_Exsufflation  Comienza la exsuflación 
* State_Hold_In       Tiempo de esperaantes de la exsuflación  
* State_Error_Pas     Estado de error  
* State_StandBy       Estado de espera. Respirador apagado
* State_Homing        Estado donde se realiza lacalibración del cero del motor paso a paso 
* State_Error         Estado de error  
*
***Seteos para la temporización
*
* \var HOLD_IN_DURATION
* \brief Duracionde la pausa en ms luego de la insuflación
*
* \var MIN_PEEP_PAUSE
* \brief Duracion en ms de la pausa luego de la exsuflación. Durante este tiempo se espera por una posible reacción del paciente.
*
* \var MAX_EX_DURATION
* \brief duración máxima de exalación en ms
*
* \var VoluTidal
* \brief Arreglo donde se guarda el volumen tidal
*
* \struct Configuration_t
* \brief   Estructura de configuración
*
* \param pip        
* \brief Peak Inspiratory Pressure
*
* \param timeoutIns
* \brief Tiempo total de inspiración
*
* \class Mech Ventilation
* \brief Contiene las variables y definiciones de los métodos para controlar la ventilación mecánica.
*
* \fn  MechVentilation(FlexyStepper *stepper,Sensors *sensors,AutoPID *pid,VentilationOptions_t options)
* \brief Construct a new Mech Ventilation object
*
* \param stepper
* \brief objeto stepper
* \param sensors
* \brief objeto sensors
* \param pid
* \brief objeto pid
* \param options
* \brief configuraciones de altura,sexo,edad etc.
*
* \fn boolean getStartWasTriggeredByPatient();
* \brief Esta función permite leer la variable que indica si el paciente tuvo intención de respirar
* \param ninguno
* \return  Bool
*
*\fn void setVentilationCyle_WaitTime(float speedExsufflation)
*\brief función de versión antigua. No borrar ni modificar
*
*
* \fn void start(void)
* \brief inicia la máquina de estados. Comienza a funcionar el respirador.
*
* \fn void stopMech(void)
* \brief Para la máquina de estados
*
* \fn uint8_t getRunning(void)
* \brief Función que permite leer la variable _running 
*
* \fn void evaluatePressure(void)
* \brief Evalúa los valores de presión a fin de activar las alarmas.
*
** \fn void evaluate220V(void)
* \brief Evalúa los valores de presión a fin de activar las alarmas.
*
* \fn void update(void)
* \brief Actualiza la máquina de estados de ventilación.
* Si alguna variable cambiara, el nuevo valor se actualiza en el siguiente ciclo de la máquina de estados.   
*
* \fn void activateRecruitment(void)
* \brief Activa el modo reclutamiento
* \fn void deactivateRecruitment(void)
* \brief Desactiva el modo reclutamiento
*
* \fn int getAlarms(void)
* \brief permite leer el estado de las alarmas
* \param ninguno
* \return Valor de las alarmas activas.
*
* \fn bool getSensorErrorDetected()
* \brief permite leer el estado del sensor de presión
* \param ninguno
* \return Estado del sensor. True si hubo error
*
*
* \fn uint8_t getRPM(void)
* \param ninguno
* \return Rpm actuales del motor
*
* \fn short getExsuflationTime(void)
* \param ninguno
* \return Tiempo de exsuflación seteado
*
* \fn short getInsuflationTime(void)
* \param ninguno
* \return Tiempo de insuflación seteado
*
* \fn short getPeakInspiratoryPressure(void)
* \param ninguno
* \return Presión inspiratoria pico seteada
*
* \fn short getPeakEspiratoryPressure(void)
* \param ninguno
* \return Presión espiratoria pico seteada
*
* \fn short getVolTidalPorcen(void)
* \param ninguno
* \return Porcentaje de volumen tidal que se está insuflando.
*
* \fn short getVolTidalByArray(void)
* \param ninguno
* \returnarreglo de volumen tidal.
*
* \fn short getIer(void)
* \param ninguno
* \return I/E ratio seteado.
*
* \fn short getTh(void)
* \param ninguno
* \return el valor del umbral que detecta la intención de respiración.
*
* \fn uint8_t getPause(void)
* \param ninguno
* \return tiempo de pausa seteado.
*
* \fn uint8_t getFrReal(void)
* \param ninguno
* \return frecuencia de ventilación seteada.
*
* \fn State getState(void)
* \param ninguno
* \return Estado de la máquina de estados
*
*
*
* \fn void  setAlarms(int alarm_value)
* \brief  Setea las alarmas en un valor determinado.
* \param alarm_value
* \brief valor de alarma deseado.
* \return ninguno
*
* \fn void  resetAlarms(int alarm_value)
* \brief  Resetea las alarmas.
* \param alarm_value
* \brief valor de alarma deseado.
* \return ninguno
*
* \fn void  setInsuflationTime(uint8_t value)
* \brief  Setea el valor del tiempo de insuflación.
* \param value
* \brief valor de tiempo deseado
* \return ninguno
*
* \fn float getInsuflationTimeApp(void)
* \brief  Lee el valor de la aplicación para el tiempo de insuflación.
* \param  ninguno
* \return Tiempo de insuflación
*
* \fn void  setRPM(uint8_t rpm)
* \brief  Setea el valor de las rpm del motor paso a paso.
* \param rpm
* \return ninguno
*
* \fn void  setPeakInspiratoryPressure(uint8_t pip)
* \brief  Setea el valor de la presión inspiratoria pico.
* \param pip
* \return ninguno
*
* \fn void  setPeakEspiratoryPressure(uint8_t peep)
* \brief  Setea el valor de vla presión espiratoria de pico.
* \param peep
* \return ninguno
*
* \fn void  setPause(uint8_t pause)
* \brief  Setea el valor de la pausa de inspiración.
* \param pause
* \return ninguno
*
* \fn void  setVolTidalInStep(uint8_t vt)
* \brief  Setea el valor del Volumen tidal en pasos.
* \param vt
* \return ninguno
*
* \fn void  setVolTidalByArray(uint8_t vt)
* \brief  Setea el valor de volumen tidal en el arreglo.
* \param vt
* \return ninguno
*
* \fn void  setIer(uint8_t ier)
* \brief  Setea el valor del I/E ratio.
* \param ier
* \return ninguno
*
* \fn void  setTh(uint8_t th)
* \brief  Setea el valor del umbral de intención de respiración.
* \param th 
* \brief valor de umbral deseado
* \return ninguno
*
* \fn void  goToPositionByDur(FlexyStepper *stepper, int goal_pos, int cur_pos, int dur)
* \brief  Esta función realiza el movimiento del motor hacia una posición en un tiempo determinado.
* \param  *stepper 
* \brief  Objeto stepper
* \param  goal_pos
* \brief  Posición final deseada
* \param  cur_pos, 
* \brief  Posición actual
* \param  dur
* \brief  duración del movimiento
* \return ninguno
*
* \fn void  setState(State state)
* \brief  Cambia el estado de la máquina de estados
* \param  state
* \brief estado al que se quiere cambiar.
* \return ninguno
*
* \fn uint8_t GetPlateau(void)
* \brief  Permite leer la presión de plateau
* \param ninguno
* \return Presión de plateau
*
* \fn unsigned long GetCiclos(void)
* \brief  Permite leer la cantidad de ciclos que lleva funcionando el respirador
* \param ninguno
* \return Cantidad de ciclos
*
* \fn void ResetCiclos(void)
* \brief  Resetea el contador de ciclos
* \param  ninguno
* \return ninguno
*
*
*
* \fn void _init(FlexyStepper *stepper,Sensors *sensors,AutoPID *pid,VentilationOptions_t options)
* \brief  Inicializa el objeto Mech Ventilation
* \param  stepper
* \brief  Objeto stepper
* \param  sensors
* \brief  Objeto sensors
* \param  pid
* \brief  Objeto pid
* \param  options
* \brief  estructura de opciones de peso, sexo, etc
* \return ninguno
*
* \fn void _setInspiratoryCycle(void)
* \brief  Setea el valor del ciclo de inspiración con valores por defecto.
* \param  ninguno
* \return ninguno
*
*
*
* \var FlexyStepper *_stepper
* \brief Objeto motor paso a paso
*
* \var Sensors *_sensors
* \brief Objeto sensor
*
* \var AutoPID *_pid
* \brief Objeto PID
*
* \var uint8_t _pause 
* \brief Valor de la pausa de inspiración
*
* \var bool enteringState
* \brief Bandera que indica si se está entrando a un estado de la maquina de estados.
*
* \var bool _hasTrigger
* \brief Bandera que indica la intención de respiración por parte del paciente.
*
* \var volatile float _triggerThreshold
* \brief Umbral de intención de respiración por parte del paciente.
*
* \var volatile float tCycleTimer
* \brief Timer de tiempo de ciclo
*
* \var float volatile _timeoutIns
* \brief Tiempo de insuflación
*
* \var float volatile _timeoutInsCopy
* \brief Copia del tiempo de insuflación
*
* \var float volatile _tHoldIn
* \brief Tiempo total de insuflación
*
* \var float volatile _tHoldInCopy
* \brief Copia del Tiempo total de insuflación
*
* \var volatile float _timeoutEsp
* \brief Tiempo total de espiración
*
* \var volatile float _timeoutEspCopy
* \brief Copia del tiempo total de espiración.
*
    
* \var volatile float _durac_exu
* \brief Tiempo de exsuflación.
*
* \var volatile float _durac_exuCopy
* \brief Copia del tiempo de exsuflación
*
     
* \var volatile uint8_t _difTime
* \brief Diferencia de tiempos
*
* \var volatile uint8_t _rpm
* \brief Valor de las rpm del motor paso a paso
*
* \var short volatile _pip
* \brief Valor de la Presión inspiratoria pico
*
* \var short _peep
* \brief Valor de la Presión espiratoria de pico
*
* \var uint8_t _volTidalSteps
* \brief Volumen tidal en pasos del motor paso a paso
*
* \var uint8_t _volTidalStepsCopy
* \brief Copia del volumen tidal en pasos del motor paso a paso
*
* \var float _ier
* \brief I/E Ratio
*
* \var uint8_t _thr
* \brief Valor del umbral de disparo por intención de respiración.
*
* \var uint8_t _vt
* \brief Valor del volumen tidal
*
* \var bool volatile _recruitmentMode
* \brief Bandera de activación/desactivación del modo reclutamiento
*
* \var Configuration_t _nominalConfiguration
* \brief Valores de configuración nominal
*
* \var State _currentState
* \brief Estado actual de la máquina de estados
*
* \var int _currentAlarm
* \brief Valor actual de las alarmas
*
* \var float _stepperSpeed
* \brief Velocidad actual del motor paso a paso
*
* \var float _stepperPos
* \brief Posición actual del motor paso a paso
*
* \var bool _running
* \brief Bandera que indica si la máquina de estado está corriendo o no
*
* \var bool _sensor_error_detected
* \brief Bandera que indica si hubo error en el sensor
*
* \var bool _startWasTriggeredByPatient
* \brief Bandera que indica si el paciente tuvo intención de respirar
*
* \var float _currentPressure
* \brief Valor actual de presión leida.
*
* \var unsigned long _ciclos
* \brief Cantidad de ciclos realizados por el respirador
*
* \fn void SumPlateau(uint8_t);
* \brief  Realiza la sumatoria de las presiones plaateau
* \param  ninguno
* \return ninguno
*
* \fn void PromPlateau(void)
* \brief  Realiza un primedio de las presiones de plateau.
* \param  ninguno
* \return ninguno
*
* \var long PressurePlateauTotal
* \brief Valor total de presión plateau.
*
* \var long PressurePlateauCont
* \brief Contador de presión plateau.
*
* \var uint8_t PressurePlateauProm;
* \brief Valor promedio de presión plateau.
*
* \var int Alarm_value
* \brief Valor de alarma actual
*
*
* 
* 
* 
* 
* 
* 
* 
* 
*/



#ifndef INC_MECHANICAL_VENTILATION_H
#define INC_MECHANICAL_VENTILATION_H
#include "mpx.h"
#include <float.h>
#include <inttypes.h>
#include "defaults.h"
#include "calc.h"
#include "Sensors.h"
#include "src/AutoPID/AutoPID.h"
#include "src/FlexyStepper/FlexyStepper.h"

/**
 * @brief   Enumneración de los estados de la máquina de estados utilizada para el control de la ventilación mecánica.
 *
 */

enum State
{
    Init_Insufflation = 1,
    State_Insufflation = 2, /**< Insufflating (PID control). */
    Init_Exsufflation = 3,
    State_Exsufflation = 4, /**< Return to position 0 and wait for the patient to exsufflate. */
    State_Hold_In = 5,
    State_Error_Pas = 6,
    State_StandBy = 7,
    State_Homing = 0,
    State_Error = -1
};

/**
 * @brief   Seteos para la temporización
 */
// Timing Settings
const float HOLD_IN_DURATION = 30.0;  // Duration (ms) to pause after inhalation
const float MIN_PEEP_PAUSE =50.0;    // Time (ms) to pause after exhalation / before watching for an assisted inhalation
const float MAX_EX_DURATION = 1000.0;   // Maximum exhale duration (ms)
const uint8_t VoluTidal[20] = {91, 96, 101, 105,  //0-18
                               110, 115, 119, 124,
                               129, 133, 138, 143,
                               148, 153, 158, 163,
                               168, 173, 178, 186};
                           

/**
 * @brief   Estructura de configuración
 *
 * @param   pip:        Peak Inspiratory Pressure
 *          timeoutIns: Tiempo total de inspiración
 * 
 */
typedef struct {
    float pip;
    unsigned short timeoutIns;
} Configuration_t;



/**
 * @brief   Clase "mech Ventilation". Contiene las variables y definiciones de los métodos para controlar la ventilación mecánica.
 */
class MechVentilation
{
public:
    /**
	 * @brief Construct a new Mech Ventilation object
	 *
	 * @param stepper
	 * @param sensors
	 * @param pid
	 * @param options
	 */
    MechVentilation(
        FlexyStepper *stepper,
        Sensors *sensors,
        AutoPID *pid,
        VentilationOptions_t options);

    boolean getStartWasTriggeredByPatient();
    void setVentilationCyle_WaitTime(float speedExsufflation);
    /** Start mechanical ventilation. */
    void start(void);
    /** Stop mechanical ventilation. */
    void stopMech(void);
    uint8_t getRunning(void);
    /** Alarms */
    void evaluatePressure(void);
    void evaluate220V(void);
    /** Update mechanical ventilation.
     *
     * If any control variable were to change, new value
     * would be applied at the beginning of the next ventilation
     * cycle.
     *
     * @note This method must be called on a timer loop.
     */
    void update(void);

    /** Recruitment */
    void activateRecruitment(void);
    void deactivateRecruitment(void);

    /**
     * getters
     */
    int getAlarms(void);//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    bool getSensorErrorDetected();
    uint8_t getRPM(void);
    short getExsuflationTime(void);
    short getInsuflationTime(void);
    short getPeakInspiratoryPressure(void);
    short getPeakEspiratoryPressure(void);
    short getVolTidalPorcen(void);
    short getVolTidalByArray(void);
    short getIer(void);
    short getTh(void);
    uint8_t getPause(void);
    uint8_t getFrReal(void);
    State getState(void);
    
    /**
     * setters
     */
    void  setAlarms(int alarm_value);//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    void  resetAlarms(int alarm_value);//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    void  setInsuflationTime(uint8_t value);//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    float getInsuflationTimeApp(void);//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    void  setRPM(uint8_t rpm);
    void  setPeakInspiratoryPressure(uint8_t pip);
    void  setPeakEspiratoryPressure(uint8_t peep);
    void  setPause(uint8_t pause);
    void  setVolTidalInStep(uint8_t vt);
    void  setVolTidalByArray(uint8_t vt);
    void  setIer(uint8_t ier);
    void  setTh(uint8_t th);
    void  goToPositionByDur(FlexyStepper *stepper, int goal_pos, int cur_pos, int dur); 
    void  setState(State state);
    uint8_t GetPlateau(void);
    unsigned long GetCiclos(void);
    void ResetCiclos(void);
    
private:
    /** Initialization. */
    void _init(
        FlexyStepper *stepper,
        Sensors *sensors,
        AutoPID *pid,
        VentilationOptions_t options);
#if 0
    int _calculateInsuflationPosition (void);
#endif
  // Returns the current time in seconds
    inline float now() { return millis()*1e-3; }
    /** Set state. */
    //void _setAlarm(Alarm alarm);
#if 0
    void _increaseInsuflationSpeed (byte factor);
    void _decreaseInsuflationSpeed (byte factor);
    void _increaseInsuflation (byte factor);
    void _decreaseInsuflation (byte factor);
#endif
    void _setInspiratoryCycle(void);

    /* Configuration parameters */
    FlexyStepper *_stepper;
    Sensors *_sensors;
    AutoPID *_pid;
    uint8_t _pause ;
    bool enteringState;
    /** Flow trigger activation. */
    bool _hasTrigger;
    /** Flow trigger value in litres per minute. */
    volatile float _triggerThreshold;
    volatile float tCycleTimer;
    /**  Insufflation timeout in seconds. */
    float volatile _timeoutIns;
    float volatile _timeoutInsCopy;
    float volatile _tHoldIn;
    float volatile _tHoldInCopy;
    /** Exsufflation timeout in seconds. */
    volatile float _timeoutEsp;
    volatile float _timeoutEspCopy;
    
    volatile float _durac_exu;
     volatile float _durac_exuCopy;
     
    volatile uint8_t _difTime; 
    /** Breaths per minute */
    volatile uint8_t _rpm;
    /** Peak inspiratory pressure */
    short volatile _pip;
    /** Peak espiratory pressure */
    short _peep;
    /** Volumen Tidal */
    uint8_t _volTidalSteps;
    uint8_t _volTidalStepsCopy;
    /** InsExpRatio */
    float _ier;
    /** Thr */
    uint8_t _thr;

    uint8_t _vt;
    
    /** Recruitment */
    bool volatile _recruitmentMode = false;

    /* Configuration */
    Configuration_t _nominalConfiguration;

    /* Internal state */
    /** Current state. */
    State _currentState = State_Homing;
    int _currentAlarm = No_Alarm;

    /** Stepper speed. Steps per seconds. */
    float _stepperSpeed;
    float _stepperPos;
    bool _running = false;
    bool _sensor_error_detected;
    bool _startWasTriggeredByPatient = false;
    float _currentPressure = 0.0;
    unsigned long _ciclos = 0;
    //float _currentFlow = 0.0;
    //float _currentVolume = 0.0;
    /// Plateau
    void SumPlateau(uint8_t);
    void PromPlateau(void);
    long PressurePlateauTotal;
    long PressurePlateauCont;
    uint8_t PressurePlateauProm;

    /* Alarmas */
    int Alarm_value;
};

#endif /* INC_MECHANICAL_VENTILATION_H */

/** Valores por defecto del sistema.
 *
 * @file defaults.h
 * @details Este archivo contiene todos los valores por defecto que emplea el sistema
 *
 *
 */
#ifndef _DEFAULTS_H
#define _DEFAULTS_H

/*
 * Constants
 * \def DEBUG_UPDATE  
 * \brief variable utilizada para mostrar mensajes importantes en debugging
 * 
 * \def DEBUG_PLOT 
 * \brief variable utilizada para mostrar gráficos importantes en debugging
 * 
 * \def TIME_BASE
 * \brief Base de tiempo en ms para la máquina de estados
 * 
 * \def TIME_SENSOR
 * \brief Base de tiempo en ms para la lectura de sensores
 * 
 * \def TIME_EPROM             6000   
 * \brief Base de tiempo para la actualización de la memoria EPROM.  
 *  TIME_BASE*TIME_EPROM=100ms x 6000 = 600000ms = 600s= 10m 
 * 
 * \def TIME_SEND_CONFIGURATION 2000 // msec
 * \brief Base de tiempo en ms para la comunicación con Tablet
 * 
 * 
 * \def ENABLED_SENSOR_VOLUME 
 * \brief Variable que indica la presencia o no del sensor de volumen en el sistema.
 * 
 * \def ENABLED_SENSOR_VOLUME_SFM3300
 * \brief Variable que indica la presencia o no del sensor de volumen SFM3300 en el sistema.
 * 
 * \def ENABLED_SENSOR_VOLUME_STEPPER 
 * \brief Variable que indica si el volumen es estimado por los pasos del motor o no.
 * 
 * \def ENABLED_SENSOR_VOLUME_byPRESSURE 
 * \brief Variable que indica si la presión se setima en base a mediciones de presión o no.
 * 
 * 
 *\struct COEFFS
 * \brief Estructura con coeficientes para la calibración de la bolsa ambu de adulto.
 * 
 * \def AMBU_MAX_ML
 * \brief Volumen máximo en ml de la bolsa ambu para dultos.
 * 
 * 
 **** Stepper defines
 * 
 * \def STEPPER_MICROSTEPS
 * \brief Micropasos del motor paso a paso
 * 
 * \def STEPPER_STEPS_PER_REVOLUTION
 * \brief Cantidad de pasos por revolución del motor paso a paso
 * 
 * \def STEPPER_MICROSTEPS_PER_REVOLUTION 
 * \brief Cantidad de micropasos por revolución del motor. Se calcula como STEPPER_MICROSTEPS_PER_REVOLUTION=STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS
 * 
 * \def STEPPER_DIR 
 * \brief dirección de movimiento del motor paso a paso.
 * 
 * \def STEPPER_HOMING_DIRECTION
 * \brief dirección de movimiento del motor paso a paso durante la rutina de posicionamiento inicial.
 * 
 * \def STEPPER_HOMING_SPEED
 * \brief velocidad de movimiento del motor paso a paso durante la rutina de posicionamiento inicial.
 * 
 * \def STEPPER_LOWEST_POSITION     
 * \brief mínima posición en pasos que puede alcanzar el eje motor. Físicamente se traduce en la bolsa ambu sin presionar
 * 
 * \def STEPPER_HIGHEST_POSITION   
 * \brief máxima posición en pasos que puede alcanzar el eje motor. Físicamente se traduce en la bolsa ambu totalmente presionada.
 * 
 * \def STEPPER_SPEED_DEFAULT
 * \brief velocidad por defecto del movimiento del motor paso a paso.
 * 
 * \def STEPPER_SPEED_MAX
 * \brief Máxima velocidad del movimiento del motor paso a paso. Limite superior
 * 
 * \def STEPPER_ACC_MAX
 * \brief Máxima aceleración del movimiento del motor paso a paso. Limite superior
 * 
 * \def STEPPER_SPEED_MAX_STOP
 * \brief Máxima velocidad del movimiento del motor paso a paso cuando se lo quiere parar.
 * 
 * \def STEPPER_ACC_MAX_STOP
 * \brief Máxima aceleración del movimiento del motor paso a paso cuando se lo quiere parar.
 *
 * \def STEPPER_ACC_EXSUFFLATION
 * \brief Aceleración del movimiento del motor paso a paso durante la exuflación.
 * 
 * \def STEPPER_ACC_INSUFFLATION
 * \brief Aceleración del movimiento del motor paso a paso durante la insuflación.
 *
 * \def DEFAULT_HEIGHT 
 * \brief Altura del paciente por defecto.
 * 
 * \def DEFAULT_SEX
 * \brief Sexo del paciente por defecto, siendo 0=varón, 1=mujer.
 * 
 * \def DEFAULT_ML_PER_KG_IDEAL_WEIGHT
 * \brief Cantidad de mililitros de aire por Kg, se considera el peso ideal.
 * 
 * \def DEFAULT_MAX_TIDAL_VOLUME
 * \brief Volumen tidal máximo por defecto.
 * 
 * \def DEFAULT_MIN_TIDAL_VOLUME 
 * \brief Volumen tidal mínimo por defecto.
 * 
 * \def DEFAULT_DIFF_TIDAL_VOLUME
 * \brief Variable que define la excursión máxima que puede tener el motor entre el máximo y el mínimo volumen tidal insuflado.
 * 
 * \def DEFAULT_TRIGGER_THRESHOLD
 * \brief Umbral de disparo por defecto. Se utiliza para detectar la intención de respiración por parte del paciente.
 * 
 * \def DEFAULT_RPM
 * \brief Revoluciones por minuto por defecto del motor.
 * 
 * \def DEFAULT_MAX_RPM
 * \brief Valor máximo de RPM del motor.
 * 
 * \def DEFAULT_MIN_RPM
 * \brief Valor mínimo de RPM del motor.
 * 
 * \def DEFAULT_INSPIRATORY_FRACTION
 * \brief Valor por defecto del I/E ratio.
 * 
 * \def DEFAULT_PEAK_INSPIRATORY_PRESSURE.
 * \brief Valor deseado de presión inspiratoria por defecto.
 * 
 * \def DEFAULT_PEAK_INSPIRATORY_PRESSURE_MIN
 * \brief Valor mínimo de presión inspiratoria por defecto.
 * 
 * \def DEFAULT_PEAK_INSPIRATORY_PRESSURE_MAX
 * \brief Valor máximo de presión inspiratoria por defecto.
 * 
 * \def DEFAULT_PEAK_ESPIRATORY_PRESSURE
 * \brief Valor deseado de presión espiratoria por defecto.
 * 
 * \def DEFAULT_PEAK_ESPIRATORY_PRESSURE_MIN
 * \brief Valor mínimo de presión espiratoria por defecto.
 * 
 * \def DEFAULT_PEAK_ESPIRATORY_PRESSURE_MAX
 * \brief Valor máximo de presión espiratoria por defecto.
 * 
 * \def STEPPER_PEEP_SOLENOID_HYSTERESIS  
 * \brief Valor de presión espiratoria por defecto en cmH2O para implementar un ciclo de histéresis con los solenoides.
 * 
 * \def DEFAULT_PORC_VOLTILDAL
 * \brief Porcentaje de volumen tidal insuflado por defecto.
 * 
 * \def DEFAULT_BYARRAY_VOLTILDAL         
 * \brief indice del arreglo volumen tidal
 * 
 * \def DEFAULT_PAUSE                     1
 * \brief Pausa por defecto. Normalmente este valor es referido como HOLD_IN
 * 
 * \def DEFAULT_TINS                     
 * \brief Tiempo de insuflación por defecto. 
 * 
 * \def DEFAULT_PA_TO_CM_H2O                     
 * \brief Constante de conversión de pascales a cmH2O.
 * 
 * \def DEFAULT_RECRUITMENT_TIMEOUT
 * \brief Tiempo de reclutamiento en ms por defecto. 
 * 
 * \def DEFAULT_RECRUITMENT_PIP    
 * \brief Peak Inspiratory Pressure de reclutamiento por defecto. 
 * 
 * 
 * Variables de configuración PID. Versión antigua. No modificar ni borrar.
 * \def PID_MIN
 * \def PID_MAX
 * \def PID_KP
 * \def PID_KI
 * \def PID_KD
 * \def PID_TS
 * \def PID_BANGBANG
 * 
 ******** variables de control de los solenoides
 * \def SOLENOID_CLOSED
 * \def SOLENOID_OPEN 
 * 
 ******** Variables de Pinout
 * 
 * \def PIN_STEPPER_STEP
 * \brief Pin STEP del controlador para motor paso a paso.
 * 
 * \def PIN_STEPPER_DIRECTION
 * \brief Pin DIR del controlador para motor paso a paso.
 * 
 * \def PIN_STEPPER_EN
 * \brief Pin EN del controlador para motor paso a paso.
 * 
 * \def PIN_BUZZ
 * \brief Pin digital para activar el buzzer.
 * 
 * \def PIN_STEPPER_ENDSTOP
 * \brief Pin de entrada para el sensor fin de carrera.
 * 
 * \def PIN_SOLENOID1
 * \brief Pin de activación del solenoide 1.
 * 
 * \def PIN_SOLENOID2
 * \brief Pin de activación del solenoide 2.
 * 
 * 
 * * \def ALARM_MAX_PRESSURE 
 * \brief  Máxima presión en cmH2O permitida.
 * 
 * \def ALARM_MIN_PRESSURE 3  // cmH2O ESTA ALARMA SE INTERPRETA COMO SENSOR DESCONECTADO
 * \brief  Mínima presión en cmH2O permitida.
 * 
 * Codificación de las alarmas. Estos códigos se envían a la Tablet.
 * Cada código tiene su versión de activación y desactivación.
 * 
 * \def No_Alarm                
 *
 * Alarma de sensor desconectado 
 * \def Alarm_desconected_EN 
 * \def Alarm_desconected_DIS 
 * 
 * Alarma de sobrepresión
 * \def Alarm_Overpressure_EN
 * \def Alarm_Overpressure_DIS
 * 
 * Alarma de intención de respiración
 * \def Alarm_breathe_EN
 * \def Alarm_breathe_DIS
 * 
 * Alarma de falta de flujo
 * \def Alarm_No_Flux_EN       
 * \def Alarm_No_Flux_DIS      
 * 
 * Códigos libres para futuras implementaciones de alarmas
 * \def Alarm_Libre_1_EN
 * \def Alarm_Libre_1_DIS
 *
 *\def Alarm_Libre_2_EN        0x2020
 *\def Alarm_Libre_2_DIS       0xDFFF*
 * 
 *\def Alarm_Libre_3_EN        0x4040
 *\def Alarm_Libre_3_DIS       0xBFFF
 *
 *\def Alarm_Libre_4_EN        0x8080
 *\def Alarm_Libre_4_DIS       0X7FFF
 * 
 ****Pinout para sensor MPX 5050/5010
 * 
 * \def PIN_MPX_DATA
 * \brief Pin de datos del sensor de presión MPX
 * 
 * \def PIN_MPX_FLOW    
 * \brief Pin de datos del sensor de flujo MPX
 * 
 * \def DEFAULT_OFFSET_MPX
 * \brief valor de desviación por defecto. Utilizada para corregir la desviación inicial del sensor.
 * 
 * \def DEFAULT_OFFSET_MPX2 210.0F
 * \brief valor de desviación por defecto. Utilizada para corregir la desviación inicial del sensor.
 * 
 * \def ENABLED_SENSOR_MPX5050 1   // 1 for MPX5050. 0 for MPX5010
 * \brief Variable que indica si el sensor conectado es el MPX5050 o el 5010.
 *  
 * 
 * \struct CiclosEPROM
 * \brief esta estructura conserva los ciclos que lleva funcionando el respirador y la cantidad de escrituras en EPROM realizadas.
 * \var ciclos 
 * \var escriturasEPROM
 * 
 * \struct VentilationOptions_t
 * \brief Estructura de parámetros ventilación.
 * 
 * \var height  
 * \brief Altura del paciente
 * 
 * \var sex 
 * \brief Sexo del paciente.
 * 
 * \var respiratoryRate. 
 * \brief Tasa respiratoria.
 * 
 * \var peakInspiratoryPressure 
 * \brief Presión inspiratoria pico.
 * 
 * \var peakEspiratoryPressure 
 * \brief Presión espiratoria pico.
 * 
 * \var triggerThreshold 
 * \brief Umbral de disparo.
 * 
 * \var hasTrigger
 * \brief Intención de respiración.
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

#define DEBUG_UPDATE 0
#define DEBUG_PLOT 0
//#define DEBUG_STATE_MACHINE 1

// Timebase
#define TIME_BASE               20   // msec
#define TIME_SENSOR             100  // msec //estaba en 100
#define TIME_EPROM             6000  //  100ms x 6000 = 600000ms = 600s= 10m
#define TIME_SEND_CONFIGURATION 2000 // msec

// Sensors
#define ENABLED_SENSOR_VOLUME 1
#if ENABLED_SENSOR_VOLUME
#define ENABLED_SENSOR_VOLUME_SFM3300 0
#define ENABLED_SENSOR_VOLUME_STEPPER 1
#define ENABLED_SENSOR_VOLUME_byPRESSURE 0
#endif
// Bag Calibration for AMBU Adult bag
/**
* \struct COEFFS
* \brief Estructura con coeficientes para la calibración de la bolsa ambu de adulto.
* 
* \def AMBU_MAX_ML
* \brief Volumen máximo en ml de la bolsa ambu para dultos.
*/
const struct {float a, b, c;} COEFFS{0.008031, 1.018,-58.37};
#define AMBU_MAX_ML 470.0F

// Stepper
/**
* Stepper defines
* 
* \def STEPPER_MICROSTEPS
* \brief Micropasos del motor paso a paso
* 
* \def STEPPER_STEPS_PER_REVOLUTION
* \brief Cantidad de pasos por revolución del motor paso a paso
* 
* \def STEPPER_MICROSTEPS_PER_REVOLUTION 
* \brief Cantidad de micropasos por revolución del motor. Se calcula como STEPPER_MICROSTEPS_PER_REVOLUTION=STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS
* 
* \def STEPPER_DIR 
* \brief dirección de movimiento del motor paso a paso.
* 
* \def STEPPER_HOMING_DIRECTION
* \brief dirección de movimiento del motor paso a paso durante la rutina de posicionamiento inicial.
* 
* \def STEPPER_HOMING_SPEED
* \brief velocidad de movimiento del motor paso a paso durante la rutina de posicionamiento inicial.
* 
* \def STEPPER_LOWEST_POSITION     
* \brief mínima posición en pasos que puede alcanzar el eje motor. Físicamente se traduce en la bolsa ambu sin presionar
* 
* \def STEPPER_HIGHEST_POSITION   
* \brief máxima posición en pasos que puede alcanzar el eje motor. Físicamente se traduce en la bolsa ambu totalmente presionada.
* 
* \def STEPPER_SPEED_DEFAULT
* \brief velocidad por defecto del movimiento del motor paso a paso.
* 
* \def STEPPER_SPEED_MAX
* \brief Máxima velocidad del movimiento del motor paso a paso. Limite superior
* 
* \def STEPPER_ACC_MAX
* \brief Máxima aceleración del movimiento del motor paso a paso. Limite superior
* 
* \def STEPPER_SPEED_MAX_STOP
* \brief Máxima velocidad del movimiento del motor paso a paso cuando se lo quiere parar.
* 
* \def STEPPER_ACC_MAX_STOP
* \brief Máxima aceleración del movimiento del motor paso a paso cuando se lo quiere parar.
*
* \def STEPPER_ACC_EXSUFFLATION
* \brief Aceleración del movimiento del motor paso a paso durante la exuflación.

* \def STEPPER_ACC_INSUFFLATION
* \brief Aceleración del movimiento del motor paso a paso durante la insuflación.
*
*/
#define STEPPER_MICROSTEPS 4 // Según Jumpers de Ramps (1/2 hay uno es medio paso)
#define STEPPER_STEPS_PER_REVOLUTION 200  // ESTE PARA METRO LO AFECTA LA REDUCCION 200*4 (ESTANDAR * REDUCCION)

#define STEPPER_MICROSTEPS_PER_REVOLUTION (STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS)
#define STEPPER_DIR                 -1 // negativo porque se aleja del home
#define STEPPER_HOMING_DIRECTION    (1)
#define STEPPER_HOMING_SPEED        (STEPPER_MICROSTEPS * 300)   // steps/s
#define STEPPER_LOWEST_POSITION     (STEPPER_DIR * 2)//STEPPER_MICROSTEPS *  1)   // Ambu sin apretar
#define STEPPER_HIGHEST_POSITION    (STEPPER_DIR * STEPPER_MICROSTEPS *  181)  // Ambu apretado, limitacion fisica de apriete.
#define STEPPER_SPEED_DEFAULT       2000 
#define STEPPER_SPEED_MAX           4000 //4000 CICLO 30
#define STEPPER_ACC_MAX             9300
#define STEPPER_SPEED_MAX_STOP      1900 //4000 CICLO 30
#define STEPPER_ACC_MAX_STOP        9300
#define STEPPER_ACC_EXSUFFLATION    (STEPPER_MICROSTEPS *  600)  // steps/s2
#define STEPPER_ACC_INSUFFLATION    (STEPPER_MICROSTEPS *  450)  // steps/s2

  

/**
 * \def DEFAULT_HEIGHT 
 * \brief Altura del paciente por defecto.
 * 
 * \def DEFAULT_SEX
 * \brief Sexo del paciente por defecto, siendo 0=varón, 1=mujer.
 * 
 * \def DEFAULT_ML_PER_KG_IDEAL_WEIGHT
 * \brief Cantidad de mililitros de aire por Kg, se considera el peso ideal.
 * 
 * \def DEFAULT_MAX_TIDAL_VOLUME
 * \brief Volumen tidal máximo por defecto.
 * 
 * \def DEFAULT_MIN_TIDAL_VOLUME 
 * \brief Volumen tidal mínimo por defecto.
 * 
 * \def DEFAULT_DIFF_TIDAL_VOLUME
 * \brief Variable que define la excursión máxima que puede tener el motor entre el máximo y el mínimo volumen tidal insuflado.
 * 
 * \def DEFAULT_TRIGGER_THRESHOLD
 * \brief Umbral de disparo por defecto. Se utiliza para detectar la intención de respiración por parte del paciente.
 * 
 * \def DEFAULT_RPM
 * \brief Revoluciones por minuto por defecto del motor.
 * 
 * \def DEFAULT_MAX_RPM
 * \brief Valor máximo de RPM del motor.
 * 
 * \def DEFAULT_MIN_RPM
 * \brief Valor mínimo de RPM del motor.
 * 
 * \def DEFAULT_INSPIRATORY_FRACTION
 * \brief Valor por defecto del I/E ratio.
 * 
 * \def DEFAULT_PEAK_INSPIRATORY_PRESSURE.
 * \brief Valor deseado de presión inspiratoria por defecto.
 * 
 * \def DEFAULT_PEAK_INSPIRATORY_PRESSURE_MIN
 * \brief Valor mínimo de presión inspiratoria por defecto.
 * 
 * \def DEFAULT_PEAK_INSPIRATORY_PRESSURE_MAX
 * \brief Valor máximo de presión inspiratoria por defecto.
 * 
 * \def DEFAULT_PEAK_ESPIRATORY_PRESSURE
 * \brief Valor deseado de presión espiratoria por defecto.
 * 
 * \def DEFAULT_PEAK_ESPIRATORY_PRESSURE_MIN
 * \brief Valor mínimo de presión espiratoria por defecto.
 * 
 * \def DEFAULT_PEAK_ESPIRATORY_PRESSURE_MAX
 * \brief Valor máximo de presión espiratoria por defecto.
 * 
 * \def STEPPER_PEEP_SOLENOID_HYSTERESIS  
 * \brief Valor de presión espiratoria por defecto en cmH2O para implementar un ciclo de histéresis con los solenoides.
 * 
 * \def DEFAULT_PORC_VOLTILDAL
 * \brief Porcentaje de volumen tidal insuflado por defecto.
 * 
 * \def DEFAULT_BYARRAY_VOLTILDAL         
 * \brief indice del arreglo volumen tidal
 * 
 * \def DEFAULT_PAUSE                     1
 * \brief Pausa por defecto. Normalmente este valor es referido como HOLD_IN
 * 
 * \def DEFAULT_TINS                     
 * \brief Tiempo de insuflación por defecto. 
*/
#define DEFAULT_HEIGHT                    170 // cm
#define DEFAULT_SEX                       0   // 0: varón, 1: mujer
#define DEFAULT_ML_PER_KG_IDEAL_WEIGHT    7
#define DEFAULT_MAX_TIDAL_VOLUME          186
#define DEFAULT_MIN_TIDAL_VOLUME          0
#define DEFAULT_DIFF_TIDAL_VOLUME         193
#define DEFAULT_TRIGGER_THRESHOLD         0
#define DEFAULT_RPM                       10
#define DEFAULT_MAX_RPM                   30
#define DEFAULT_MIN_RPM                   10
#define DEFAULT_INSPIRATORY_FRACTION      0.5F
#define DEFAULT_PEAK_INSPIRATORY_PRESSURE 35
#define DEFAULT_PEAK_INSPIRATORY_PRESSURE_MIN 5
#define DEFAULT_PEAK_INSPIRATORY_PRESSURE_MAX 39
#define DEFAULT_PEAK_ESPIRATORY_PRESSURE  10
#define DEFAULT_PEAK_ESPIRATORY_PRESSURE_MIN 5
#define DEFAULT_PEAK_ESPIRATORY_PRESSURE_MAX 20
#define STEPPER_PEEP_SOLENOID_HYSTERESIS  0.8F     // cmH2O
#define DEFAULT_PORC_VOLTILDAL            70
#define DEFAULT_BYARRAY_VOLTILDAL         4
#define DEFAULT_PAUSE                     1
#define DEFAULT_TINS                     200

/**
 * \def DEFAULT_PA_TO_CM_H2O                     
 * \brief Constante de conversión de pascales a cmH2O
 * 
 */
// Pressure
#define DEFAULT_PA_TO_CM_H2O 0.0102F

/**         
 * 
 * \def DEFAULT_RECRUITMENT_TIMEOUT
 * \brief Tiempo de reclutamiento en ms por defecto. 
 * 
 * \def DEFAULT_RECRUITMENT_PIP     20    // cmH2O
 * \brief Peak Inspiratory Pressure de reclutamiento por defecto. 
 */
// Recruitment
#define DEFAULT_RECRUITMENT_TIMEOUT 20000 // msec
#define DEFAULT_RECRUITMENT_PIP     20    // cmH2O



// Overpressure that triggers valve
#define VALVE_MAX_PRESSURE 41 // cmH2O

// PID constants
// Variables de configuración PID. Versión antigua. No modificar ni borrar.
//#define PID_MIN       -5000
//#define PID_MAX        5000
#define PID_MIN       STEPPER_LOWEST_POSITION
#define PID_MAX       STEPPER_HIGHEST_POSITION
#define PID_KP         40
#define PID_KI         0
#define PID_KD         0
#define PID_TS         TIME_BASE
#define PID_BANGBANG   8

// Solenoid
/**
*variables de control de los solenoides 
*\def SOLENOID_CLOSED
*\def SOLENOID_OPEN 
*/
#define SOLENOID_CLOSED 0
#define SOLENOID_OPEN   1


/*
 * Variables de Pinout
 * 
 * \def PIN_STEPPER_STEP
 * \brief Pin STEP del controlador para motor paso a paso.
 * 
 * \def PIN_STEPPER_DIRECTION
 * \brief Pin DIR del controlador para motor paso a paso.
 * 
 * \def PIN_STEPPER_EN
 * \brief Pin EN del controlador para motor paso a paso.
 * 
 * \def PIN_BUZZ
 * \brief Pin digital para activar el buzzer.
 * 
 * \def PIN_STEPPER_ENDSTOP
 * \brief Pin de entrada para el sensor fin de carrera.
 * 
 * \def PIN_SOLENOID1
 * \brief Pin de activación del solenoide 1.
 * 
 * \def PIN_SOLENOID2
 * \brief Pin de activación del solenoide 2.
 * 
 *  \def PIN_TEST220V
 * \brief Pin de deteccion de 220V, analogico para ACS712, sino puede ser digital con UPS led.
 */

#define PIN_STEPPER_STEP        26
#define PIN_STEPPER_DIRECTION   28
#define PIN_STEPPER_EN          24

#define PIN_BUZZ                11
#define PIN_STEPPER_ENDSTOP     3

#define PIN_SOLENOID1            9
#define PIN_SOLENOID2            10

#define PIN_TEST220V              A10 //cambiar por A3 para ramps


/**
* \def ALARM_MAX_PRESSURE 
* \brief  Máxima presión en cmH2O permitida.
* 
* \def ALARM_MIN_PRESSURE 3  // cmH2O ESTA ALARMA SE INTERPRETA COMO SENSOR DESCONECTADO
* \brief  Mínima presión en cmH2O permitida.
* 
* Codificación de las alarmas. Estos códigos se envían a la Tablet.
* Cada código tiene su versión de activación y desactivación.
* 
* \def No_Alarm                
*
* Alarma de sensor desconectado 
* \def Alarm_desconected_EN 
* \def Alarm_desconected_DIS 
* 
* Alarma de sobrepresión
* \def Alarm_Overpressure_EN
* \def Alarm_Overpressure_DIS
* 
* Alarma de intención de respiración
* \def Alarm_breathe_EN
* \def Alarm_breathe_DIS
* 
* Alarma de falta de flujo
* \def Alarm_No_Flux_EN       
* \def Alarm_No_Flux_DIS      
* 
* Códigos libres para futuras implementaciones de alarmas
* \def Alarm_Libre_1_EN
* \def Alarm_Libre_1_DIS
*
*\def Alarm_Libre_2_EN        0x2020
*\def Alarm_Libre_2_DIS       0xDFFF*
*
*\def Alarm_Libre_3_EN        0x4040
*\def Alarm_Libre_3_DIS       0xBFFF
*
*\def Alarm_Libre_4_EN        0x8080
*\def Alarm_Libre_4_DIS       0X7FFF
*
*/

#define ALARM_MAX_PRESSURE 38 // cmH2O
#define ALARM_MIN_PRESSURE 3  // cmH2O ESTA ALARMA SE INTERPRETA COMO SENSOR DESCONECTADO

#define No_Alarm                0x0000

#define Alarm_desconected_EN    0x0101
#define Alarm_desconected_DIS   0xFEFF

#define Alarm_Overpressure_EN   0x0202
#define Alarm_Overpressure_DIS  0xFDFF

#define Alarm_breathe_EN        0x0404
#define Alarm_breathe_DIS       0xFBFF

#define Alarm_No_Flux_EN        0x0808
#define Alarm_No_Flux_DIS       0xF7FF

#define Alarm_Libre_1_EN        0x1010
#define Alarm_Libre_1_DIS       0xEFFF

#define Alarm_Libre_2_EN        0x2020
#define Alarm_Libre_2_DIS       0xDFFF

#define Alarm_Libre_3_EN        0x4040
#define Alarm_Libre_3_DIS       0xBFFF

#define Alarm_Libre_4_EN        0x8080
#define Alarm_Libre_4_DIS       0X7FFF

/**
 *  Pinout para sensor MPX 5050/5010
 * 
 * \def PIN_MPX_DATA
 * \brief Pin de datos del sensor de presión MPX
 * 
 * \def PIN_MPX_FLOW    
 * \brief Pin de datos del sensor de flujo MPX
 * 
 * \def DEFAULT_OFFSET_MPX
 * \brief valor de desviación por defecto. Utilizada para corregir la desviación inicial del sensor.
 * 
 * \def DEFAULT_OFFSET_MPX2 210.0F
 * \brief valor de desviación por defecto. Utilizada para corregir la desviación inicial del sensor.
 * 
 * \def ENABLED_SENSOR_MPX5050 1   // 1 for MPX5050. 0 for MPX5010
 * \brief Variable que indica si el sensor conectado es el MPX5050 o el 5010.
 *  
*/


#define PIN_MPX_DATA		   A5//A5	// A0
#define PIN_MPX_FLOW       A9
#define ENABLED_SENSOR_MPX 1	
#if ENABLED_SENSOR_MPX
#define DEFAULT_OFFSET_MPX 185.0F//185.0F//205
#define DEFAULT_OFFSET_MPX2 210.0F
#define ENABLED_SENSOR_MPX5050 0 	// 1 for MPX5050. 0 for MPX5010
#endif


/**
* \struct CiclosEPROM
* \brief esta estructura conserva los ciclos que lleva funcionando el respirador y la cantidad de escrituras en EPROM realizadas.
*
*/
struct CiclosEPROM {
  unsigned long  ciclos;
  unsigned int escriturasEPROM;
};

/*
* \struct VentilationOptions_t
* \brief Estructura de parámetros ventilación.
* 
* \var height  
* \brief Altura del paciente
* 
* \var sex 
* \brief Sexo del paciente.
* 
* \var respiratoryRate. 
* \brief Tasa respiratoria.
* 
* \var peakInspiratoryPressure 
* \brief Presión inspiratoria pico.
* 
* \var peakEspiratoryPressure 
* \brief Presión espiratoria pico.
* 
* \var triggerThreshold 
* \brief Umbral de disparo.
* 
* \var hasTrigger
* \brief Intención de respiración.
*/
typedef struct {
    short height;
    bool sex;
    short respiratoryRate;
    short peakInspiratoryPressure;
    short peakEspiratoryPressure;
    float triggerThreshold;
    bool hasTrigger;
} VentilationOptions_t;

#endif // DEFAULTS_H

/** 
 *
 * \file mpx.h
 * \details Este archivo contiene las declaraciones de las funciones utilizadas para leer los sensores MPX 50XX  
 * así como tambien las estructureas de datos de dichos sensores.
 *\def    byPRESSURE_DELTA     
 *\brief   valor de densisdad en kg/m3
 *
 *\def    byPRESSURE_SECCION 
 *\brief  Sección del tubo conductor
 *
 *\def    byPRESSURE_KDELTA  
 *\brief este parámetro interno se calcula como ((-16,893) * byPRESSURE_SECCION* byPRESSURE_SECCION)
 * 
 * \class MPX_PRESSURE_SENSOR
 *  \brief Clase que almacena valores del sensosr de presión
 *  \var max_pressure    Valor máximo de presión en kPa 
 *  \var min_pressure    Valor mínimo de presión en kPa
 *  \var pressure        Valor de lectura en kPa
 *  \var _offset         Valor en volts
 *  \var _pin            Número de pin donde se conecta el sensor
 * 
 * \fn float autoajuste (float offset)
 * 
 * \brief Función de autoajuste para sensor de presion MPX.
 * \param offset: valor de offset con el cual se compara
 * \return nuevo valor de offset ajustado
 * \details toma 10 medidas, realiza un promedio y compara el valor con el offset ingresado. 
 * En base a esa ccomparación realiza un ajuste del valor del offset para el sensor de presión.
 * 
 * \fn void  setOffsetValue(float value)
 * 
 * \brief Función de seteo de la variable offset para sensor de presion MPX.
 * \param value: valor a setear
 * \return ninguno
 * 
 * \fn float getOffsetValue(void)
 *  
 * \brief Función que permite ver el valor de la variable offset para sensor de presion MPX.
 * \param ninguno
 * \return valor de la variable de offset
 * 
 * 
 * \fn MPX_PRESSURE_SENSOR(uint8_t pin)
 * 
 * \brief Constructor del objeto sensor de presion MPX.
 * \param ninguno
 * \return ninguno
 * 
 * \fn void MPX_SET_ADC(void)
 * 
 * \brief Configura la tensión de referencia del ADC basado en el sensor definido por ENABLED_SENSOR_MPX5050.
 * \param ninguno
 * \return ninguno
 * \details Si ENABLED_SENSOR_MPX5050 = 1 la tensión de refencia es 1.1V, caso contrario la tensión será 2.56V
 * 
 * \fn float MPX_READ_PRESSURE(void)
 * 
 * \brief Realiza la lectura de presión analogica
 * \param ninguno
 * \return Valor que representa el voltaje de la presion sin offset
 * \details Dependiendo de la definición de ENABLED_SENSOR_MPX5050 mapea la lectura del pin PIN_MPX_DATA en los valores correcto
 *  
 * \fn float MPX_CONVERT_PRESSURE(float Vfinal)
 * 
 * \brief Realiza conversion a KPa desde la tension leida previemente
 * \param Tensión devuelta por MPX_PRESSURE_SENSOR::MPX_READ_PRESSURE
 * \return Valor que representa la presion leída en KPa
 * \details Dependiendo de la definición de ENABLED_SENSOR_MPX5050 mapea en los valores correcto.
 * Vout = VS x (0.09 x P + 0.04) = 0.45 x P + 0.2 
 * P = (Vout - 0.2)/0.45 = Vfinal /0.45 
 * MPX 5050
 * Vout = VS (P x 0.018 + 0.04)
 * P = Vfinal / 0.09
 * 
 * 
 * \fn MPX_CONVERT_cmH20(float pressureKPA)
 * 
 * \brief  Realiza conversion del valor de presión en KPa a cmH20
 * \param  Valor de presión en KPa
 * \return Valor en float que representa la presion en cmH2O
 * 
 * \fn float MPX_CONVERT_flujo();
 * 
 * \brief  Realiza una estimación del flujo basándose en mediciones de presión
 * \param  ninguno
 * \return Valor de flujo estimado 
 * \details Solo funciona si está habilitada la  variable ENABLED_SENSOR_VOLUME_byPRESSURE
 * 
 * 
 * ~MPX_PRESSURE_SENSOR()
 * 
 * \brief Destructor del objeto MPX_PRESSURE_SENSOR
 * 
 */ 
 
#ifndef _MPX_H_
#define _MPX_H_

#include "defaults.h"
#include "Arduino.h"

#if ENABLED_SENSOR_VOLUME_byPRESSURE
#define    byPRESSURE_DELTA 1.1839    //   kg/m3  346.13m/s   -> a 25�C
#define    byPRESSURE_SECCION 0.14
#define    byPRESSURE_KDELTA  3 //((-16,893) * byPRESSURE_SECCION* byPRESSURE_SECCION)
#endif


 class MPX_PRESSURE_SENSOR
{

private:

	float max_pressure;		//kPa 
	float min_pressure;		//kPa
	float pressure;			//Valor de lectura en kPa
	float _offset;			//Valor en volts
  uint8_t _pin;

public:

/**
 * float autoajuste (float offset)
 * @brief Función de autoajuste para sensor de presion MPX.
 * @param offset: valor de offset con el cual se compara
 * @return nuevo valor de offset ajustado
 * @details toma 10 medidas, realiza un promedio y compara el valor con el offset ingresado. 
 * En base a esa ccomparación realiza un ajuste del valor del offset para el sensor de presión.
 */ 
  float autoajuste (float offset);

  /**
   * void  setOffsetValue(float value)
   * 
   * @brief Función de seteo de la variable offset para sensor de presion MPX.
   * @param value: valor a setear
   * @return ninguno
   */
  void  setOffsetValue(float value);
  
/** 
 *  float getOffsetValue(void)
 *  
 * @brief Función que permite ver el valor de la variable offset para sensor de presion MPX.
 * @param ninguno
 * @return valor de la variable de offset
 */
  float getOffsetValue(void);

/**
 * MPX_PRESSURE_SENSOR(uint8_t pin)
 * 
 * @brief Constructor del objeto sensor de presion MPX.
 * @param ninguno
 * @return ninguno
 * 
 */  
	MPX_PRESSURE_SENSOR(uint8_t pin);

/**
 * void MPX_SET_ADC(void)
 * 
 * @brief Configura la tensión de referencia del ADC basado en el sensor definido por ENABLED_SENSOR_MPX5050.
 * @param ninguno
 * @return ninguno
 * @details Si ENABLED_SENSOR_MPX5050 = 1 la tensión de refencia es 1.1V, caso contrario la tensión será 2.56V
 * 
 */  
	void MPX_SET_ADC(void);	

/**
 * float MPX_READ_PRESSURE(void)
 * 
 * @brief Realiza la lectura de presión analogica
 * @param ninguno
 * @return Valor que representa el voltaje de la presion sin offset
 * @details Dependiendo de la definición de ENABLED_SENSOR_MPX5050 mapea la lectura del pin PIN_MPX_DATA en los valores correcto
 * 
 */ 
  float MPX_READ_PRESSURE(void);	

/**
 * float MPX_CONVERT_PRESSURE(float Vfinal)
 * 
 * @brief Realiza conversion a KPa desde la tension leida previemente
 * @param Tensión devuelta por MPX_PRESSURE_SENSOR::MPX_READ_PRESSURE
 * @return Valor que representa la presion leída en KPa
 * @details Dependiendo de la definición de ENABLED_SENSOR_MPX5050 mapea en los valores correcto.
 * MPX 5010
 * Vout = VS x (0.09 x P + 0.04) = 0.45 x P + 0.2 
 * P = (Vout - 0.2)/0.45 = Vfinal /0.45 
 * MPX 5050
 * Vout = VS (P x 0.018 + 0.04)
 * P = Vfinal / 0.09
 */ 	
	float MPX_CONVERT_PRESSURE(float Vfinal);

/**
 * MPX_CONVERT_cmH20(float pressureKPA)
 * 
 * @brief  Realiza conversion del valor de presión en KPa a cmH20
 * @param  Valor de presión en KPa
 * @return Valor en float que representa la presion en cmH2O
 * 
 */   
	float MPX_CONVERT_cmH20(float pressureKPA);


/**
 * float MPX_CONVERT_flujo();
 * 
 * @brief  Realiza una estimación del flujo basándose en mediciones de presión
 * @param  ninguno
 * @return Valor de flujo estimado 
 * @details Solo funciona si está habilitada la  variable ENABLED_SENSOR_VOLUME_byPRESSURE
 * 
 */ 
 
  #if ENABLED_SENSOR_VOLUME_byPRESSURE
       float MPX_CONVERT_flujo(); 
  #endif
  
/**
 * ~MPX_PRESSURE_SENSOR()
 * 
 * @brief Destructor
 * @param void
 * @return void
 */   
	~MPX_PRESSURE_SENSOR(){};

};

   
#endif // MPX_H

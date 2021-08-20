/** Funciones de cálculo necesarias
 *
 * @file calc.h
 * @details Este archivo contiene las declaraciones de las funciones de cálculo necesarias.
 * 
 * \fn int estimateTidalVolume(int estatura, int sexo)
 * 
 * \brief estima el volumen tidal en función de estatura y sexo, en ml.
 * \param estatura en cm, del paciente
 * \param sexo 0: varón, 1: mujer, sexo del paciente
 * \return *volumenTidal volumen tidal estimado, en mililitros
 *
 * \fn int QuickSelectMedian(unsigned int arr[], uint8_t n)
 * 
 * \brief filtro de media movil.
 * \param estatura en cm, del paciente
 * \param sexo 0: varón, 1: mujer, sexo del paciente
 * \return señal filtrada.
 *
 *
 * \fn void refreshWatchDogTimer()
 * 
 * \brief Refresca el WDT (Watch Dog Timer)
 * \param ninguno
 * \returm ninguno
 * 
 */
#ifndef CALC_H
#define CALC_H

#include "defaults.h"
#include "Arduino.h"
#include <avr/wdt.h>
//#include <math.h>// o <cmath>

/**
 * \fn int estimateTidalVolume(int estatura, int sexo)
 * \brief estima el volumen tidal en función de estatura y sexo, en ml.
 *
 * \param estatura en cm, del paciente
 * \param sexo 0: varón, 1: mujer, sexo del paciente
 * \return *volumenTidal volumen tidal estimado, en mililitros
 */
int estimateTidalVolume(int estatura, int sexo);

/**
 * \fn int QuickSelectMedian(unsigned int arr[], uint8_t n)
 * 
 * \brief filtro de media movil.
 * \param estatura en cm, del paciente
 * \param sexo 0: varón, 1: mujer, sexo del paciente
 * \return señal filtrada.
 */
unsigned int QuickSelectMedian(unsigned int arr[], uint8_t n);
/**
 * \fn void refreshWatchDogTimer()
 * \brief Refresca el WDT (Watch Dog Timer)
 * \param ninguno
 * \returm ninguno
 */
void refreshWatchDogTimer();

#endif // CALC_H

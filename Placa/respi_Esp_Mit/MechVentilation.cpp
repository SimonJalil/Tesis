/** Ventilación mecánica
 *
 * @file    MechVentilation.cpp
 * @details Este archivo contiene las definiciones de los métodos para controlar la ventilación mecánica a través de todos sus componentes.
 */

#include "MechVentilation.h"

int currentWaitTriggerTime = 0;
int currentStopInsufflationTime = 0;
float currentFlow = 0;

MechVentilation::MechVentilation(FlexyStepper *stepper, Sensors *sensors,
                                 AutoPID *pid, VentilationOptions_t options) {
  _init(stepper, sensors, pid, options);
}


boolean MechVentilation::getStartWasTriggeredByPatient() {  // returns true if last respiration cycle
                                       // was started by patient trigger. It is
                                       // cleared when read.
  if (_startWasTriggeredByPatient) {
    return true;
  } else {
    return false;
  }
}


boolean MechVentilation::getSensorErrorDetected() {  // returns true if there was an
                                             // sensor error detected. It is
                                             // cleared when read.
  if (_sensor_error_detected) {
    return true;
  } else {
    return false;
  }
}


void MechVentilation::start(void) { _running = true; }
 

void MechVentilation::stopMech(void) { _running = false; }


uint8_t MechVentilation::getRunning(void) { return(_running); }


uint8_t MechVentilation::getRPM(void) { return _rpm; }


short MechVentilation::getExsuflationTime(void) { return _timeoutEsp; }


short MechVentilation::getInsuflationTime(void) { return _timeoutIns; }


short MechVentilation::getPeakInspiratoryPressure(void) { return _pip; }


short MechVentilation::getPeakEspiratoryPressure(void) { return _peep; }

short MechVentilation::getIer(void) { return ((short)(_ier * 100)); }

short MechVentilation::getTh(void) { return _triggerThreshold; }

State MechVentilation::getState(void) { return _currentState; }

short MechVentilation::getVolTidalByArray(void) {
  /* Esta funcion debe convertir a porcentaje pasos que indican volumen ml,
   * puede no ser lineal */
   return(_vt);
}

short MechVentilation::getVolTidalPorcen(void) {
  /* Esta funcion debe convertir a porcentaje pasos que indican volumen ml,
   * puede no ser lineal */
  float aux = (COEFFS.a * _volTidalSteps * _volTidalSteps) +(COEFFS.b * _volTidalSteps) + COEFFS.c;  // volumen ml
  short auxx = (short)((aux * 100.0) / AMBU_MAX_ML) + 1;
  return auxx;
}
void MechVentilation::setPause(uint8_t pause){
  _pause = pause;
}

uint8_t MechVentilation::getPause(void){
  return(_pause);
}

void MechVentilation::setVolTidalInStep(uint8_t vt) {
  // Esta funcion debe convertir a pasos desde volumen porcentual
  float aux = (vt * AMBU_MAX_ML) / 100.0;  // Obtengo ml desde el porcentaje de apriete del AMBU,
                      // depende del máximo volumen posible a sacar
  //------------------ Ajustar modelo ------------------//
  _volTidalSteps = (-COEFFS.b + sqrt((COEFFS.b * COEFFS.b) -4 * COEFFS.a * (COEFFS.c - aux))) /
                   (2 * COEFFS.a);
  if (_volTidalSteps > DEFAULT_MAX_TIDAL_VOLUME) _volTidalSteps = DEFAULT_MAX_TIDAL_VOLUME;
  if (_volTidalSteps < DEFAULT_MIN_TIDAL_VOLUME) _volTidalSteps = DEFAULT_MIN_TIDAL_VOLUME;
}

void MechVentilation::setVolTidalByArray(uint8_t vt) {
  // Esta funcion debe convertir a pasos desde volumen porcentual
  //float aux = (vt * AMBU_MAX_ML) / 100.0;  // Obtengo ml desde el porcentaje de apriete del AMBU,
                      // depende del máximo volumen posible a sacar
  //------------------ Ajustar modelo ------------------//
  if(vt > 19) vt = 19;
  if(vt >= 14 && _rpm == 26){
    vt = 14;
  }
  else{
    if(vt >= 12 && _rpm == 27){
      vt = 12;
    }
    else{
      if(vt >= 10 && _rpm == 28){
        vt = 10;
      }
      else{
        if(vt >= 8 && _rpm >= 29){
          vt = 8;
        }
      }
    }
  }
   _vt = vt;
  _volTidalSteps = VoluTidal[_vt];
  if (_volTidalSteps > DEFAULT_MAX_TIDAL_VOLUME) _volTidalSteps = DEFAULT_MAX_TIDAL_VOLUME;
  if (_volTidalSteps < DEFAULT_MIN_TIDAL_VOLUME) _volTidalSteps = DEFAULT_MIN_TIDAL_VOLUME;
}

void MechVentilation::setRPM(uint8_t rpm) {
  if(rpm<=DEFAULT_MIN_RPM) rpm =DEFAULT_MIN_RPM;
  if(rpm>=DEFAULT_MAX_RPM) rpm =DEFAULT_MAX_RPM;
  if (rpm== 26 && _vt >= 14){
    setVolTidalByArray(14);
    }
  else{
    if (rpm== 27 && _vt >= 12) {
      setVolTidalByArray(12);
    }
    else{
      if (rpm== 28 && _vt >= 10) {
        setVolTidalByArray(10);
      }
      else{
        if(rpm>= 29 && _vt >= 8) setVolTidalByArray(8);
      }
    }
  }
  _rpm = rpm;
  _setInspiratoryCycle();
}

void MechVentilation::setPeakInspiratoryPressure(uint8_t pip) { 
  if (pip < DEFAULT_PEAK_INSPIRATORY_PRESSURE_MIN)  pip = DEFAULT_PEAK_INSPIRATORY_PRESSURE_MIN;
  if (pip > DEFAULT_PEAK_INSPIRATORY_PRESSURE_MAX) pip = DEFAULT_PEAK_INSPIRATORY_PRESSURE_MAX;
  //if(pip <= _peep) pip = DEFAULT_PEAK_INSPIRATORY_PRESSURE;
  _pip = pip;
  }

void MechVentilation::setPeakEspiratoryPressure(uint8_t mypeep) { 
  Serial.print(mypeep); 
  if (mypeep < DEFAULT_PEAK_ESPIRATORY_PRESSURE_MIN)  mypeep = DEFAULT_PEAK_ESPIRATORY_PRESSURE_MIN;
  if (mypeep > DEFAULT_PEAK_ESPIRATORY_PRESSURE_MAX) mypeep = DEFAULT_PEAK_ESPIRATORY_PRESSURE_MAX;
  if( mypeep >= _pip) mypeep = DEFAULT_PEAK_ESPIRATORY_PRESSURE_MAX;
  _peep = mypeep;
  Serial.print(_peep); 
  }

void MechVentilation::setIer(uint8_t ier) {
  if (ier < 25) ier = 25;
  if (ier > 50) ier = 50;
  _ier = ((float)(ier)) / 100.0;
  _setInspiratoryCycle();
}

void MechVentilation::setTh(uint8_t thr) { 
  if(thr > _pip-5) thr =  _pip-5;
  _triggerThreshold = thr;
  }

void MechVentilation::goToPositionByDur(FlexyStepper *stepper, int goal_pos, int cur_pos, int dur) {
  // Medidas de seguridad mecanicas
  if (dur <= 0) return;  
  if (goal_pos > 0) return;
  if (cur_pos > 0) return;
  if (goal_pos < -DEFAULT_MAX_TIDAL_VOLUME) goal_pos= -DEFAULT_MAX_TIDAL_VOLUME;
  if (goal_pos > -DEFAULT_MIN_TIDAL_VOLUME) goal_pos =-DEFAULT_MIN_TIDAL_VOLUME;
  //dur = dur * 2;
  long dist = goal_pos - cur_pos;
  if (dist > DEFAULT_DIFF_TIDAL_VOLUME ) dist = DEFAULT_DIFF_TIDAL_VOLUME;
  if (dist < -DEFAULT_DIFF_TIDAL_VOLUME ) dist = -DEFAULT_DIFF_TIDAL_VOLUME;
 
  dist = dist * 1000; 
  long vel = (abs(dist) / dur) * 12; //STEPPER_MICROSTEPS /// 2 ;//*4; ;
  dist = dist / 1000;

//  if (vel > STEPPER_SPEED_MAX) {
//    vel = STEPPER_SPEED_MAX;
//  }
  
  vel = vel * 1000;
  long acc = (vel / dur);             // Constant acc in and out
  vel = vel / 1000; 

//  if (acc > STEPPER_ACC_MAX) {
//    acc = STEPPER_ACC_MAX;
//  }
  _stepper->setSpeedInStepsPerSecond(vel*3/5);
  _stepper->setAccelerationInStepsPerSecondPerSecond((acc)*11/10);//*750);
  //_stepper->setAccelerationInStepsPerSecondPerSecond(STEPPER_ACC_MAX);
  long setStep = dist + cur_pos;
 // ojo evaluar que pasa 22_5 <------------------------------------------------------------------------------------ 
  if (setStep > 0) setStep = 0;
  if (setStep < -DEFAULT_MAX_TIDAL_VOLUME) setStep = -DEFAULT_MAX_TIDAL_VOLUME;
  _stepper->setTargetPositionInSteps(STEPPER_MICROSTEPS * setStep);
}

void MechVentilation::_setInspiratoryCycle(void) 
{
//  float localrpm = _rpm;
//  float timeoutCycle =(60000.0 / (localrpm));  // T     Tiempo de ciclo en msegundos
//  float IE=1/_ier;
//  _tHoldIn = timeoutCycle/(1 + IE);  // DEFAULT_INSPIRATORY_FRACTION debe ser variable por consola y este es el valor x default
//  _timeoutIns = _tHoldIn - HOLD_IN_DURATION;  // Tin
//  _timeoutEsp = timeoutCycle - _tHoldIn;
//  _durac_exu = min(_tHoldIn + MAX_EX_DURATION, timeoutCycle - MIN_PEEP_PAUSE);//<------------------cambio 13/05
float divisor=0,aux=0;
  
  aux=getIer();
  
  if(aux==25){ //1:4
      divisor=5;
      }
      else{ //si no es 25 debo preguntar si es 33
         if(aux==33){//1:3
              divisor=4;
         }
         else{ //si no es 33 debe ser 50 (o lo fuerzo a ser 50 por seguridad)
             divisor=3;
         }
      }

  float localrpm = _rpm;
  float timeoutCycle =(60000.0 / (localrpm));  // T     Tiempo de ciclo en msegundos
  
  _tHoldIn= timeoutCycle / divisor;//el tiempo de inspiración es la fracción que resultade dividir el tiempo total en el divisor.
  _timeoutIns = _tHoldIn ;
  _timeoutEsp = timeoutCycle - _tHoldIn - HOLD_IN_DURATION;
  //_durac_exu = min(_tHoldIn + MAX_EX_DURATION, timeoutCycle - MIN_PEEP_PAUSE);
  _durac_exu = _tHoldIn;


}

void MechVentilation::evaluatePressure(void) {

  //alarma de sobrepresion
  
//  if (_currentPressure > ALARM_MAX_PRESSURE) {
//    //_currentAlarm = setAlarms(Alarm_Overpressure_EN);
//    setAlarms(Alarm_Overpressure_EN);
//  }
//  else{
//    resetAlarms(Alarm_Overpressure_DIS);
//  }
//  
//  if (_currentPressure < ALARM_MIN_PRESSURE)
//   {
// 
//   //_currentAlarm = setAlarms(Alarm_desconected_EN);
//        setAlarms(Alarm_desconected_EN);
//   }else{
//        resetAlarms(Alarm_desconected_DIS);
//   }

//   if(_startWasTriggeredByPatient){
//        setAlarms(Alarm_breathe_EN);
//   }else{
//        resetAlarms(Alarm_desconected_DIS);
//   }
  
  
  // Valve
  if (_currentPressure > VALVE_MAX_PRESSURE) {
     digitalWrite(PIN_SOLENOID1, SOLENOID_OPEN);
//    digitalWrite(PIN_SOLENOID2, SOLENOID_OPEN);
  }

}

void MechVentilation::evaluate220V(void) {
  unsigned long bufV[]={0,0,0,0,0,0,0,0}, Vin;
  static unsigned int cuenta0=0; 
  bufV[0] = (analogRead(PIN_TEST220V));
  bufV[1] = (analogRead(PIN_TEST220V));
  bufV[2] = (analogRead(PIN_TEST220V));
  bufV[3] = (analogRead(PIN_TEST220V));
  bufV[4] = (analogRead(PIN_TEST220V));
  bufV[5] = (analogRead(PIN_TEST220V));
  bufV[6] = (analogRead(PIN_TEST220V));
  bufV[7] = (analogRead(PIN_TEST220V));
  Vin = (bufV[0]+bufV[1]+bufV[2]+bufV[3]+bufV[4]+bufV[5]+bufV[6]+bufV[7])/8;
  // print out the value you read:
  if (abs(Vin) < 400) cuenta0++;
  else              cuenta0=0;
  if(cuenta0 > 100){
    setAlarms(0x01);
    cuenta0 = 100;
  }
  else{
    resetAlarms(0x00);
  }
  //Serial.println(Vin);
}

void MechVentilation::activateRecruitment(void) {
  _nominalConfiguration.pip = _pip;
  _nominalConfiguration.timeoutIns = _timeoutIns;
  _pip = DEFAULT_RECRUITMENT_PIP;
  _timeoutIns = DEFAULT_RECRUITMENT_TIMEOUT;
  _recruitmentMode = true;
  setState(Init_Insufflation);
}

void MechVentilation::deactivateRecruitment(void) {
  _pip = _nominalConfiguration.pip;
  _timeoutIns = _nominalConfiguration.timeoutIns;
  _recruitmentMode = false;
  setState(Init_Exsufflation);
}

void MechVentilation::SumPlateau(uint8_t PressurePlateau){
  PressurePlateauTotal += PressurePlateau;
  PressurePlateauCont++;
}
void MechVentilation::PromPlateau(void){
  PressurePlateauProm = (uint8_t)(PressurePlateauTotal/PressurePlateauCont);
  PressurePlateauCont = 0;
  PressurePlateauTotal = 0; 
}
uint8_t MechVentilation::GetPlateau(void){
  return(PressurePlateauProm);
}
uint8_t MechVentilation::getFrReal(void){
  return(_difTime);
}

unsigned long MechVentilation::GetCiclos(void){
  return(_ciclos);
}

void MechVentilation::ResetCiclos(void){
  _ciclos=0;
}
/**
 * It's called from timer1Isr
 */
void MechVentilation::update(void) {
  static bool flag_exceso=false;
  static int totalCyclesInThisState = 0;
  static int currentTime = 0;
  static int flowSetpoint = 0;
  static long actualPos = 0;
  static float actualVel=0.0,remainingTime=0.0;
  static long difTime, prevTime = 0;
  static SensorLastPressure_t PEEPPIMval_local;
  #if DEBUG_UPDATE
  static long difTimeIns, prevTimeIns = 0;
  #endif
//////////////////////////////////////////////////////////////////////////////////
// Lecturas de presion, y posicion del motor (tambien da idea del volumen)
  _sensors->readPressure();
  SensorPressureValues_t pressures = _sensors->getRelativePressureInCmH2O();
  _currentPressure = pressures.pressure1;
  actualPos = _stepper->getCurrentPositionInSteps();
// 25/6 Corroborar  
  if ((actualPos > 0) || actualPos < -780){
    digitalWrite(PIN_STEPPER_EN, HIGH); // Desactivo motor
  }
  else{
    digitalWrite(PIN_STEPPER_EN, LOW); // Activo motor
  }
// Fin Verificar

  actualVel = _stepper->getCurrentVelocityInStepsPerSecond();
    #if DEBUG_PLOT
    //Serial.print(millis());
    //Serial.print("\t");
//    #if ENABLED_SENSOR_VOLUME_byPRESSURE
//        _sensors->readVolume(); 
//    #endif
//    #if ENABLED_SENSOR_VOLUME_STEPPER
//        _sensors->readVolume(_stepper); 
//    #endif
    Serial.print(actualPos);
    //Serial.print(",");
   // Serial.print(_currentPressure);
 //   Serial.print(",");
    //Serial.print(actualVel);
 //   Serial.print(_sensors->getFlow());
//    Serial.print(","); 
//    Serial.print(currentTime);
    
    Serial.println("");
    #endif
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Verificaciones basicas
  if (pressures.state != SensorStateOK) {  
    // Sensor error detected: return to zero position and continue from there
    _sensor_error_detected = true;  // An error was detected in sensors
    /* Status update, for this time */
    #if DEBUG_UPDATE
        Serial.println("fail sensor");
    #endif
    setState(State_Homing);
  } else {
    _sensor_error_detected = false;  // clear flag
  }
  // Check pressures
  evaluatePressure();
  evaluate220V();
                       
  refreshWatchDogTimer();

// Fin Verificaciones basicas  
//////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////

/////////////     MAQUINA DE ESTADO PRINCIPAL VENTILATORIA   ////////////////////

/////////////////////////////////////////////////////////////////////////////////
  switch (_currentState) {
    case Init_Insufflation: {
/* Este estado da el compado de cerrar el ambu basado en el volumen tidal a
 * insuflar */
      flag_exceso=false;
      #if DEBUG_UPDATE
      Serial.println("Starting insuflation");
      #endif
      // Close Solenoid Valve
      digitalWrite(PIN_SOLENOID1, SOLENOID_CLOSED);
      digitalWrite(PIN_SOLENOID2, SOLENOID_CLOSED);
      // Reset volume
      PEEPPIMval_local = _sensors->getLastPressure();
      _sensors->resetVolumeIntegrator();
      _sensors->resetPressures();
      
      // Se trabaja con copia de los parametros para evitar cambios en los ciclos
      _timeoutInsCopy = _timeoutIns;
      _timeoutEspCopy = _timeoutEsp;
      _durac_exuCopy = _durac_exu;
      _volTidalStepsCopy = _volTidalSteps;
      // Fin copia de los parametros para evitar cambios en los ciclos
           
      totalCyclesInThisState = (_timeoutInsCopy) / TIME_BASE;

      
      /* Stepper control: set acceleration and end-position
       * Mover motor dependiendo de la duración de _timeoutIns y PIP, esto
       * determina la curva de movimento, seria velocidad y posición final
       * Depende del volumen Tital Vt calculado por formula del ambu. */

      goToPositionByDur(_stepper, STEPPER_DIR * _volTidalStepsCopy,STEPPER_LOWEST_POSITION, _timeoutInsCopy);
      
      /* Status update, reset timer, for next time */
      difTime = millis() - prevTime;
      prevTime = millis();
      _difTime = (uint16_t)(60000 / difTime);
      currentTime = 0;
      #if DEBUG_UPDATE
      prevTimeIns = millis();
      #endif
      _ciclos ++;
      setState(State_Insufflation);
    } break;

    /*
     * Insufflation
     */
    case State_Insufflation: {
      /* Este estado espera a que el ambu se termine de cerrar basado en el
      * tiempo _timeoutIns */
      
      // time expired
      if (currentTime > totalCyclesInThisState) {
        if (!_stepper->motionComplete()) {
          /// Alarma??????  <----------------------------
          // motor not finished, force motor to stop in current position
          _stepper->setTargetPositionInSteps(
              _stepper->getCurrentPositionInSteps());
        }
        currentTime = 0;
        totalCyclesInThisState = (HOLD_IN_DURATION) / TIME_BASE;   //<---- va HOLD_IN_DURATION no _tHoldIn
        setState(State_Hold_In);
        #if DEBUG_UPDATE
            Serial.println("Hold");
        #endif
        if (_recruitmentMode) {
          deactivateRecruitment();
        }
      } 
      if (currentTime > (totalCyclesInThisState/3)){
          if (digitalRead(PIN_STEPPER_ENDSTOP)==1) { // 0: con pulsador 1: con Efecto Hall
              /* Stepper homme - drift dorrection */
              #if DEBUG_UPDATE
                Serial.println("Salida por pasarse");
              #endif
              _stepper->setTargetPositionToStop();
              _stepper->setTargetPositionInSteps(_stepper->getCurrentPositionInSteps());
              //_stepper->setSpeedInStepsPerSecond(STEPPER_SPEED_MAX_STOP);
              _stepper->setAccelerationInStepsPerSecondPerSecond(STEPPER_ACC_MAX_STOP*3/2);
              _stepper->setTargetPositionInSteps(0);
              _startWasTriggeredByPatient = false;
              currentTime = 0;
              totalCyclesInThisState = (_timeoutEspCopy) / TIME_BASE;
              setState(State_Error_Pas);
              digitalWrite(PIN_STEPPER_EN, HIGH); // Desactivo motor
                #if DEBUG_UPDATE
                Serial.println("Error_Pas");
                #endif
         }
     }
     if (_currentPressure >= _pip){
          flag_exceso=true;
//          digitalWrite(PIN_SOLENOID1, SOLENOID_OPEN);
          
         
          #if DEBUG_UPDATE
            Serial.print(_pause);
            Serial.println("");
          #endif
          if(_pause){
            remainingTime =  totalCyclesInThisState - currentTime;
            totalCyclesInThisState = ((HOLD_IN_DURATION) / TIME_BASE) + remainingTime; 
             //_stepper->setSpeedInStepsPerSecond(STEPPER_SPEED_MAX_STOP);
             _stepper->setAccelerationInStepsPerSecondPerSecond(STEPPER_ACC_MAX_STOP);
          }
          else{
            totalCyclesInThisState = (HOLD_IN_DURATION) / TIME_BASE;   //<---- va HOLD_IN_DURATION no _tHoldIn
             //_stepper->setSpeedInStepsPerSecond(STEPPER_SPEED_MAX_STOP);
            _stepper->setAccelerationInStepsPerSecondPerSecond(STEPPER_ACC_MAX_STOP*3/2);
          }
          
          _stepper->setTargetPositionToStop();
          _stepper->setTargetPositionInSteps(_stepper->getCurrentPositionInSteps());
          currentTime = 0;
          setState(State_Hold_In);
          #if DEBUG_UPDATE
            Serial.println("HoldPIM");
          #endif        
          if (_recruitmentMode) {
          deactivateRecruitment();
          }
      }
     if (_recruitmentMode) {
          if (_currentPressure > DEFAULT_RECRUITMENT_PIP + 2) {
            digitalWrite(PIN_SOLENOID1, SOLENOID_OPEN);
            digitalWrite(PIN_SOLENOID2, SOLENOID_OPEN);
          } else if (_currentPressure < DEFAULT_RECRUITMENT_PIP - 0.5) {
            digitalWrite(PIN_SOLENOID1, SOLENOID_CLOSED);
            digitalWrite(PIN_SOLENOID2, SOLENOID_CLOSED);
          }
      }
        currentTime++;
    } break;

    case State_Hold_In: {
      /*Este estado espera a que se cumpla el tiempo de presion constante de
       * Plateau*/
       #if DEBUG_UPDATE
       difTimeIns = millis() - prevTimeIns;
       Serial.println(difTimeIns);
       #endif
       SensorPressureValues_t pressuresPlateau = _sensors->getRelativePressureInCmH2O();
       uint8_t PressurePlateau = pressures.pressure1;
       SumPlateau(PressurePlateau);
      if (digitalRead(PIN_STEPPER_ENDSTOP)==1) { // 0: con pulsador 1: con Efecto Hall
              /* Stepper homme - drift dorrection */
              #if DEBUG_UPDATE
                Serial.println("Salida por pasarse");
              #endif
              _stepper->setTargetPositionToStop();
              _stepper->setTargetPositionInSteps(_stepper->getCurrentPositionInSteps());
              //_stepper->setSpeedInStepsPerSecond(STEPPER_SPEED_MAX_STOP);
              _stepper->setAccelerationInStepsPerSecondPerSecond(STEPPER_ACC_MAX_STOP*3/2);
              _stepper->setTargetPositionInSteps(0);
              _startWasTriggeredByPatient = false;
              currentTime = 0;
              totalCyclesInThisState = (_timeoutEspCopy) / TIME_BASE;
              setState(State_Error_Pas);
              digitalWrite(PIN_STEPPER_EN, HIGH); // Desactivo motor
                #if DEBUG_UPDATE
                Serial.println("Error_Pas");
                #endif
         }
      if (currentTime > totalCyclesInThisState) {
        setState(Init_Exsufflation);
      }
      currentTime++;
    } break;

    case Init_Exsufflation: {
      /*Este estado abrir al todo el ambu (recordar valvula de retención), y deja que
      la valvular Peep regule La velocidad de exuflación la da el sistema neumatico,
      es importante conservar la curva de exsuflacion y peep plano para que funcione
      el modo de asistido
      */
      PromPlateau();
      #if DEBUG_UPDATE
            Serial.println("Starting exsuflation");
      #endif
     
      // Open Solenoid Valve
      digitalWrite(PIN_SOLENOID1, SOLENOID_OPEN);
      digitalWrite(PIN_SOLENOID2, SOLENOID_OPEN);

      totalCyclesInThisState = _timeoutEspCopy / TIME_BASE;
      _sensors->saveVolume();
      _sensors->resetVolumeIntegrator();

      /* Stepper control
          void goToPositionByDur(FlexyStepper *stepper, int goal_pos, int cur_pos, int dur);
      */
       
     goToPositionByDur(_stepper, STEPPER_LOWEST_POSITION, STEPPER_DIR * _volTidalStepsCopy, _durac_exuCopy);
      currentTime = 0;
      setState(State_Exsufflation);
    } break;

    /*
     * Exsufflation
     */
    case State_Exsufflation: {
      /*Este estado espera cumplir el tiempo de ciclo de exuflación o, si esta
      habilitado el modo de asistido, que el paciente intente respirar
      (detectado por presion), para inciar un nuevo ciclo de insuflación*/
      flag_exceso=false;
      if (_stepper->motionComplete()) {
        // Se abre al todo las paletas, y me quedo en la zona PEEP esperando que
        // pase el tiempo o haya un sub PEEP (quiso respirar)
        if (_currentPressure < (_triggerThreshold) && _hasTrigger) {  // The start was triggered by patient
          _startWasTriggeredByPatient = true;
          /* Status update, for next time */
          setState(Init_Insufflation);
          if(_running == false){
            setState(State_StandBy);
          }
        }
      }
      // Time has expired
      if (currentTime > totalCyclesInThisState) {
        if (!_stepper->motionComplete()) {
          // Alarma???
          // <-----------------------------------------------------------------------
          // motor not finished, force motor to stop in current position
          _stepper->setTargetPositionInSteps(
          _stepper->getCurrentPositionInSteps());
        }

        /* Status update and reset timer, for next time */
        _startWasTriggeredByPatient = false;
        currentTime = 0;
        setState(Init_Insufflation);
                if(_running == false){
        setState(State_StandBy);
        }
      } else {
        //_stepper->setSpeedInStepsPerSecond(abs(_stepperSpeed));
        
        currentTime++;
      }
      if (digitalRead(PIN_STEPPER_ENDSTOP)==1) { // 0: con pulsador 1: con Efecto Hall
          /* Stepper homme - drift dorrection */
          #if DEBUG_UPDATE
            //Serial.println("Salida por home");
          #endif
          _stepper->setTargetPositionToStop();
          _stepper->setTargetPositionInSteps(_stepper->getCurrentPositionInSteps());
          _stepper->setTargetPositionInSteps(0);
          _startWasTriggeredByPatient = false;
          //currentTime = 0;
          //setState(Init_Insufflation);
        }

    } break;
    
    case State_Error_Pas: {
       if (currentTime > totalCyclesInThisState) {
        /* Status update and reset timer, for next time */
        _startWasTriggeredByPatient = false;
        currentTime = 0;
        setState(Init_Insufflation);
        digitalWrite(PIN_STEPPER_EN, LOW);
      } else {
        currentTime++;
      }
    } break;
    
    case State_Homing: {
      // Open Solenoid Valve
      digitalWrite(PIN_SOLENOID1, SOLENOID_OPEN);
      digitalWrite(PIN_SOLENOID2, SOLENOID_OPEN);

      if (_sensor_error_detected) {
        // error sensor reading
        _running = false;
        #if DEBUG_UPDATE
        Serial.println("Sensor: FAILED");
        #endif
      }
      /* If not in home, do Homing.
       *  // 0: con pulsador 1: con Efecto Hall
       * Con pulsador
       * 0: stepper is in home
       * 1: stepper is not in home 
       * Con pulsador
       * 1: stepper is in home
       * 0: stepper is not in home */
      if (digitalRead(PIN_STEPPER_ENDSTOP) == 0) {
        /* Stepper control: homming */
        #if DEBUG_UPDATE
        Serial.println("Attempting homing...");
        #endif
        if (_stepper->moveToHomeInSteps(
                STEPPER_HOMING_DIRECTION, STEPPER_HOMING_SPEED,
                STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS,
                PIN_STEPPER_ENDSTOP) != true) {
#if DEBUG_UPDATE
          Serial.println("Homing failed");
#endif
        }
      }
      /* Status update and reset timer, for next time */
      currentTime = 0;
      _stepper->setTargetPositionInSteps(0);
      
      setState(State_StandBy);
      stopMech();
      //setState(Init_Insufflation);
      
    } break;
    
    case State_StandBy:{
      if(_running == false){
        setState(State_StandBy);
        digitalWrite(PIN_STEPPER_EN, HIGH); // Modo bajo consumo
      }
      if(_running == true){
        start();
        setState(Init_Insufflation);
        digitalWrite(PIN_STEPPER_EN, LOW);
      }
    }break;
    
    case State_Error:
      break;
    default:
      // TODO
      break;
  }
}

void MechVentilation::_init(FlexyStepper *stepper, Sensors *sensors, AutoPID *pid, VentilationOptions_t options) {
  /* Set configuration parameters */
  
   _stepper = stepper;
  _sensors = sensors;
  _pid = pid;
  _rpm = options.respiratoryRate;
  _pip = options.peakInspiratoryPressure;
  _peep = options.peakEspiratoryPressure;
  _ciclos = 0;
  setRPM(_rpm);
  //setVolTidalInStep(DEFAULT_PORC_VOLTILDAL);
  setVolTidalByArray(DEFAULT_BYARRAY_VOLTILDAL);
  setIer((uint8_t)(DEFAULT_INSPIRATORY_FRACTION*100));
  setTh(5);
  _setInspiratoryCycle();
  setPause(DEFAULT_PAUSE);
  _hasTrigger = options.hasTrigger;
  if (_hasTrigger) {
    _triggerThreshold = options.triggerThreshold;
  }
  //    else
  //    {
  //        _triggerThreshold = FLT_MAX;
  //    }

  /* Initialize internal state */
  _currentState = State_Homing;
  _stepperSpeed = STEPPER_SPEED_DEFAULT;

  //
  // connect and configure the stepper motor to its IO pins
  //
  //;
  _stepper->connectToPins(PIN_STEPPER_STEP, PIN_STEPPER_DIRECTION);
  _stepper->setStepsPerRevolution(STEPPER_STEPS_PER_REVOLUTION *
                                  STEPPER_MICROSTEPS);

  _sensor_error_detected = false;
}

void MechVentilation::setState(State state) { _currentState = state; }

//void MechVentilation::_setAlarm(Alarm alarm) { _currentAlarm = alarm; }

int MechVentilation::getAlarms(void){ return (_currentAlarm); }

void MechVentilation::setAlarms(int alarm_value){
  
  _currentAlarm=_currentAlarm | alarm_value;//operacion or entre alarmas
  
  }

void MechVentilation::resetAlarms(int alarm_value){
  
  _currentAlarm=_currentAlarm & alarm_value;//operacion and entre alarmas
  
  }





  /**
 * @brief Función de seteo de la variable _tHoldIn
 * @param value: valor a setear
 * @return ninguno
 * @details permite cambiar el valor de la variable _tHoldIn y recalcular los demas valores de tiempos
 */
void  MechVentilation::setInsuflationTime(uint8_t value){
  
      float divisor=0,aux=0,tInspMin=0,tInspMax=0;    
      float localrpm = _rpm;
      float timeoutCycle =(60000.0 / (localrpm));  // T     Tiempo de ciclo en msegundos
            
      tInspMin=timeoutCycle / 5; //Tiempo inspiratorio Mínimo
      tInspMax=timeoutCycle / 3; //Tiempo inspiratorio Mínimo
      
      //_timeoutIns=value;
      _tHoldIn=value*10;



      if(_tHoldIn>tInspMax){
          
          _tHoldIn=tInspMax;
      }
      
      if(_tHoldIn<tInspMin){
        
          _tHoldIn=tInspMin;
      }
      
      _timeoutIns = _tHoldIn ;
      _timeoutEsp = timeoutCycle - _tHoldIn - HOLD_IN_DURATION;
      //_durac_exu = min(_tHoldIn + MAX_EX_DURATION, timeoutCycle - MIN_PEEP_PAUSE);
      _durac_exu = _tHoldIn;
           
}

  /**
 * @brief Función que permite ver el valor de la variable _timeoutIns para sensor de presion MPX.
 * @param ninguno
 * @return valor de la variable _timeoutIns
 * @details permite leer el valor de la variable _timeoutIns
 */

float  MechVentilation::getInsuflationTimeApp(void){
  
  return(_tHoldIn);
}












  

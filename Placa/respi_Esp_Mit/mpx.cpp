/** 
 *
 * @file mpx.cpp
 * @details Este archivo contiene las definiciones de las funciones utilizadas para leer los sensores MPX 50XX  
 * así como tambien las funciones necesarias para convertir los valores leidos a las diversas unidades requeridas.
 *
 */
#include "mpx.h"
#include "calc.h"

 
float  MPX_PRESSURE_SENSOR::autoajuste (float offset){
  const unsigned char veces=10;
  static char signo=1;
  float medidas=0, pres1=0.0,pres2=0.0,prom=0,diff=0;

    for (int i=0;i<veces+1;i++){ // leo tantas veces el sensor
      
        pres1 =  MPX_READ_PRESSURE();          //Volts
        pres2 =  MPX_CONVERT_PRESSURE(pres1);  //kPa
        pres2 =  MPX_CONVERT_cmH20(pres2);     //cmH2O
        medidas= medidas+pres2;                //voy sumando medidas
    }
    
//saco el promedio
prom=medidas/veces+1;


  //si el valor es positivo el offset debe aumentar
  //si el valor es negativo el offset debe disminuir 
  if(prom >0){
    signo=1;
  }else{
    signo=-1;
    }

diff=abs(0-prom);

//Serial.print("Promedio: ");
//Serial.print(prom);
//Serial.print("  Diferencia: ");
//Serial.print(diff);
//Serial.print("   ");

 //si la diferencia es cero, entonces retorno sin cambios.
 if(diff!=0){

    if(diff>3){
      offset=offset+(signo*50);
    }else{
            //si la diferencia es menor que 1 , ajusto de a 5, sinó ajusto de a 10
          if(diff<1){
            offset=offset+(signo*1); //5
          }else{
                offset=offset+(signo*20);  //15 recuperable
          }
          
    }



 }
  return(offset);
}


void  MPX_PRESSURE_SENSOR::setOffsetValue(float value){
      
      _offset=value;
}


float  MPX_PRESSURE_SENSOR::getOffsetValue(void){
  
//  Serial.print("el offset seteado es: ");
//  Serial.println(_offset);
  return(_offset);
}

 MPX_PRESSURE_SENSOR::MPX_PRESSURE_SENSOR(uint8_t pin){ 

	max_pressure = 5;	//kPa
	min_pressure = 0;	//kPa
	pressure = 0;		//kPa
	_offset = DEFAULT_OFFSET_MPX;		//mVolts
  _pin = pin;//PIN_MPX_DATA;
}


void MPX_PRESSURE_SENSOR::MPX_SET_ADC(void){

	#if ENABLED_SENSOR_MPX5050
		analogReference(INTERNAL1V1);
	#else
		analogReference(INTERNAL2V56);
  #endif
  _offset = DEFAULT_OFFSET_MPX;
//  Serial.print(offset);
//  Serial.println("offsetadc");
}



float MPX_PRESSURE_SENSOR::MPX_READ_PRESSURE(void){						//Si está en 1 es el MPX5050
  float  Vfinal;
  static unsigned int bufV[]={0,0,0,0,0,0,0,0,0}, Vin;
  static unsigned char indice=0;
  bufV[indice] = analogRead(_pin);
  
  //Vin = (bufV[0] + bufV[1] + bufV[2]+ bufV[3] + bufV[4] + bufV[5]+ bufV[6] + bufV[7]+ bufV[6] ) / 9;
  Vin = QuickSelectMedian(bufV, 9);
  indice++;
  if(indice==9) indice = 0;
  //Vin = analogRead(_pin);
  #if ENABLED_SENSOR_MPX5050
  Vfinal = map(Vin, 0, 1023, 0, 1100);
  #else
  Vfinal = map(Vin, 0, 1023, 0, 2560);
  #endif
  Vfinal = (Vfinal - _offset)/1000.0; //1000.0
//  Serial.print(Vfinal);
//  Serial.println("");
  return(Vfinal);
}



float MPX_PRESSURE_SENSOR::MPX_CONVERT_PRESSURE(float Vfinal){
	float pressurekPa;
  #if ENABLED_SENSOR_MPX5050
		pressurekPa = Vfinal/0.09;
	#else
		pressurekPa = Vfinal/0.45;       //               <--------------- Vuelta atras 9/11/2020 3era revision estaba 0.45 paso a 0.468
  #endif
	return(pressurekPa);
}


float MPX_PRESSURE_SENSOR::MPX_CONVERT_cmH20(float pressureKPA){
  float pressureValue_cmH20;
	pressureValue_cmH20 = (pressureKPA)* 10.197162129779282;			// 1 pascal = 0.010197162129779282 cmH2O
	return(pressureValue_cmH20);
}

#if ENABLED_SENSOR_VOLUME_byPRESSURE
float MPX_PRESSURE_SENSOR::MPX_CONVERT_flujo(){
        float pres1, pres2,flow;
        float  Vfinal;
        static unsigned int bufV[]={0,0,0,0,0,0,0,0,0}, Vin;
        static unsigned char indice=0;
        bufV[indice] = analogRead(_pin);
        Vin = QuickSelectMedian(bufV, 9);
        indice++;
        if(indice==9) indice = 0;
        #if ENABLED_SENSOR_MPX5050
        Vfinal = map(Vin, 0, 1023, 0, 1100);
        #else
        Vfinal = map(Vin, 0, 1023, 0, 2560);
        #endif
        pres1 = (Vfinal - DEFAULT_OFFSET_MPX2); //1000.0        
        pres1=pres1 * byPRESSURE_KDELTA;
        if(pres1>0){
          flow = sqrt (pres1);
        }
        else
        {
          pres1=-pres1 ;
          flow = -sqrt ( pres1);
        }
        return(flow);
        //return(pres1);
}
#endif

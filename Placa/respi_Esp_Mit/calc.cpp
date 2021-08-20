/** Funciones de cálculo necesarias
 *
 * @file calc.cpp
 * @details Este archivo contiene las definiciones de las funciones de cálculo necesarias
 *
 */
#include "calc.h"


int estimateTidalVolume(int estatura, int sexo) {
  float peso0, pesoIdeal, volumenEstimado;
  if (sexo == 0) { // Varón
    peso0 = 50.0;
  } else if (sexo == 1) { // Mujer
    peso0 = 45.5;
  }
  pesoIdeal = peso0 + 0.91 * (estatura - 152.4); // en kg

  return ((int)(round(pesoIdeal * DEFAULT_ML_PER_KG_IDEAL_WEIGHT)));
}



unsigned int QuickSelectMedian(unsigned int arr[], uint8_t n)
{
  #define ELEM_SWAP(a,b) { unsigned int t=(a);(a)=(b);(b)=t; }
   uint16_t low, high;
   uint16_t median;
   uint16_t middle, ll, hh;
   low = 0; high = n - 1; median = (low + high) / 2;
   for (;;)
   {
      if (high <= low)
         return arr[median];
      if (high == low + 1)
      {
         if (arr[low] > arr[high])
            ELEM_SWAP(arr[low], arr[high]);
         return arr[median];
      }
 
      middle = (low + high) / 2;
      if (arr[middle] > arr[high])
         ELEM_SWAP(arr[middle], arr[high]);
      if (arr[low] > arr[high])
         ELEM_SWAP(arr[low], arr[high]);
      if (arr[middle] > arr[low])
         ELEM_SWAP(arr[middle], arr[low]);
 
      ELEM_SWAP(arr[middle], arr[low + 1]);
 
      ll = low + 1;
      hh = high;
      for (;;)
      {
         do ll++; while (arr[low] > arr[ll]);
         do hh--; while (arr[hh] > arr[low]);
         if (hh < ll)
            break;
         ELEM_SWAP(arr[ll], arr[hh]);
      }
      ELEM_SWAP(arr[low], arr[hh]);
 
      if (hh <= median)
         low = ll;
      if (hh >= median)
         high = hh - 1;
   }
   return arr[median];
}


void refreshWatchDogTimer() {
  //TODO implementar
  wdt_reset();
}

#if 0

float computeLPF(int parameter, int lpfArray[])
{
  int samples = sizeof(lpfArray);
  int k = samples - 1;           // tamano de la matriz
  int cumParameter = parameter; // el acumulador suma ya el param
  lpfArray[0] = parameter;
  while (k > 0)
  {
    lpfArray[k] = lpfArray[k - 1];   // desplaza el valor una posicion
    cumParameter += lpfArray[k];     // acumula valor para calcular la media
    k--;
  }
  float result = cumParameter / samples;
  return result;
}
#endif

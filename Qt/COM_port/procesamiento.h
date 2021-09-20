#ifndef PROCESAMIENTO_H
#define PROCESAMIENTO_H

#include "math.h"
using namespace std;

#define GIROSCOPO       1
#define ACELEROMETRO    2

#define EJE_X           1
#define EJE_Y           2
#define EJE_Z           3

class Cdato{
private:
    int aceX;
    int aceY;
    int aceZ;

    int girX;
    int girY;
    int girZ;

    double modGir;
    double modAce;

    int timeStamp;


public:
    Cdato();
    ~Cdato(){};

    //Metodos de seteo
    void set(int aceVal, char dev = 0, char eje = 0);

    //Metodos de obtencion de valores de atributos
    int get(char dev = 0, char eje = 0) const;

    //Calculo de modulo
    void calculoMod(char dev = 0);

    //Obtengo modulo
    double getMod(char dev = 0) const;
};



#endif // PROCESAMIENTO_H

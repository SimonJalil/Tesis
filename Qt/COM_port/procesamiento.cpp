#include "procesamiento.h"

Cdato::Cdato(){
    aceX = 0;
    aceY = 0;
    aceZ = 0;

    girX = 0;
    girY = 0;
    girZ = 0;

    modAce = 0;
    modGir = 0;

    timeStamp = 0;
}

void Cdato::set(int aceVal, char dev, char eje){
    switch(eje){
        case EJE_X:
            if(dev == 1)
               girX = aceVal;
            else if(dev == 2)
               aceX = aceVal;
        break;

        case EJE_Y:
            if(dev == 1)
                girY = aceVal;
            else if(dev == 2)
                aceY = aceVal;
        break;

        case EJE_Z:
            if(dev == 1)
                girZ = aceVal;
            else if(dev == 2)
                aceZ = aceVal;
        break;

    default:
        break;
    }
}

int Cdato::get(char dev, char eje) const{
    switch(eje){
        case EJE_X:
            if(dev == 1)
               return girX;
            else if(dev == 2)
               return aceX;
        break;

        case EJE_Y:
            if(dev == 1)
               return girY;
            else if(dev == 2)
               return aceY;
        break;

        case EJE_Z:
            if(dev == 1)
               return girZ;
            if(dev == 2)
               return aceZ;
        break;

    default:
        break;
    }

    return 0;
}



void Cdato::calculoMod(char dev){
    if(dev == 1)
        modGir = sqrt( pow(girX,2) + pow(girY,2) + pow(girZ,2) );
    else if(dev == 2)
        modAce = sqrt( pow(aceX,2) + pow(aceY,2) + pow(aceZ,2) );
}

double Cdato::getMod(char dev) const{
    if(dev == 1)
        return modGir;
    else if(dev == 2)
        return modAce;
    return 0;
}

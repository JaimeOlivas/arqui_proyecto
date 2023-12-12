#ifndef DRIVER_GPIOCONFIG_JRL_H
#define DRIVER_GPIOCONFIG_JRL_H

//Definiciones de palabras claves
  /*modos de los pines*/
#define OUTPUT 1
#define INPUT  0
#define INPUT_PULLUP    2
#define INPUT_PULLDOWN  3
    /*niveles para la salida*/
#define HIGH 1
#define LOW  0   

enum GPIOS 
{
    PIN0,  PIN1,  PIN2,  PIN3,  PIN4,  PIN5,  PIN6,  PIN7,  PIN8,  PIN9,
    PIN10, PIN11, PIN12, PIN13, PIN14, PIN15, PIN16, PIN17, PIN18, PIN19,
    PIN20, PIN21, PIN22, PIN23, PIN24, PIN25, PIN26, PIN27, PIN28, PIN29,
    PIN30, PIN31, PIN32, PIN33, PIN34, PIN35, PIN36, PIN37, PIN38, PIN39
};

#endif
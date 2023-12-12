#include "gpio.h"

/****************************************************************************************
 * Nombre: gpio_pinMode
 * Precondiciones: --
 * Función: Establecer un pin como entrada o salida
 * parametros: pin, modo(entrada, salida, entrada con pull up, entrada con pull down)
**************************************************************************************/
void gpio_pinMode(uint8_t pin, uint8_t mode){


    uint32_t pin_mask = (1 << pin);
        // Acceder al registro GPIO_ENABLE_W1TS_REG para configurar el pin como salida  
   // volatile uint32_t * const GPIO_EN = (volatile uint32_t * const)GPIO_EN_ADDR;
    //volatile uint32_t * const DownUp_REG = (volatile uint32_t * const)0X3FF49074;
  

    if(mode == 1 && pin <= 34){  //si el registro esta en 1 será salida y menor que 34 ya que 34-39 solo son entradas
        if (pin <= 31)
            GPIO_EN_ADDR |= pin_mask;
        else  
           GPIO_EN1_ADDR |= pin_mask; 
        if(IO_MUX_OFS[pin] != 0x00){
           volatile uint32_t * const OE_REG = (volatile uint32_t * const)(IO_MUX_BASE_ADDR  + IO_MUX_OFS[pin] ); 
           *OE_REG |= 0X01;     //HABILITA EL BIT DE OUTPUT ENABLE
           *OE_REG &= ~(1<<9);  // SE DESHABILITA EL BIT DE INPUT ENABLE

          *OE_REG |= (1<<13); //SE SELECCIONA LA FUNCIÓN 2 DEL MUX IO

        }
    } else if ((mode == 0 || mode == 2 || mode == 3 ) && pin <=34 ){  // si es 0, 2 o 3 será entrada de los pines 0 - 34
        if (pin <= 31)                                                //si el pin esta entre 34 y 39 no entrara ya wue siempre son entradas
            GPIO_EN_ADDR &= ~pin_mask;                                // y no cuentan con resistencias pull up o pull down
        else 
           GPIO_EN1_ADDR &= ~pin_mask; 

        if(IO_MUX_OFS[pin] != 0x00){
           volatile uint32_t * const OE_REG = (volatile uint32_t * const)(IO_MUX_BASE_ADDR  + IO_MUX_OFS[pin] ); 
           *OE_REG &= ~0X01;    // SE DESHABILITA EL BIT DE INPUT ENABLE
           *OE_REG |= (1<<9);  //HABILITA EL BIT DE OUTPUT ENABLE

        }
       if(IO_MUX_OFS[pin] != 0x00){                                                                                     // hay algunos pines que no cuentan con resistencias de pull up a los cuales en el offset se les puso 0x00
            volatile uint32_t * const DownUp_REG = (volatile uint32_t * const)(IO_MUX_BASE_ADDR  + IO_MUX_OFS[pin] );   
            if(mode == 2){  //pullup
                *DownUp_REG |= (1 << 8); 
            }else if (mode == 3){ // pull down
                *DownUp_REG |= (1 << 7);
            }

                *DownUp_REG |= (1<<13);
       }

       GPIO_FUN_OUT_SEL(pin) |= 0x100;
    } 
    
}
/****************************************************************************************
 * Nombre: gpio_write
 * Precondiciones: Haber estableciodo el pin como salida con la función gpio_pinMode
 * Función: Escribir un uno o un cero en el pin 
 * parametros: pin, nivel al que se va poner el pin (HIGH o LOW)
**************************************************************************************/
void gpio_write(uint8_t pin, uint8_t level){
 
      
    // Acceder al registro GPIO_OUT_REG para poner el pin en estado lógico alto
    if(pin <= 31){
        uint32_t pin_mask = (1 << pin);  
        if(level)
            GPIO_OUT_REG |= pin_mask;
        else 
            GPIO_OUT_REG &= ~pin_mask;
    }else {
        uint32_t pin_mask = (1 << (pin-32)); 
        if(level)
            GPIO_OUT1_REG |= pin_mask;
        else 
            GPIO_OUT1_REG &= ~pin_mask;
    }

}
/****************************************************************************************
 * Nombre: gpio_read
 * Precondiciones: Haber estableciodo el pin como entrada con la función gpio_pinMode
 * Función: Leer el nivel en el que se encuentra el pin 
 * parametros: pin a leer
**************************************************************************************/
uint8_t gpio_read(uint8_t pin){

    if(pin > 31) 
        pin -= 32;
        
    if(pin < 32)
        return ((GPIO_IN_ADDR & (1 << pin))? 1: 0);
    else 
        return ((GPIO_IN1_ADDR & (1 << pin))? 1: 0); 
        
}

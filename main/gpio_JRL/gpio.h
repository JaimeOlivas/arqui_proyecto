#pragma once


#include "gpio_JRL/gpioConfig.h"

#include <stdio.h>
/***********************DECLARACIONES PARA ACCESO A MEMORIA***********************/
#define HWREG(x) (*((volatile uint32_t *)(x)))

// DIRECCIONES BASE PARA GPIO
#define GPIO_BASE_ADDR      (0x3FF44000 )
#define IO_MUX_BASE_ADDR    (0x3FF49000 )// Dirección base de los registros IO_MUX_REG
 //será la direccion del registro necesario para activar pull up o pulldowm el pin x

//DIRECCIONES EXACTAS DE GPIOS
#define GPIO_EN_ADDR            HWREG(GPIO_BASE_ADDR + 0X20)  //0-31
#define GPIO_EN1_ADDR           HWREG(GPIO_BASE_ADDR + 0X2C)  //32-39
#define GPIO_OUT_REG            HWREG(GPIO_BASE_ADDR + 0X04)
#define GPIO_OUT_WS_REG         HWREG(GPIO_BASE_ADDR + 0X08)
#define GPIO_OUT_WC_REG         HWREG(GPIO_BASE_ADDR + 0X0C)
#define GPIO_OUT1_REG           HWREG(GPIO_BASE_ADDR + 0X10)
#define GPIO_OUT1_WS_REG        HWREG(GPIO_BASE_ADDR + 0X14)
#define GPIO_OUT1_WC_REG        HWREG(GPIO_BASE_ADDR + 0X18)
#define GPIO_FUN_OUT_SEL(J)     HWREG(GPIO_BASE_ADDR+(0x530 + (0x4*J)))  //selector del mux gpio
#define GPIO_IN_ADDR            HWREG(GPIO_BASE_ADDR + 0x3C)  // Dirección de GPIO_IN_REG
#define GPIO_IN1_ADDR           HWREG(GPIO_BASE_ADDR + 0x40)  // Dirección de GPIO_IN1_REG

//OFSETS PARA DIRECCIONES GPIO

//OFFSETS PARA IO_MUX 
/*los lugares que tienen cero corresponden a pines que no cuentan con resistencias para 
*  pull up o pull down */
uint16_t IO_MUX_OFS[40] = {
    0x44, 0x88, 0x40, 0x84, 0x48, 0x6C, 0x60, 0x64, 0x68, 0x54, 
    0x58, 0x5C, 0x34, 0x38, 0x30, 0x3C, 0x4C, 0x50, 0x70, 0x74,
    0x78, 0x7C, 0x80, 0x8C, 0x90, 0x24, 0x28, 0x2C, 0x00, 0x00,
    0x00, 0x00, 0x1C, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
 
// 0x14, 0x18, 0x04, 0x08, 0x0C, 0x10 
/*********************************************************************************/

/***************************DECLARACIONES DE FUNCIONES GPIO***********************/


extern void gpio_pinMode(uint8_t pin, uint8_t mode);
extern void gpio_write(uint8_t pin, uint8_t level);
extern uint8_t gpio_read(uint8_t pin);

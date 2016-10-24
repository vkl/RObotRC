/**
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HWDRV_H
#define __HWDRV_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//#define GPIO_sensor GPIOB
//#define QTRSENSOR1 	GPIO_Pin_10
//#define QTRSENSOR2 	GPIO_Pin_11
//#define READSENSOR1 GPIO_sensor->IDR & QTRSENSOR1
//#define READSENSOR2 GPIO_sensor->IDR & QTRSENSOR2

//#define MOTORx		GPIOB
//#define MOTORA1 	GPIO_Pin_6
//#define MOTORA2 	GPIO_Pin_7
//#define MOTORB1 	GPIO_Pin_8
//#define MOTORB2 	GPIO_Pin_9

#define ROBOTADDR   98D3,32,30511C,10

//#define LEFT 		(MOTORx->BSRR = MOTORA1 | MOTORB2; MOTORx->BRR = MOTORA2 | MOTORB1)
//#define RIGHT 		(MOTORx->BSRR = MOTORA2 | MOTORB1; MOTORx->BRR = MOTORA1 | MOTORB2)

#endif

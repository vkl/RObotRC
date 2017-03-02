/**
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HWDRV_H
#define __HWDRV_H

/* Includes ------------------------------------------------------------------*/

#define ROBOTADDR   98D3,32,30511C,10

/* Definition for new status register */
#define Status_STOP         0x00 // 0b00000000
#define Status_FORWARD      0x01 // 0b00000001 
#define Status_BACK         0x02 // 0b00000010    
#define Status_LEFT         0x03 // 0b00000011   
#define Status_RIGHT        0x04 // 0b00000100     
#define Status_SLEFT        0x05 // 0b00000101     
#define Status_SRIGHT       0x06 // 0b00000110   
#define Status_SBLEFT       0x07 // 0b00000111     
#define Status_SBRIGHT      0x08 // 0b00001000 
#define Status_AUTO         0x40 // 0b01000000
#define Status_OK           0x80 // 0b10000000 

#endif

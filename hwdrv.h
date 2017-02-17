/**
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HWDRV_H
#define __HWDRV_H

/* Includes ------------------------------------------------------------------*/

#define ROBOTADDR   98D3,32,30511C,10

#define AUTO      ((0x51 << 8) | 0x52) /* the command is QR */
#define FORWARD   ((0x46 << 8) | 0x52) /* the command is FR */
#define BACK      ((0x42 << 8) | 0x4B) /* the command is BK */
#define LEFT      ((0x4C << 8) | 0x46) /* the command is LF */
#define RIGHT     ((0x52 << 8) | 0x54) /* the command is RT */
#define STOP      ((0x53 << 8) | 0x54) /* the command is ST */
#define OK        ((0x4F << 8) | 0x4B) /* the command is OK */

/* Bit definition for status register */
#define FORWARD_Status     0
#define BACK_Status        1
#define LEFT_Status        2
#define RIGHT_Status       3
#define MOVE_Status        4
#define AUTO_Status        6
#define CONNOK             7

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
#define Status_OK           0x80 // 0b10000000 
#define Status_AUTO         0x40 // 0b01000000

#endif

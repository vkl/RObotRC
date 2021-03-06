#ifndef __DISPLAY_H
#define __DISPLAY_H

#include "st7735.h"

#define LINE(POS, STR) (ST7735_PutStr5x7(30, POS, (STR), ST7735_Color565(0, 0xFF, 0), ST7735_Color565(0x0, 0x0, 0x0), 1))
#define SEL(POS) (ST7735_PutChar5x7(20, POS, 0x80, ST7735_Color565(0, 0xFF, 0), ST7735_Color565(0x0, 0x0, 0x0), 1))
#define UNSEL(POS) (ST7735_PutChar5x7(20, POS, ' ', ST7735_Color565(0, 0xFF, 0), ST7735_Color565(0x0, 0x0, 0x0), 1))

#define LINE0(STR) (ST7735_PutStr5x7(5, 5, (STR), ST7735_Color565(0, 0xFF, 0), ST7735_Color565(0x0, 0x0, 0x0), 1))
#define LINE1(STR) (ST7735_PutStr5x7(5, 15, (STR), ST7735_Color565(0, 0xFF, 0), ST7735_Color565(0x0, 0x0, 0x0), 1))
#define LINE2(STR) (ST7735_PutStr5x7(5, 25, (STR), ST7735_Color565(0, 0xFF, 0), ST7735_Color565(0x0, 0x0, 0x0), 1))
#define LINE3(STR) (ST7735_PutStr5x7(5, 35, (STR), ST7735_Color565(0, 0xFF, 0), ST7735_Color565(0x0, 0x0, 0x0), 1))
#define LINE4(STR) (ST7735_PutStr5x7(5, 45, (STR), ST7735_Color565(0, 0xFF, 0), ST7735_Color565(0x0, 0x0, 0x0), 1))
#define LINE5(STR) (ST7735_PutStr5x7(5, 55, (STR), ST7735_Color565(0, 0xFF, 0), ST7735_Color565(0x0, 0x0, 0x0), 1))
#define LINE6(STR) (ST7735_PutStr5x7(5, 65, (STR), ST7735_Color565(0, 0xFF, 0), ST7735_Color565(0x0, 0x0, 0x0), 1))
#define LINE7(STR) (ST7735_PutStr5x7(5, 75, (STR), ST7735_Color565(0, 0xFF, 0), ST7735_Color565(0x0, 0x0, 0x0), 1))
#define LINE8(STR) (ST7735_PutStr5x7(5, 85, (STR), ST7735_Color565(0, 0xFF, 0), ST7735_Color565(0x0, 0x0, 0x0), 1))

#define COL1LINE0(STR)  (ST7735_PutStr5x7(80, 5, (STR), ST7735_Color565(0, 0xFF, 0), ST7735_Color565(0x0, 0x0, 0x0), 1))
#define COL2LINE0(STR)  (ST7735_PutStr5x7(115, 5, (STR), ST7735_Color565(0, 0xFF, 0), ST7735_Color565(0x0, 0x0, 0x0), 1))

#define TOTALITEMS 3

typedef enum
{
    FALSE = 0,
    TRUE = 1
} BOOL_TypeDef;

typedef enum
{
    SW_NONE = 1,
    SW_UP,
    SW_DOWN,
    SW_LEFT,
    SW_RIGHT,
    SW_OK    
} KEY_TypeDef;

typedef enum
{
    DISPLAY_unknown = 1,
    DISPLAY_menu,
    DISPLAY_status,
    DISPLAY_btmodule
} DISPLAY_Typedef;

typedef struct
{
    uint8_t prevStatus;
    uint8_t currentStatus;
    uint8_t responseStatus;
    KEY_TypeDef prevKey;
    KEY_TypeDef currentKey;
    BOOL_TypeDef keyPressed;
    DISPLAY_Typedef prevPage;
    DISPLAY_Typedef currentPage;
} DATA_TypeDef;

/* Menu */
typedef struct
{
    int8_t prevItem;
    int8_t currentItem;
    int8_t nextItem;
} MENU_TypeDef;

void MenuPage(DATA_TypeDef*);
void StatusPage(DATA_TypeDef*);
void ShowPage(DATA_TypeDef*);
void InitMenu(void);

#endif

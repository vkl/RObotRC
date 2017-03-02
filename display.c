

#include "display.h"
#include "hwdrv.h"

MENU_TypeDef menu;

typedef enum
{
    STATUS_MenuIndex = 0,
    BT_MODULE_MenuIndex,
    SYSTEM_MenuIndex
} MenuIndex_TypeDef;

char *items[] = {"STATUS    ", "BT MODULE ", "SYSTEM    "};

void InitMenu(void)
{
    menu.prevItem = 0;
    menu.currentItem = 0;
    menu.nextItem = 1;
}


void ShowPage(DATA_TypeDef *data)
{
    if (data->prevPage != data->currentPage)
    {
        ST7735_Clear(ST7735_Color565(0x0, 0x0, 0x0));
        data->prevPage = data->currentPage;
    }        
    switch (data->currentPage)
    {
        case DISPLAY_menu:
            MenuPage(data);
            break;
        case DISPLAY_status:
            StatusPage(data);
            break;
        default:
            break;
    }
}

void MenuPage(DATA_TypeDef *data)
{
    uint8_t i;
    
    if (data->keyPressed == TRUE)
    {
        switch (data->currentKey)
        {
            case SW_DOWN:
                menu.currentItem++;
                if (menu.currentItem >= TOTALITEMS) menu.currentItem = 0;
                break;
            case SW_LEFT:
                break;
            case SW_RIGHT:
                break;
            case SW_UP:
                menu.currentItem--;
                if (menu.currentItem < 0) menu.currentItem = (TOTALITEMS - 1);
                break;
            case SW_OK:
                switch (menu.currentItem)
                {
                    case STATUS_MenuIndex:
                        data->prevStatus = 0xFF;
                        data->currentPage = DISPLAY_status;
                        break;
                    default:
                        break;
                }
                break;
            case SW_NONE:
                break;
            default:
                break;
        }

        for (i=0; i<TOTALITEMS; i++)
        {
            if (menu.currentItem == i)
            {
                SEL((i*12) + 20);
                LINE((i*12) + 20, items[i]);
            }
            else
            {
                UNSEL((i*12) + 20);
                LINE((i*12) + 20, items[i]);
            }
        }

        data->currentKey = SW_NONE;
        
    }
}

void StatusPage(DATA_TypeDef *data)
{
    
    if (data->keyPressed == TRUE)
    {
        if (data->currentKey != SW_NONE) data->currentPage = DISPLAY_menu;
    }
    
    if (data->prevStatus != data->responseStatus)
    {
        data->prevStatus = data->responseStatus;
        if (data->responseStatus == 0)
        {
            ST7735_Clear(ST7735_Color565(0x0, 0x0, 0x0));
            LINE0("DISCONNECTED");
        }
        else
        {
            LINE0("CONNECTED   ");
            if (data->responseStatus & Status_AUTO) LINE1("Status: Auto ON "); else LINE1("Status: Auto OFF");
            switch (data->responseStatus & 0x0F)
            {
                case Status_FORWARD:
                    LINE2("Command: Forward        ");
                    break;
                case Status_BACK:
                    LINE2("Command: Back           ");
                    break;
                case Status_LEFT:
                    LINE2("Command: Left           ");
                    break;
                case Status_RIGHT:
                    LINE2("Command: Right          ");
                    break;
                case Status_SLEFT:
                    LINE2("Command: Forward + Left ");
                    break;
                case Status_SRIGHT:
                    LINE2("Command: Forward + Right");
                    break;
                case Status_SBLEFT:
                    LINE2("Command: Back + Left    ");
                    break;
                case Status_SBRIGHT:
                    LINE2("Command: Back + Right   ");
                    break;
                default:
                    LINE2("                        ");
                    break;
            }
        }
        LINE8(" Press any key for menu");
    }
    
    data->currentKey = SW_NONE;
}

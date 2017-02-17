

#include "display.h"
#include "hwdrv.h"

void ShowPage(DATA_TypeDef *data)
{
    if (data->prevKey != data->currentKey)
    {
        data->prevKey = data->currentKey;
        data->keyPressed = TRUE;
    }
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
    if (data->keyPressed == TRUE)
    {
        data->keyPressed = FALSE;
        switch (data->currentKey)
        {
            case SW_DOWN:
                LINE3("The key down ");
                break;
            case SW_LEFT:
                LINE3("The key left ");
                break;
            case SW_RIGHT:
                LINE3("The key right");
                break;
            case SW_UP:
                LINE3("The key up   ");
                break;
            case SW_OK:
                LINE3("The key ok   ");
                break;
            default:
                break;
        }
    }
    LINE0(" .");
    LINE1(" Status");
    LINE2(" BT Module");
}

void StatusPage(DATA_TypeDef *data)
{
    
    if (data->keyPressed == TRUE)
    {
        data->currentPage = DISPLAY_menu;
        data->keyPressed = FALSE;
    }
    
    if (data->prevStatus != data->currentStatus)
    {
        data->prevStatus = data->currentStatus;
        if (data->currentStatus == 0)
        {
            ST7735_Clear(ST7735_Color565(0x0, 0x0, 0x0));
            LINE0("Disconnected");
        }
        else
        {
            LINE0("Connected   ");
            if (data->currentStatus & Status_AUTO) LINE1("Auto ON "); else LINE1("Auto OFF");
            switch (data->currentStatus & 0x0F)
            {
                case Status_FORWARD:
                    LINE2("Direction: Forward");
                    break;
                case Status_BACK:
                    LINE2("Direction: Back   ");
                    break;
                case Status_LEFT:
                    LINE2("Direction: Left   ");
                    break;
                case Status_RIGHT:
                    LINE2("Direction: Right  ");
                    break;
                default:
                    LINE2("                  ");
                    break;
            }
        }
        LINE8(" Press any key for menu");
    }
    
}

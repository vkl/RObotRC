#include "stm32f10x.h"
#include "delay.h"
#include "usart_rxtx.h"
#include "hwdrv.h"
#include "st7735.h"

#define NUM 10
#define FAILURES 3

#define AUTO      ((0x51 << 8) | 0x52) /* the command is QR */
#define FORWARD      ((0x46 << 8) | 0x52) /* the command is FR */
#define BACK       ((0x42 << 8) | 0x4B) /* the command is BK */
#define LEFT      ((0x4C << 8) | 0x46) /* the command is LF */
#define RIGHT       ((0x52 << 8) | 0x54) /* the command is RT */
#define STOP       ((0x53 << 8) | 0x54) /* the command is ST */
#define OK          ((0x4F << 8) | 0x4B) /* the command is OK */

/* Bit definition for status register */
#define FORWARD_Status    0
#define BACK_Status        1
#define LEFT_Status        2
#define RIGHT_Status    3
#define MOVE_Status        4
#define AUTO_Status        6
#define CONNOK          7

#define TEN(NUMBER) ((3 << 4) | NUMBER / 10)
#define ONE(NUMBER) ((3 << 4) | NUMBER % 10)

#define MAXVAL  4096
#define MINVAL  0
#define MIDVAL  2048
#define MAXPWM  127
#define MIDPWM  64
#define MAXPREC 10000

#define STARTMARKER        0x81
#define STOPMARKER         0x8F

#define TIMEOUT 5

#define LINE0(STR) (ST7735_PutStr5x7(5, 5, (STR), ST7735_Color565(0, 0xFF, 0), ST7735_Color565(0x0, 0x0, 0x0), 1))
#define LINE1(STR) (ST7735_PutStr5x7(5, 15, (STR), ST7735_Color565(0, 0xFF, 0), ST7735_Color565(0x0, 0x0, 0x0), 1))
#define LINE2(STR) (ST7735_PutStr5x7(5, 25, (STR), ST7735_Color565(0, 0xFF, 0), ST7735_Color565(0x0, 0x0, 0x0), 1))
#define LINE3(STR) (ST7735_PutStr5x7(5, 35, (STR), ST7735_Color565(0, 0xFF, 0), ST7735_Color565(0x0, 0x0, 0x0), 1))
#define COL1LINE0(STR)  (ST7735_PutStr5x7(80, 5, (STR), ST7735_Color565(0, 0xFF, 0), ST7735_Color565(0x0, 0x0, 0x0), 1))
#define COL2LINE0(STR)  (ST7735_PutStr5x7(115, 5, (STR), ST7735_Color565(0, 0xFF, 0), ST7735_Color565(0x0, 0x0, 0x0), 1))

__IO uint8_t status = (1 << CONNOK);
__IO uint8_t count = 0;
__IO uint16_t sensor_data = 0xFFFF;

uint8_t response[NUM] = { '\0' };
uint8_t cmd[NUM] = { '\0' };
uint8_t buffer[NUM] = { '\0' };

__IO uint8_t i = 0;
__IO uint8_t j = 0;
__IO uint8_t k = 0;
__IO uint8_t p = 0;
__IO uint16_t x, y;
__IO uint8_t drvl = 0x40;
__IO uint8_t drvr = 0x40;
__IO uint8_t tmp_drvl = 0;
__IO uint8_t tmp_drvr = 0;
__IO uint8_t tmp = 0;
__IO uint8_t flg = 0;
__IO uint8_t response_count = 0;

void ADC1_Init(void);
void DisplayStatus(uint8_t*);
void TIM3_NVIC_Configuration(void);
void TIM3_Init(void);
void EXT_INT_Init(void);

ADC_InitTypeDef ADC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;

uint16_t ADCBuffer[] = {0xAAAA, 0xAAAA};

void number2str(uint8_t byte, uint8_t *str)
{
    char hundred = ((3 << 4) | (byte / 100));
    char ten = ((3 << 4) | ((byte % 100) / 10));
    char one = ((3 << 4) | ((byte % 100) % 10));
    if ((sizeof(str) / sizeof(char)) >= 4)
    {
        str[0] = hundred; str[1] = ten; str[2] = one; str[3] = 0;
    }
}

/**
 * @brief  This function handles USARTx global interrupt request.
 * @param  None
 * @retval None
 */
void USART1_IRQHandler(void)
{
    if ((USART1->SR & USART_FLAG_RXNE) != (u16) RESET)
    {
        i = USART_ReceiveData(USART1);

        if (i == STOPMARKER)
        {
            response_count++;
            status = response[0];
            DisplayStatus(&response[0]);
            p = 0;
            flg &= ~(1 << 0);
            count = 0;
        }
        if (flg & (1 << 0))
        {
            if (p < NUM)
            {
                response[p] = i;
                p++;
            }
        }
        if (i == STARTMARKER)
        {
            flg |= (1 << 0);
            p = 0;
        }
    }
}


void DisplayStatus(uint8_t *response)
{
    if (tmp != status)
    {
        tmp = status;
        LINE0("Connected   ");
        if (status & (1 << AUTO_Status)) LINE1("Auto ON "); else LINE1("Auto OFF");
        if (status & (1 << MOVE_Status)) LINE2("Move ON ");
        if (status & (1 << FORWARD_Status)) LINE3("Forward ");
        else if (status & (1 << BACK_Status)) LINE3("Backward");
        else if (status & (1 << LEFT_Status)) LINE3("Left    ");
        else if (status & (1 << RIGHT_Status)) LINE3("Right   ");
        else LINE3("        ");
    }

    if ((tmp_drvl != response[1]) || (tmp_drvr != response[2]))
    {
        tmp_drvl = response[1]; tmp_drvr = response[2];
        buffer[0] = 'L'; buffer[1] = '=';
        number2str(tmp_drvl, &buffer[2]);
        COL1LINE0(&buffer[0]);
        buffer[0] = 'R';
        number2str(tmp_drvr, &buffer[2]);
        COL2LINE0(&buffer[0]);

        //ST7735_FillRect(80, 20, 100, 60, ST7735_Color565(0x60, 0x10, 0x50));
        //ST7735_FillRect(115, 20, 135, 60, ST7735_Color565(0x60, 0x10, 0x50));
    }
}

void TIM3_IRQHandler(void)
{
    // if interrupt happens then do this
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        // clear interrupt and start counting again to get precise freq
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        //enable tim3 to one pulse
        //TIM_Cmd(TIM3,ENABLE);
        y = ADCBuffer[0];
        x = ADCBuffer[1];

        count++;

        if (count > TIMEOUT)
        {
            if (status != 0)
            {
                ST7735_Clear(ST7735_Color565(0x0, 0x0, 0x0));
                LINE0("Disconnected");
            }
            status = 0; count = 0;
        }

        if (status & (1 << AUTO_Status))
        {
            cmd[0] = STARTMARKER;
            cmd[3] = STOPMARKER;
            cmd[1] = 'O'; cmd[2] = 'K';
            UARTSend(&cmd[0], 4);
            if (y > 3000) { cmd[1] = 'F'; cmd[2] = 'R'; UARTSend(&cmd[0], 4); }
            else if (y < 1000) { cmd[1] = 'B'; cmd[2] = 'K'; UARTSend(&cmd[0], 4); }
            else if (x > 3000) { cmd[1] = 'R'; cmd[2] = 'T'; UARTSend(&cmd[0], 4); }
            else if (x < 1000) { cmd[1] = 'L'; cmd[2] = 'F'; UARTSend(&cmd[0], 4); }
        }
        else
        {
            drvl = drvr = y * MAXPWM / MAXVAL;

            if ((x >= MIDVAL) && (x <= MAXVAL))
            {
                tmp = (x * MIDPWM / MIDVAL) - MIDPWM;
                if ((drvl + tmp) > MAXPWM) drvl = MAXPWM; else drvl += tmp;
                if ((drvr - tmp) < 0) drvr = 0; else drvr -= tmp;
            }
            else if ((x >= MINVAL) && (x < MIDVAL))
            {
                tmp = MAXPWM - ((x - MIDVAL) * (-1) * MIDPWM / MIDVAL);
                if ((drvr + tmp) > MAXPWM) drvr = MAXPWM; else drvr += tmp;
                if ((drvl - tmp) < 0) drvl = 0; else drvl -= tmp;
            }

            cmd[0] = STARTMARKER;
            cmd[1] = 'M'; cmd[2] = 'V';
            cmd[3] = drvl; cmd[4] = drvr;
            cmd[5] = STOPMARKER;
            UARTSend(&cmd[0], 6);
        }
    }
}

void EXTI4_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line4) != RESET)
    {
        cmd[0] = STARTMARKER;
        cmd[1] = 'Q'; cmd[2] = 'R';
        cmd[3] = STOPMARKER;
        UARTSend(&cmd[0], 4);
        delay_10ms(6000);
        //
        //Clear the EXTI line 9 pending bit
        //
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}

int main(void)
{

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1
                            | RCC_APB2Periph_GPIOA
                            | RCC_APB2Periph_GPIOB
                            | RCC_APB2Periph_GPIOC, ENABLE);

    EXT_INT_Init();
    ADC1_Init();

    ST7735_Init();
    ST7735_Orientation(1);
    ST7735_Clear(ST7735_Color565(0x0, 0x0, 0x0));

    USART_NVIC_Configuration();
    USART_GPIO_Configuration();
    USART_Configuration();
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    TIM3_NVIC_Configuration();
    TIM3_Init();

    while (1)
    {
        ;
    }
}

void TIM3_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void TIM3_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / 10000) - 1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 1000;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    // TIM IT enable
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    // TIM2 enable counter
    TIM_Cmd(TIM3, ENABLE);
}

void EXT_INT_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);
    // Configure EXTI Line4 to generate an interrupt on rising or falling edge
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void ADC1_Init(void)
{
    // input of ADC (it doesn't seem to be needed, as default GPIO state is floating input)
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3;        // that's ADC1_IN2 and ADC1_IN3 (PA2, PA3 on STM32)
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_InitStructure.DMA_BufferSize = 2;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADCBuffer;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    DMA_Cmd (DMA1_Channel1 , ENABLE) ;

    // clock for ADC (max 14MHz --> 72/6=12MHz)
    RCC_ADCCLKConfig (RCC_PCLK2_Div6);
    // enable ADC system clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    // define ADC config
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;    // we work in continuous sampling mode
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 2;

    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_28Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_28Cycles5);
    ADC_Init (ADC1, &ADC_InitStructure);

    // enable ADC
    ADC_Cmd (ADC1, ENABLE);
    ADC_DMACmd (ADC1 , ENABLE) ;

    //    ADC calibration (optional, but recommended at power on)
    ADC_ResetCalibration(ADC1);    // Reset previous calibration
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);    // Start new calibration (ADC must be off at that time)
    while(ADC_GetCalibrationStatus(ADC1));

    // start conversion
    ADC_Cmd (ADC1, ENABLE);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);    // start conversion (will be endless as we are in continuous mode)
}

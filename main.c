#include "stm32f10x.h"
#include "delay.h"
#include "usart_rxtx.h"
#include "hwdrv.h"
#include "st7735.h"
#include "display.h"

#define NUM 16
#define FAILURES 3

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

#define TIMEOUT 10

#define ARRAYSIZE 3*5

__IO uint8_t count = 0;
__IO uint16_t sensor_data = 0xFFFF;

uint8_t response[NUM] = { '\0' };
uint8_t cmd[NUM] = { '\0' };
uint8_t buffer[NUM] = { '\0' };

__IO uint8_t i = 0;
__IO uint8_t j = 0;
__IO uint8_t k = 0;
__IO uint8_t p = 0;
__IO uint16_t x_coord, y_coord, menu_val;
int16_t tmp_l, tmp_r;
__IO uint8_t drvl = 0x40;
__IO uint8_t drvr = 0x40;
__IO uint8_t tmp_drvl = 0;
__IO uint8_t tmp_drvr = 0;
__IO uint8_t flg = 0;
__IO uint8_t response_count = 0;

void ADC1_Init(void);
void TIM3_NVIC_Configuration(void);
void TIM3_Init(void);
void EXT_INT_Init(void);
void KeyHandler(void);

ADC_InitTypeDef ADC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;

uint16_t ADCBuffer[ARRAYSIZE];

DATA_TypeDef data;

void KeyHandler()
{
    if (menu_val < 50)
    {
        data.currentKey = SW_LEFT;
    } 
    else if (menu_val < 700)
    {
        data.currentKey = SW_UP;
    }
    else if (menu_val < 1500)
    {
        data.currentKey = SW_DOWN;
    }
    else if (menu_val < 2100)
    {
        data.currentKey = SW_RIGHT;
    }
    else if (menu_val < 3100)
    {
        data.currentKey = SW_OK;
    }
}

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
            data.currentStatus = response[0];
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

void TIM3_IRQHandler(void)
{
    // if interrupt happens then do this
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        // clear interrupt and start counting again to get precise freq
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        //enable tim3 to one pulse
        //TIM_Cmd(TIM3,ENABLE);
        y_coord = (ADCBuffer[0] + ADCBuffer[3] + ADCBuffer[6] + ADCBuffer[9] + ADCBuffer[12]) / 5;
        x_coord = (ADCBuffer[1] + ADCBuffer[4] + ADCBuffer[7] + ADCBuffer[10] + ADCBuffer[13]) / 5;
        menu_val = (ADCBuffer[2] + ADCBuffer[5] + ADCBuffer[8] + ADCBuffer[11] + ADCBuffer[14]) / 5;
        
        if (menu_val < 3800) KeyHandler();
        
        count++;

        if (count > TIMEOUT)
        {
            data.currentStatus = 0; count = 0;
        }

        if (data.currentStatus & Status_AUTO)
        {
            data.currentStatus |= Status_OK;
            cmd[0] = STARTMARKER;
            if (y_coord > 3000) { data.currentStatus &= 0xF0; data.currentStatus |= Status_FORWARD; }
            else if (y_coord < 1000) { data.currentStatus &= 0xF0; data.currentStatus |= Status_BACK; }
            else if (x_coord > 3000) { data.currentStatus &= 0xF0; data.currentStatus |= Status_RIGHT; }
            else if (x_coord < 1000) { data.currentStatus &= 0xF0; data.currentStatus |= Status_LEFT; }
            cmd[1] = data.currentStatus;
            cmd[2] = STOPMARKER;
            UARTSend(&cmd[0], 3);
        }
        else
        {
            
            tmp_l = tmp_r = y_coord;
            
            if (x_coord >= MIDVAL)
            {
                tmp_l = y_coord + (x_coord - MIDVAL);
                tmp_r = y_coord - (x_coord - MIDVAL);
            }
            else if (x_coord < MIDVAL)
            {
                tmp_l = y_coord - (MIDVAL - x_coord);
                tmp_r = y_coord + (MIDVAL - x_coord);
            }
            if (tmp_l > MAXVAL) tmp_l = MAXVAL;
            if (tmp_r > MAXVAL) tmp_r = MAXVAL;

            if (tmp_l < 0) tmp_l = 0;
            if (tmp_r < 0) tmp_r = 0;
            drvl = tmp_l * MAXPWM / MAXVAL;
            drvr = tmp_r * MAXPWM / MAXVAL;

            data.currentStatus &= 0xF0;
            cmd[0] = STARTMARKER;
            cmd[1] = data.currentStatus;
            cmd[2] = drvl;
            cmd[3] = drvr;
            cmd[4] = (uint8_t)(x_coord & 0x00FF);
            cmd[5] = (uint8_t)((x_coord & 0xFF00) >> 8);
            cmd[6] = (uint8_t)(y_coord & 0x00FF);
            cmd[7] = (uint8_t)((y_coord & 0xFF00) >> 8);
            cmd[8] = (uint8_t)(menu_val & 0x00FF);
            cmd[9] = (uint8_t)((menu_val & 0xFF00) >> 8);
            cmd[10] = STOPMARKER;
            UARTSend(&cmd[0], 11);
        }

        ShowPage(&data);
    }
}

void EXTI4_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line4) != RESET)
    {
        if (data.currentStatus & Status_AUTO)
            data.currentStatus &= ~(Status_AUTO);
        else
            data.currentStatus |= Status_AUTO;
        cmd[0] = STARTMARKER;
        cmd[1] = data.currentStatus;
        cmd[2] = STOPMARKER;
        UARTSend(&cmd[0], 3);
        delay_10ms(6000);
        //Clear the EXTI line 9 pending bit
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}

int main(void)
{
    
    data.currentPage = DISPLAY_status;
    data.prevKey = SW_NONE;
    data.currentKey = SW_NONE;
    data.prevStatus = 0xFF;
    data.currentStatus = 0;
    data.keyPressed = FALSE;

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
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6;        // that's ADC12_IN2, ADC12_IN3 and ADC12_IN6 (PA2, PA3, PA6 on STM32)
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_InitStructure.DMA_BufferSize = ARRAYSIZE;
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
    ADC_InitStructure.ADC_NbrOfChannel = 3;

    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_28Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_28Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 3, ADC_SampleTime_28Cycles5);
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
